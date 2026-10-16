# autoware_ptv3

## Purpose

The `autoware_ptv3` package is used for 3D lidar segmentation and 3D object detection with tracking.

## Inner-workings / Algorithms

This package implements a TensorRT powered inference node for Point Transformers V3 (PTv3) [1].
The sparse convolution backend corresponds to [spconv](https://github.com/traveller59/spconv).
Autoware installs it automatically in its setup script. If needed, the user can also build it and install it following the [following instructions](https://github.com/autowarefoundation/spconv_cpp).

The network input is the current lidar frame, every point carrying `(x, y, z, intensity)`. The
cloud is voxelized on the GPU into padded voxels (`max_points_per_voxel` slots holding the first
points of each voxel in input order) together with the valid point count per voxel, and the encoder
graph averages the points of every voxel itself, so training and inference share one voxel feature
definition. Voxel cells are counted from the lower border of `point_cloud_range` as in the training
voxelizer. The temporal context comes from the instance memory of the detection head.

The detection head is a Sparse4D query head. Every query is one object instance that samples the
sparse encoder features around its box, on the encoder stages the model declares. The sampling runs
in the `SparseDeformableAggregation` plugin of `autoware_tensorrt_plugins`. The head keeps a
memory of the most confident instances and carries it to the next frame: the memory is warped by
the ego motion between the frames and read by the instances of the new frame, so an object keeps
its instance from frame to frame. A frame gap outside `(0, max_time_interval]` starts a new stream
with an empty memory. Every instance is decoded into one box of its best class, scored by its
class probability times its centerness. Boxes below the score threshold of their class and distance
are dropped and the classes of a circle NMS group suppress each other within the group radius.

An instance whose score reaches `track_birth_score` starts a track, and the track follows the
instance while the memory carries it. Detected boxes are published as `DetectedObjects`, and those
holding a track are published again as `TrackedObjects` whose `object_id` identifies the track.

## Inputs / Outputs

### Input

| Name                 | Type                            | Description             |
| -------------------- | ------------------------------- | ----------------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | Input pointcloud topic. |

### Output

| Name                                   | Type                                                | Description                                                         |
| -------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------- |
| `~/output/pointcloud/segmentation`     | `sensor_msgs::msg::PointCloud2`                     | `PointXYZCPE` cloud with class ID, probability, and entropy fields. |
| `~/output/pointcloud/visualization`    | `sensor_msgs::msg::PointCloud2`                     | XYZ cloud with RGB field.                                           |
| `~/output/pointcloud/filtered`         | `sensor_msgs::msg::PointCloud2`                     | Filtered cloud in the requested `filter.output_format`.             |
| `~/output/objects`                     | `autoware_perception_msgs::msg::DetectedObjects`    | Detected 3D objects after score filtering and circle NMS.           |
| `~/output/tracked_objects`             | `autoware_perception_msgs::msg::TrackedObjects`     | Detected 3D objects holding a track, identified by their track.     |
| `debug/cyclic_time_ms`                 | `autoware_internal_debug_msgs::msg::Float64Stamped` | Cyclic time (ms).                                                   |
| `debug/pipeline_latency_ms`            | `autoware_internal_debug_msgs::msg::Float64Stamped` | Pipeline latency time (ms).                                         |
| `debug/processing_time/preprocess_ms`  | `autoware_internal_debug_msgs::msg::Float64Stamped` | Preprocess (ms).                                                    |
| `debug/processing_time/inference_ms`   | `autoware_internal_debug_msgs::msg::Float64Stamped` | Inference time (ms).                                                |
| `debug/processing_time/postprocess_ms` | `autoware_internal_debug_msgs::msg::Float64Stamped` | Postprocess time (ms).                                              |
| `debug/processing_time/total_ms`       | `autoware_internal_debug_msgs::msg::Float64Stamped` | Total processing time (ms).                                         |

#### Segmentation point cloud format

The `~/output/pointcloud/segmentation` topic uses the naturally aligned `PointXYZCPE` point type.
Its `point_step` is 24 bytes.

| Field         | Data type | Offset | Description                                                                                                                |
| ------------- | --------- | -----: | -------------------------------------------------------------------------------------------------------------------------- |
| `x`           | `FLOAT32` |      0 | X coordinate in meters.                                                                                                    |
| `y`           | `FLOAT32` |      4 | Y coordinate in meters.                                                                                                    |
| `z`           | `FLOAT32` |      8 | Z coordinate in meters.                                                                                                    |
| `class_id`    | `UINT8`   |     12 | Predicted class encoded as `autoware::point_types::PointCloudClassification`; `INVALID` (255) represents an invalid label. |
| `probability` | `FLOAT32` |     16 | Probability of the predicted class.                                                                                        |
| `entropy`     | `FLOAT32` |     20 | Shannon entropy normalized by the logarithm of the number of classes.                                                      |

Bytes 13 through 15 are alignment padding. The output is intentionally not packed so that the
floating-point fields remain naturally aligned on both the CPU and GPU.

Points with an invalid label (`class_id` = `PointCloudClassification::INVALID`, that is 255) have no
class distribution: their `probability` is `0.0` and their `entropy` is `NaN`, which is the default
value of `autoware::point_types::PointXYZCPE::entropy` and means "not available". Consumers must
therefore guard against `NaN` before using the `entropy` field.

## Parameters

### PTv3Node node

{{ json_to_markdown("perception/autoware_ptv3/schema/ptv3.schema.json") }}

### PTv3Node model

{{ json_to_markdown("perception/autoware_ptv3/schema/ml_package_ptv3.schema.json") }}

`filter.*` and `class_mapping.*` parameters are configured in `config/ptv3.param.yaml`, while
class metadata and the visualization `palette` are configured in
`config/ml_package_ptv3_seg3d_head.param.yaml`.

`segmentation3d.class_mapping` maps each segmentation class name to the
`autoware::point_types::PointCloudClassification` value published in the segmentation point cloud's
`class_id` field. Its keys must match `segmentation3d.class_names` exactly, and every entry of
`class_names` must be present — the node fails to start otherwise, naming the missing class. This
makes the consolidation of model classes (for example `traffic_cone`, `debris` and `vertical_thin`
into `HAZARD`) a configuration choice rather than a compile-time constant.

### The `build_only` option

The `autoware_ptv3` node has a `build_only` option to build the TensorRT engine file from the specified ONNX file, after which the program exits.

```bash
ros2 launch autoware_ptv3 ptv3.launch.xml build_only:=true
```

### The `log_level` option

The default logging severity level for `autoware_ptv3` is `info`. For debugging purposes, the developer may decrease severity level using `log_level` parameter:

```bash
ros2 launch autoware_ptv3 ptv3.launch.xml log_level:=debug
```

## Assumptions / Known limits

This node detects the input pointcloud format automatically on the first received message and
supports:

- `XYZIRCAEDT` (10 fields)
- `XYZIRADRT` (9 fields)
- `XYZIRC` (6 fields)
- `XYZI` (4 fields)

The filtered output cloud format is controlled by `filter.output_format`. When it is set to an
empty string, the filtered output preserves the same format as the input cloud.

The detection memory requires the transform between `world_frame_id` and the lidar frame, frames
are skipped while the transform is unavailable.

Tracked objects are expressed in the lidar frame like the detected objects. Their twist is the
velocity the detection head estimates, their covariances are not estimated.

Track ids restart with the node. The `object_id` of a track starts with the track id and ends with
a random identifier of the node session, so the ids of different runs never collide and the
short ids RViz prints differ between tracks.

## Trained Models

The model was trained on the T4Dataset using approximately 4,000 frames and is available in the Autoware artifacts.

## Troubleshooting

### `Fail to create host memory`

This error may occur when TensorRT cannot satisfy the memory requirements for building an engine. A `workspace_size` that is too small can prevent TensorRT from using the tactics needed for the configured input profiles. Conversely, a `workspace_size` that is too large can allow memory usage to exceed the GPU's available VRAM. A large maximum value in `encoder.voxels_num` also increases the size of TensorRT profiles and intermediate buffers.

Adjust the `workspace_size` values and the maximum `encoder.voxels_num` value in `config/ptv3.param.yaml` to fit the available GPU memory. Finding a suitable balance between these parameters can resolve the issue.

### GPU memory and `encoder.pooled_voxels_num_max`

`encoder.pooled_voxels_num_max` bounds each pooled encoder level's TensorRT profile and buffers (one entry per `encoder.pooling_strides` entry). Set it from measured per-stage peaks with a margin. A frame whose level would exceed its bound is truncated: the voxels with the largest serialization codes are dropped until every level fits, and a warning is logged.

## References/External links

[1] Xiaoyang Wu, Li Jiang, Peng-Shuai Wang, Zhijian Liu, Xihui Liu, Yu Qiao, Wanli Ouyang, Tong He, and Hengshuang Zhao. "Point Transformer V3: Simpler, Faster, Stronger." 2024 Conference on Computer Vision and Pattern Recognition. <!-- cspell:disable-line -->
