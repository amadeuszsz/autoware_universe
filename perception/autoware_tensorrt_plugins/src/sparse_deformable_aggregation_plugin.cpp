// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/tensorrt_plugins/sparse_deformable_aggregation_plugin.hpp"

#include "autoware/sparse_deformable_aggregation_ops/sparse_deformable_aggregation.hpp"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"

#include <NvInferRuntime.h>
#include <NvInferRuntimePlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <cstdint>
#include <exception>
#include <string>

namespace nvinfer1::plugin
{

namespace ops = autoware::sparse_deformable_aggregation_ops;

SparseDeformableAggregationPlugin::SparseDeformableAggregationPlugin(
  const std::string & name, const SparseDeformableAggregationParameters & params)
: layer_name_{name}, params_{params}
{
  initFieldsToSerialize();
}

std::int32_t SparseDeformableAggregationPlugin::numLevels() const
{
  return static_cast<std::int32_t>(params_.voxel_sizes.size());
}

void SparseDeformableAggregationPlugin::initFieldsToSerialize()
{
  data_to_serialize_.clear();
  data_to_serialize_.emplace_back(
    "grid_dims", params_.grid_dims.data(), PluginFieldType::kINT32, params_.grid_dims.size());
  data_to_serialize_.emplace_back(
    "voxel_sizes", params_.voxel_sizes.data(), PluginFieldType::kFLOAT32,
    params_.voxel_sizes.size());
  data_to_serialize_.emplace_back(
    "grid_origin", params_.grid_origin.data(), PluginFieldType::kFLOAT32,
    params_.grid_origin.size());
  data_to_serialize_.emplace_back(
    "min_occupancy", &params_.min_occupancy, PluginFieldType::kFLOAT32, 1);
  fc_to_serialize_.nbFields = data_to_serialize_.size();
  fc_to_serialize_.fields = data_to_serialize_.data();
}

IPluginCapability * SparseDeformableAggregationPlugin::getCapabilityInterface(
  PluginCapabilityType type) noexcept
{
  try {
    if (type == PluginCapabilityType::kBUILD) {
      return static_cast<IPluginV3OneBuild *>(this);
    }
    if (type == PluginCapabilityType::kRUNTIME) {
      return static_cast<IPluginV3OneRuntime *>(this);
    }
    PLUGIN_ASSERT(type == PluginCapabilityType::kCORE);
    return static_cast<IPluginV3OneCore *>(this);
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

IPluginV3 * SparseDeformableAggregationPlugin::clone() noexcept
{
  try {
    return new SparseDeformableAggregationPlugin{layer_name_, params_};
  } catch (std::exception const & e) {
    caughtError(e);
  }
  return nullptr;
}

char const * SparseDeformableAggregationPlugin::getPluginName() const noexcept
{
  return kSPARSE_DEFORMABLE_AGGREGATION_PLUGIN_NAME;
}

char const * SparseDeformableAggregationPlugin::getPluginVersion() const noexcept
{
  return kSPARSE_DEFORMABLE_AGGREGATION_PLUGIN_VERSION;
}

char const * SparseDeformableAggregationPlugin::getPluginNamespace() const noexcept
{
  return kSPARSE_DEFORMABLE_AGGREGATION_PLUGIN_NAMESPACE;
}

std::int32_t SparseDeformableAggregationPlugin::getNbOutputs() const noexcept
{
  return 1;
}

std::int32_t SparseDeformableAggregationPlugin::configurePlugin(
  DynamicPluginTensorDesc const * in, std::int32_t num_inputs, DynamicPluginTensorDesc const * out,
  std::int32_t num_outputs) noexcept
{
  PLUGIN_ASSERT(num_inputs == INOUT_FIRST_FEATURES_INDEX + 2 * numLevels());
  PLUGIN_ASSERT(num_outputs == 1);
  PLUGIN_ASSERT(in[INOUT_KEY_POINTS_INDEX].desc.dims.nbDims == 4);
  PLUGIN_ASSERT(in[INOUT_WEIGHTS_INDEX].desc.dims.nbDims == 4);
  PLUGIN_ASSERT(out[0].desc.dims.nbDims == 3);
  return 0;
}

bool SparseDeformableAggregationPlugin::supportsFormatCombination(
  std::int32_t pos, DynamicPluginTensorDesc const * in_out, std::int32_t num_inputs,
  [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  const auto & desc = in_out[pos].desc;
  if (desc.format != TensorFormat::kLINEAR) {
    return false;
  }
  const std::int32_t first_grid = INOUT_FIRST_FEATURES_INDEX + numLevels();
  if (pos == INOUT_KEY_POINTS_INDEX || pos == INOUT_WEIGHTS_INDEX) {
    return desc.type == DataType::kFLOAT;
  }
  if (pos < first_grid) {
    // Every stage and the output share the feature type of the first stage
    const auto type = in_out[INOUT_FIRST_FEATURES_INDEX].desc.type;
    return (desc.type == DataType::kFLOAT || desc.type == DataType::kHALF) &&
           (pos == INOUT_FIRST_FEATURES_INDEX || desc.type == type);
  }
  if (pos < num_inputs) {
    return desc.type == DataType::kINT64;
  }
  return desc.type == in_out[INOUT_FIRST_FEATURES_INDEX].desc.type;
}

std::int32_t SparseDeformableAggregationPlugin::getOutputDataTypes(
  DataType * output_types, [[maybe_unused]] std::int32_t num_outputs, DataType const * input_types,
  [[maybe_unused]] std::int32_t num_inputs) const noexcept
{
  output_types[0] = input_types[INOUT_FIRST_FEATURES_INDEX];
  return 0;
}

std::int32_t SparseDeformableAggregationPlugin::getOutputShapes(
  DimsExprs const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DimsExprs const * shape_inputs, [[maybe_unused]] std::int32_t num_shape_inputs,
  DimsExprs * outputs, [[maybe_unused]] std::int32_t num_outputs,
  [[maybe_unused]] IExprBuilder & expr_builder) noexcept
{
  outputs[0].nbDims = 3;
  outputs[0].d[0] = inputs[INOUT_KEY_POINTS_INDEX].d[0];
  outputs[0].d[1] = inputs[INOUT_KEY_POINTS_INDEX].d[1];
  outputs[0].d[2] = inputs[INOUT_FIRST_FEATURES_INDEX].d[1];
  return 0;
}

std::int32_t SparseDeformableAggregationPlugin::enqueue(
  PluginTensorDesc const * input_desc, PluginTensorDesc const * output_desc,
  void const * const * inputs, void * const * outputs, [[maybe_unused]] void * workspace,
  cudaStream_t stream) noexcept
{
  const auto & key_points = input_desc[INOUT_KEY_POINTS_INDEX].dims;
  const auto & weights = input_desc[INOUT_WEIGHTS_INDEX].dims;
  const auto num_anchors = static_cast<std::int32_t>(key_points.d[0] * key_points.d[1]);
  const auto num_points = static_cast<std::int32_t>(key_points.d[2]);
  const auto num_groups = static_cast<std::int32_t>(weights.d[3]);
  const auto channels = static_cast<std::int32_t>(input_desc[INOUT_FIRST_FEATURES_INDEX].dims.d[1]);

  ops::SparseLevels levels{};
  levels.num_levels = numLevels();
  for (std::int32_t level = 0; level < levels.num_levels; ++level) {
    levels.features[level] = inputs[INOUT_FIRST_FEATURES_INDEX + level];
    levels.grids[level] = static_cast<const std::int64_t *>(
      inputs[INOUT_FIRST_FEATURES_INDEX + levels.num_levels + level]);
    for (std::int32_t axis = 0; axis < 3; ++axis) {
      levels.dims[level][axis] = params_.grid_dims[level * 3 + axis];
    }
    levels.voxel_sizes[level] = params_.voxel_sizes[level];
  }

  const auto * key_points_ptr = static_cast<const float *>(inputs[INOUT_KEY_POINTS_INDEX]);
  const auto * weights_ptr = static_cast<const float *>(inputs[INOUT_WEIGHTS_INDEX]);
  cudaError_t status = cudaSuccess;
  if (output_desc[0].type == DataType::kHALF) {
    status = ops::sparse_deformable_aggregation_launch<half>(
      key_points_ptr, weights_ptr, levels, params_.grid_origin.data(), num_anchors, num_points,
      num_groups, channels, params_.min_occupancy, static_cast<half *>(outputs[0]), stream);
  } else {
    status = ops::sparse_deformable_aggregation_launch<float>(
      key_points_ptr, weights_ptr, levels, params_.grid_origin.data(), num_anchors, num_points,
      num_groups, channels, params_.min_occupancy, static_cast<float *>(outputs[0]), stream);
  }
  return status == cudaSuccess ? 0 : 1;
}

std::int32_t SparseDeformableAggregationPlugin::onShapeChange(
  [[maybe_unused]] PluginTensorDesc const * in, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] PluginTensorDesc const * out, [[maybe_unused]] std::int32_t num_outputs) noexcept
{
  return 0;
}

IPluginV3 * SparseDeformableAggregationPlugin::attachToContext(
  [[maybe_unused]] IPluginResourceContext * context) noexcept
{
  return clone();
}

PluginFieldCollection const * SparseDeformableAggregationPlugin::getFieldsToSerialize() noexcept
{
  return &fc_to_serialize_;
}

std::size_t SparseDeformableAggregationPlugin::getWorkspaceSize(
  [[maybe_unused]] DynamicPluginTensorDesc const * inputs, [[maybe_unused]] std::int32_t num_inputs,
  [[maybe_unused]] DynamicPluginTensorDesc const * outputs,
  [[maybe_unused]] std::int32_t num_outputs) const noexcept
{
  return 0;
}

}  // namespace nvinfer1::plugin
