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

#include "autoware/tensorrt_plugins/sparse_deformable_aggregation_plugin_creator.hpp"

#include "autoware/sparse_deformable_aggregation_ops/sparse_deformable_aggregation.hpp"
#include "autoware/tensorrt_plugins/plugin_utils.hpp"
#include "autoware/tensorrt_plugins/sparse_deformable_aggregation_plugin.hpp"

#include <NvInferRuntimePlugin.h>

#include <exception>
#include <string>

namespace nvinfer1::plugin
{

REGISTER_TENSORRT_PLUGIN(SparseDeformableAggregationPluginCreator);

SparseDeformableAggregationPluginCreator::SparseDeformableAggregationPluginCreator()
{
  plugin_attributes_.clear();
  plugin_attributes_.emplace_back("grid_dims", nullptr, PluginFieldType::kINT32, 0);
  plugin_attributes_.emplace_back("voxel_sizes", nullptr, PluginFieldType::kFLOAT32, 0);
  plugin_attributes_.emplace_back("grid_origin", nullptr, PluginFieldType::kFLOAT32, 3);
  plugin_attributes_.emplace_back("min_occupancy", nullptr, PluginFieldType::kFLOAT32, 1);
  fc_.nbFields = plugin_attributes_.size();
  fc_.fields = plugin_attributes_.data();
}

nvinfer1::PluginFieldCollection const *
SparseDeformableAggregationPluginCreator::getFieldNames() noexcept
{
  return &fc_;
}

IPluginV3 * SparseDeformableAggregationPluginCreator::createPlugin(
  char const * name, PluginFieldCollection const * fc,
  [[maybe_unused]] TensorRTPhase phase) noexcept
{
  try {
    PLUGIN_VALIDATE(fc != nullptr);
    SparseDeformableAggregationParameters params{};
    for (std::int32_t i = 0; i < fc->nbFields; ++i) {
      const auto & field = fc->fields[i];
      const std::string field_name = field.name;
      if (field_name == "grid_dims") {
        PLUGIN_VALIDATE(field.type == PluginFieldType::kINT32);
        const auto * data = static_cast<const std::int32_t *>(field.data);
        params.grid_dims.assign(data, data + field.length);
      } else if (field_name == "voxel_sizes") {
        PLUGIN_VALIDATE(field.type == PluginFieldType::kFLOAT32);
        const auto * data = static_cast<const float *>(field.data);
        params.voxel_sizes.assign(data, data + field.length);
      } else if (field_name == "grid_origin") {
        PLUGIN_VALIDATE(field.type == PluginFieldType::kFLOAT32);
        const auto * data = static_cast<const float *>(field.data);
        params.grid_origin.assign(data, data + field.length);
      } else if (field_name == "min_occupancy") {
        PLUGIN_VALIDATE(field.type == PluginFieldType::kFLOAT32);
        params.min_occupancy = static_cast<const float *>(field.data)[0];
      }
    }
    PLUGIN_VALIDATE(!params.voxel_sizes.empty());
    PLUGIN_VALIDATE(
      params.voxel_sizes.size() <=
      static_cast<std::size_t>(autoware::sparse_deformable_aggregation_ops::kMaxLevels));
    PLUGIN_VALIDATE(params.grid_dims.size() == 3 * params.voxel_sizes.size());
    PLUGIN_VALIDATE(params.grid_origin.size() == 3);
    PLUGIN_VALIDATE(params.min_occupancy > 0.0F);
    return new SparseDeformableAggregationPlugin(std::string(name), params);
  } catch (std::exception & e) {
    caughtError(e);
  }
  return nullptr;
}

}  // namespace nvinfer1::plugin
