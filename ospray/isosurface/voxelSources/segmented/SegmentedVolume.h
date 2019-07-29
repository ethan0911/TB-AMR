// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

namespace ospray {
  namespace impi {
    namespace structured {

struct SegmentedVolume : public Impi::VoxelSource {
        typedef uint64_t VoxelRef;
        virtual getActiveVoxels(std::vector<VoxelRef> &activeVoxels, float isoValue) const = 0;
        virtual box3f getBounds(const VoxelRef voxelRef) const = 0;
        virtual Voxel getVoxel(const VoxelRef voxelRef) const = 0;

std::shared_ptr<LogicalVolume> volume;
      // hack for lukas: filter segmentation volume
      std::shared_ptr<LogicalVolume> seg;
      };

    }
  }
}

      std::cout << "loading test data-set, and testing generation of iso-voxels" << std::endl;
      volume = VolumeT<float>::loadRAW("density_064_064_2.0.raw",vec3i(64));
      seg    = VolumeT<float>::loadRAW("density_064_064_2.0_seg.raw",vec3i(64));

      volume->filterVoxels(hotVoxels,[&](const LogicalVolume *v, const vec3i &idx) {
          return
            (volume->getVoxel(idx).getRange().contains(isoValue)
             &&
             seg->getVoxel(idx).getRange().contains(128)
             );
        });
      
