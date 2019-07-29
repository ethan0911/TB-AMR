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

#include "SegmentedVolumeSource.h"

namespace structured {

  /*! create lits of *all* voxel (refs) we want to be considered for
   * interesction */
  void SegmentedVolumeSource::getActiveVoxels(
      std::vector<VoxelSource::VoxelRef> &activeVoxels, float isoValue) const
  {
    activeVoxels.clear();
    volume->filterVoxels(
        (std::vector<structured::VoxelRef> &)activeVoxels,
        [&](LogicalVolume *, const vec3i &idx) {
          return this->volume->getVoxel(idx).getRange().contains(isoValue) &&
                 this->segVol->getVoxel(idx).getRange().contains(this->segment);
        });
  }

}  // namespace structured
