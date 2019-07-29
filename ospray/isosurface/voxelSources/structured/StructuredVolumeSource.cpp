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

#include "StructuredVolumeSource.h"

namespace structured {

  /*! create lits of *all* voxel (refs) we want to be considered for
   * interesction */
  void StructuredVolumeSource::getActiveVoxels(
      std::vector<VoxelSource::VoxelRef> &activeVoxels, float isoValue) const
  {
    volume->filterAllVoxelsThatOverLapIsoValue(
        (std::vector<structured::VoxelRef> &)activeVoxels, isoValue);
  }

  /*! compute world-space bounds for given voxel */
  box3fa StructuredVolumeSource::getVoxelBounds(
      const VoxelSource::VoxelRef voxelRef) const
  {
    const vec3i idx = ((structured::VoxelRef &)voxelRef).asVec3i();
    const vec3f lo  = vec3f(idx) * scaleDims;
    return box3fa(lo, lo + scaleDims);
  }

  /*! get full voxel - bounds and vertex values - for given voxel */
  Impi::Voxel StructuredVolumeSource::getVoxel(
      const VoxelSource::VoxelRef voxelRef) const
  {
    Impi::Voxel voxel;
    const vec3i idx = ((structured::VoxelRef &)voxelRef).asVec3i();
    const vec3f lo  = vec3f(idx) * scaleDims;
    voxel.bounds    = box3fa(lo, lo + scaleDims);
    volume->getVoxel((structured::Voxel &)voxel.vtx, idx);
    return voxel;
  }

}  // namespace structured
