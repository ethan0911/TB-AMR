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

#include "ospcommon/array3D/for_each.h"
#include "../../geometry/Impi.h"
#include "./Volume.h"

namespace structured {

  /*! implements a simple (vertex-cenetred) AMR test case
    consisting of a 2x2x2-cell base level in which one of the
    cells is refined into another 2x2x2-cell finer level */
  struct StructuredVolumeSource : public Impi::VoxelSource
  {
    StructuredVolumeSource(
        std::shared_ptr<structured::LogicalVolume> volume)
        : volume(volume),
          dims(volume->getDims()),
          scaleDims(rcp(vec3f(volume->getDims()) - vec3f(1.f)))
    {
    };

    /*! create lits of *all* voxel (refs) we want to be considered for
     * interesction */
    virtual void getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                 float isoValue) const override;

    /*! compute world-space bounds for given voxel */
    virtual box3fa getVoxelBounds(const VoxelRef voxelRef) const override;

    /*! get full voxel - bounds and vertex values - for given voxel */
    virtual Impi::Voxel getVoxel(const VoxelRef voxelRef) const override;

    std::shared_ptr<structured::LogicalVolume> volume;
    const vec3i dims;
    const vec3f scaleDims;
  };

}  // namespace structured
