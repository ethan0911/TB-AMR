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

#include "TestAMR.h"

namespace testCase {

  const float TestAMR::baseLevel[3][3][3] = {
      1, 0, 0, 1, 0, 0, 1,      0, 0,

      3, 2, 2, 3, 2, 2, 3.0001, 2, 2,

      5, 4, 4, 5, 4, 4, 5,      4, 4,
  };

  const float TestAMR::fineLevel[3][3][3] = {
      /*! this refines the top right back voxel (the {2/2/2/2}{4/4/4/4} one) */
      2, 2, 2, 2, 2, 2, 2, 2, 2,

      3, 3, 3, 3, 6, 3, 3, 3, 3,

      4, 4, 4, 4, 4, 4, 4, 4, 4,
  };

  vec4i TestAMR::getIdx(const uint64_t vr) const
  {
    vec4i idx;
    int cellID;
    if (vr >= 7) {
      idx.w  = 1;
      cellID = vr - 7;
    } else {
      idx.w  = 0;
      cellID = vr;
    }
    idx.x = (cellID >> 0) & 1;
    idx.y = (cellID >> 1) & 1;
    idx.z = (cellID >> 2) & 1;
    return idx;
  }

  /*! create lits of *all* voxel (refs) we want to be considered for
   * interesction */
  void TestAMR::getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                float isoValue) const
  {
    // the whole thing has 7 coarse cells and 8 fine cells ... 15 total
    activeVoxels.clear();
    for (int i = 0; i < 15; i++)
      activeVoxels.push_back(i);
  }

  /*! compute world-space bounds for given voxel */
  box3fa TestAMR::getVoxelBounds(const VoxelRef voxelRef) const
  {
    vec4i idx = getIdx(voxelRef);
    if (idx.w) {
      vec3f lo = vec3f((const vec3i &)idx) * 0.5f + vec3f(1.f);
      vec3f hi = lo + vec3f(0.5f);
      return box3fa(lo, hi);
    } else {
      vec3f lo = vec3f((const vec3i &)idx);
      vec3f hi = lo + vec3f(1.f);
      return box3fa(lo, hi);
    }
  }

  /*! get full voxel - bounds and vertex values - for given voxel */
  Impi::Voxel TestAMR::getVoxel(const VoxelRef voxelRef) const
  {
    Impi::Voxel voxel;
    vec4i idx = getIdx(voxelRef);
    if (idx.w) {
      vec3f lo     = vec3f((const vec3i &)idx) * 0.5f + vec3f(1.f);
      vec3f hi     = lo + vec3f(0.5f);
      voxel.bounds = box3fa(lo, hi);
      array3D::for_each(vec3i(2), [&](const vec3i vtx) {
        voxel.vtx[vtx.z][vtx.y][vtx.x] =
            fineLevel[idx.z + vtx.z][idx.y + vtx.y][idx.x + vtx.x];
      });
    } else {
      vec3f lo     = vec3f((const vec3i &)idx);
      vec3f hi     = lo + vec3f(1.f);
      voxel.bounds = box3fa(lo, hi);
      array3D::for_each(vec3i(2), [&](const vec3i vtx) {
        voxel.vtx[vtx.z][vtx.y][vtx.x] =
            baseLevel[idx.z + vtx.z][idx.y + vtx.y][idx.x + vtx.x];
      });
    }
    return voxel;
  }
}  // namespace testCase
