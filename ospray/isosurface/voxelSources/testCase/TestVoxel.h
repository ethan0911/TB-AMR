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

namespace testCase {

  /*! implements a simple test case consisting of a single voxel,
      with fixed values, at a fixed location */
  struct TestVoxel : public Impi::VoxelSource
  {
    /*! create lits of *all* voxel (refs) we want to be considered for
     * interesction */
    virtual void getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                 float isoValue) const override
    {
      activeVoxels.clear();
      activeVoxels.push_back(0);
    };

    /*! compute world-space bounds for given voxel */
    virtual box3fa getVoxelBounds(const VoxelRef voxelRef) const override
    {
      return box3fa(vec3fa(0.f), vec3f(1.f));
      // return box3fa(vec3fa(11,13,15.f), vec3f(13,14,17));
    };

    /*! get full voxel - bounds and vertex values - for given voxel */
    virtual Impi::Voxel getVoxel(const VoxelRef voxelRef) const override
    {
      Impi::Voxel voxel;
      voxel.bounds       = getVoxelBounds(voxelRef);
      voxel.vtx[0][0][0] = 0.f;
      voxel.vtx[0][0][1] = 1.f;
      voxel.vtx[0][1][0] = 10.f;
      voxel.vtx[0][1][1] = 0.f;
      voxel.vtx[1][0][0] = 10.f;
      voxel.vtx[1][0][1] = 0.f;
      voxel.vtx[1][1][0] = 0.f;
      voxel.vtx[1][1][1] = 100.f;
      return voxel;
    };
  };

}  // namespace testCase