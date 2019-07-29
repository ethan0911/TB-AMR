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

#include "TestTAMR.h"
#include "ospcommon/tasking/parallel_for.h"


namespace testCase {

  extern "C" void externC_ExatracVoxelValues(void *_cVoxels,
                                             void *_cVoxelRange,
                                             const int index,
                                             const float v0,
                                             const float v1,
                                             const float v2,
                                             const float v3,
                                             const float v4,
                                             const float v5,
                                             const float v6,
                                             const float v7,
                                             const float r1,
                                             const float r2)
  {
    auto c_voxel                = (Voxel *)_cVoxels;
    auto c_voxelRange           = (vec2f *)_cVoxelRange;
    c_voxel[index].vtx[0][0][0] = v0;
    c_voxel[index].vtx[0][0][1] = v1;
    c_voxel[index].vtx[0][1][0] = v2;
    c_voxel[index].vtx[0][1][1] = v3;
    c_voxel[index].vtx[1][0][0] = v4;
    c_voxel[index].vtx[1][0][1] = v5;
    c_voxel[index].vtx[1][1][0] = v6;
    c_voxel[index].vtx[1][1][1] = v7;
    c_voxelRange[index].x       = r1;
    c_voxelRange[index].y       = r2;
  }

  extern "C" void externC_ExatracActiveVoxels(void *_activeVoxels,                                          
                                              const float v0,
                                              const float v1,
                                              const float v2,
                                              const float v3,
                                              const float v4,
                                              const float v5,
                                              const float v6,
                                              const float v7,
                                              const float b1,
                                              const float b2,
                                              const float b3,
                                              const float b4,
                                              const float b5,
                                              const float b6)
  {
    auto c_activeVoxel = (std::vector<Voxel> *)_activeVoxels;
    const box3fa box(vec3f(b1, b2, b3), vec3f(b4, b5, b6));
    c_activeVoxel->emplace_back();
    c_activeVoxel->back().vtx[0][0][0] = v0;
    c_activeVoxel->back().vtx[0][0][1] = v1;
    c_activeVoxel->back().vtx[0][1][0] = v2;
    c_activeVoxel->back().vtx[0][1][1] = v3;
    c_activeVoxel->back().vtx[1][0][0] = v4;
    c_activeVoxel->back().vtx[1][0][1] = v5;
    c_activeVoxel->back().vtx[1][1][0] = v6;
    c_activeVoxel->back().vtx[1][1][1] = v7;
    c_activeVoxel->back().bounds       = box;
  }

  TestTAMR::TestTAMR(TAMRVolume *tamrVolume,
                     const voxel *inputVoxels,
                     size_t numVoxels,
                     float isoValue)
  {
    // this->voxels.resize(numVoxels);
    // this->voxelVRange.resize(numVoxels);
    // tasking::parallel_for(numVoxels, [&](size_t vid) {
    //   this->voxels[vid].bounds =
    //       box3fa(inputVoxels[vid].lower,
    //              inputVoxels[vid].lower + vec3f(inputVoxels[vid].width));
    // });

    size_t numOcts = numVoxels * 8;
    PRINT(numVoxels);
    this->voxels.resize(numOcts);
    this->voxelVRange.resize(numOcts);
    tasking::parallel_for(numVoxels, [&](size_t vid) {

      float width = inputVoxels[vid].width;
      float halfW = 0.5 * width;

      for (int i = 0; i < 8; i++) {
        bool x = (i & 1), y = (i & 2), z = (i & 4);
        vec3f lower = inputVoxels[vid].lower +
                      vec3f(x * halfW, y * halfW, z * halfW);

        // hack for exajet data;
        // vec3f lower = inputVoxels[vid].lower +
        //               vec3f(x * halfW, y * halfW, z * halfW) +
        //               vec3f(-1.73575, -9.44, -3.73281);
        this->voxels[vid * 8 + i].bounds = box3fa(lower, lower + vec3f(halfW));
      }
    });

    std::cout << "Start to calculate the voxel value! VoxelNUM:"
              << this->voxels.size() << std::endl;

    time_point t1 = Time();

    tasking::parallel_for(numOcts, [&](size_t vid) {
      ispc::getVoxelValue(tamrVolume->getIE(),
                          (ispc::Voxel*)&this->voxels[vid],
                          (ispc::vec2f*)&this->voxelVRange[vid]);
    });

    // tasking::parallel_for(numOcts, [&](size_t vid) {
    //   ispc::getVoxelValue2(tamrVolume->getIE(),
    //                       (void *)&this->voxels[vid],
    //                       (void *)&this->voxelVRange[vid]);
    // });

    double calVoxelValueTime = Time(t1);

    // ispc::getVoxelsValues(tamrVolume->getIE(),
    //                       (void*)this->voxels.data(),
    //                       (void*)this->voxelVRange.data(),
    //                       &this->aVoxels,
    //                       this->voxels.size(),
    //                       isoValue);

    std::cout << green
              << "End to calculate the voxel value! takes:" << calVoxelValueTime
              << reset << std::endl;

    // for (int i = 0; i < 8; i++) {
    //   printf("value: [%f,%f,%f,%f,%f,%f,%f,%f]\n",
    //          this->voxels[i].vtx[0][0][0],
    //          this->voxels[i].vtx[0][0][1],
    //          this->voxels[i].vtx[0][1][0],
    //          this->voxels[i].vtx[0][1][1],
    //          this->voxels[i].vtx[1][0][0],
    //          this->voxels[i].vtx[1][0][1],
    //          this->voxels[i].vtx[1][1][0],
    //          this->voxels[i].vtx[1][1][1]);
    //   printf(
    //       "range: [%f,%f]\n", this->voxelVRange[i].x, this->voxelVRange[i].y);
    // }
  }

  TestTAMR::~TestTAMR() {}

  /*! create lits of *all* voxel (refs) we want to be considered for
   * interesction */
  void TestTAMR::getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                 float isoValue) const
  {
    // the whole thing has 7 coarse cells and 8 fine cells ... 15 total
    activeVoxels.clear();
    size_t sIdx = 0; 
    size_t eIdx = this->voxels.size();//32;
    for (size_t i = sIdx; i < eIdx; i++) {
      if (this->voxelVRange[i].x <= isoValue &&
          this->voxelVRange[i].y >= isoValue)
        activeVoxels.push_back(i);
    }
    // PRINT(activeVoxels.size());

    // for (int i = 0; i < this->aVoxels.size(); ++i) {
    //   activeVoxels.push_back(i);
    // }
    // activeVoxels.push_back(0);
  }

  /*! compute world-space bounds for given voxel */
  box3fa TestTAMR::getVoxelBounds(const VoxelRef voxelRef) const
  {
    // return box3fa(vec3f(0.f), vec3f(2.f,2.f,2.f));
    // return this->aVoxels[voxelRef].bounds;
    return this->voxels[voxelRef].bounds;

  }

  /*! get full voxel - bounds and vertex values - for given voxel */
  Impi::Voxel TestTAMR::getVoxel(const VoxelRef voxelRef) const
  {
    // Impi::Voxel voxel;
    // voxel.bounds       = getVoxelBounds(voxelRef);
    // // voxel.vtx[0][0][0] = 0.f;
    // // voxel.vtx[0][0][1] = 1.f;
    // // voxel.vtx[0][1][0] = 10.f;
    // // voxel.vtx[0][1][1] = 0.f;
    // // voxel.vtx[1][0][0] = 10.f;
    // // voxel.vtx[1][0][1] = 0.f;
    // // voxel.vtx[1][1][0] = 0.f;
    // // voxel.vtx[1][1][1] = 100.f;
    // voxel.vtx[0][0][0] = 4.f;
    // voxel.vtx[0][0][1] = 4.f;
    // voxel.vtx[0][1][0] = 8.f;
    // voxel.vtx[0][1][1] = 8.f;
    // voxel.vtx[1][0][0] = 4.f;
    // voxel.vtx[1][0][1] = 4.f;
    // voxel.vtx[1][1][0] = 8.f;
    // voxel.vtx[1][1][1] = 8.f;
    // return voxel;
    // return this->aVoxels[voxelRef];
    return this->voxels[voxelRef];
  }
}  // namespace testCase
