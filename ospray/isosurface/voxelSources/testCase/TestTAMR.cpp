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


  extern "C" void
  externC_ExatracAllVoxelValues(void *_cVoxels,
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

  extern "C" void externC_ExatracActiveVoxels(void *C_actVoxels,
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
    auto c_activeVoxels = (std::vector<Voxel> *)C_actVoxels;
    const box3fa box(vec3f(b1, b2, b3), vec3f(b4, b5, b6));

    c_activeVoxels->emplace_back();
    c_activeVoxels->back().vtx[0][0][0] = v0;
    c_activeVoxels->back().vtx[0][0][1] = v1;
    c_activeVoxels->back().vtx[0][1][0] = v2;
    c_activeVoxels->back().vtx[0][1][1] = v3;
    c_activeVoxels->back().vtx[1][0][0] = v4;
    c_activeVoxels->back().vtx[1][0][1] = v5;
    c_activeVoxels->back().vtx[1][1][0] = v6;
    c_activeVoxels->back().vtx[1][1][1] = v7;
    c_activeVoxels->back().bounds       = box;
  }

#define SIMD_ONLY 0
#define BUILD_ALL 0

  TestTAMR::TestTAMR(TAMRVolume *tamrVolume,
                     const voxel *inputVoxels,
                     size_t numVoxels,
                     float isoValue)
  {
#if SIMD_ONLY
    this->voxels.resize(numVoxels);
    this->voxelVRange.resize(numVoxels);
    tasking::parallel_for(numVoxels, [&](size_t vid) {
      this->voxels[vid].bounds =
          box3fa(inputVoxels[vid].lower,
                 inputVoxels[vid].lower + vec3f(inputVoxels[vid].width));
    });

    time_point t1 = Time();

#if BUILD_ALL
    ispc::build_allVoxels(tamrVolume->getIE(),
                          (void *)this->voxels.data(),
                          (void *)this->voxelVRange.data(),
                          this->voxels.size());
    double calVoxelValueTime = Time(t1);
    std::cout << green << "SIMD Only! Build all voxel value takes:" << calVoxelValueTime
              << reset << std::endl;
#else
    ispc::build_activeVoxels(tamrVolume->getIE(),
                             (void *)this->voxels.data(),
                             (void *)&this->actVoxels,
                             this->voxels.size(),
                             isoValue);
    double calVoxelValueTime = Time(t1);
    std::cout << green << "SIMD Only! Build active voxel value takes:" << calVoxelValueTime
              << reset << std::endl;

#endif

#else
  // NOTICE: for finest method, each voxel should be divided into voxel that has
  // the same width with the octant in the finest voxels.

  // size_t numOcts = 256;
  // PRINT(numVoxels);
  // this->voxelVRange.resize(numOcts);
  // const float minWidth = 0.5;

  // for (int i = 0; i < numVoxels; i++) {
  //   float width = inputVoxels[i].width;
  //   vec3f lower = inputVoxels[i].lower;
  //   float halfWidth = 0.5 * width;
  //   halfWidth = minWidth;

  //   float N = width / halfWidth;

  //   for (int x = 0; x < N; x++)
  //     for (int y = 0; y < N; y++)
  //       for (int z = 0; z < N; z++) {
  //         vec3f curLower = lower + vec3f(x,y,z) * halfWidth;
  //         Voxel vxl;
  //         vxl.bounds = box3fa(curLower, curLower + vec3f(minWidth));
  //         this->voxels.push_back(vxl);
  //       }
  // }
  // PRINT(this->voxels.size());

#if BUILD_ALL
    size_t numOcts = numVoxels * 8;
    PRINT(numVoxels);
    this->voxels.resize(numOcts);
    this->voxelVRange.resize(numOcts);
    tasking::parallel_for(numVoxels, [&](size_t vid) {

      float width = inputVoxels[vid].width;
      float halfW = 0.5 * width;

      for (int i = 0; i < 8; i++) {
        bool x = (i & 1), y = (i & 2), z = (i & 4);
        vec3f lower =
            inputVoxels[vid].lower + vec3f(x * halfW, y * halfW, z * halfW);
        this->voxels[vid * 8 + i].bounds = box3fa(lower, lower + vec3f(halfW));
      }
    });

    std::cout << "Start to calculate the voxel value! VoxelNUM:"
              << this->voxels.size() << std::endl;

    time_point t1 = Time();

    tasking::parallel_for(numOcts, [&](size_t vid) {
      ispc::getVoxelValue(tamrVolume->getIE(),
                          (ispc::Voxel *)&this->voxels[vid],
                          (ispc::vec2f *)&this->voxelVRange[vid]);
    });

    double calVoxelValueTime = Time(t1);
    std::cout << green
              << "SIMD and TBB! Build all voxel value takes:" << calVoxelValueTime
              << reset << std::endl;
#else

    // numVoxels = 4000000;// 0.02 * numVoxels;
    // numVoxels = 0.02 * numVoxels;

    std::cout << "Start to calculate the voxel value! VoxelNUM:" << numVoxels << std::endl;
    time_point t1 = Time();

    auto filterMethodEnv =
        utility::getEnvVar<std::string>("OSPRAY_TAMR_METHOD");
    std::string filterMethod = filterMethodEnv.value_or("nearest");

    int fMethod = NEAREST;

    if (filterMethod == "nearest")
      fMethod = NEAREST;
    else if (filterMethod == "current")
      fMethod = CURRENT;
    else if (filterMethod == "finest")
      fMethod = FINEST;
    else if (filterMethod == "octant")
      fMethod = OCTANT;
    else if (filterMethod == "trilinear")
      fMethod = TRILINEAR;

    auto activeVoxelsContainer = new std::vector<Voxel>[numVoxels];
    tasking::parallel_for(numVoxels, [&](size_t vid) {

      float localCellWidth = inputVoxels[vid].width / tamrVolume->gridWorldSpace.x;
      
      // This determines how many decents are divided from a parent cell when build the active voxels
      uint32 refineFactor = 2; 
      if(fMethod == FINEST)
        refineFactor = 2 * localCellWidth;

      ispc::build_activeVoxel(tamrVolume->getIE(),
                              (ispc::vec3f &)inputVoxels[vid].lower,
                              inputVoxels[vid].value,
                              localCellWidth,
                              refineFactor,
                              (void *)&activeVoxelsContainer[vid],
                              isoValue,
                              fMethod);
    });
    double calVoxelValueTime = Time(t1);

    std::cout << green
              << "SIMD and TBB! Build active voxel value takes:" << calVoxelValueTime
              << reset << std::endl;

    std::vector<size_t> begin(numVoxels, size_t(0));
    size_t n(0);
    for (int vid = 0; vid < numVoxels; ++vid) {
      begin[vid] = n;
      n += activeVoxelsContainer[vid].size();
    }
    this->actVoxels.resize(n);
    tasking::parallel_for(numVoxels, [&](const size_t vid) {
      std::copy(activeVoxelsContainer[vid].begin(),
                activeVoxelsContainer[vid].end(),
                &this->actVoxels[begin[vid]]);
    });

    delete[] activeVoxelsContainer;
    PRINT(this->actVoxels.size());

    // for(int i = 0; i < 1; i++){
    //   printf("activeVoxel:[%f,%f,%f,%f,%f,%f,%f,%f]\n",
    //    this->actVoxels[i].vtx[0][0][0], this->actVoxels[i].vtx[0][0][1],
    //    this->actVoxels[i].vtx[0][1][0], this->actVoxels[i].vtx[0][1][1],
    //    this->actVoxels[i].vtx[1][0][0], this->actVoxels[i].vtx[1][0][1],
    //    this->actVoxels[i].vtx[1][1][0], this->actVoxels[i].vtx[1][1][1]);
    //   PRINT(this->actVoxels[i].bounds);
    // }

#endif

#endif
  }

  TestTAMR::~TestTAMR() {}

  /*! create lits of *all* voxel (refs) we want to be considered for
   * interesction */
  void TestTAMR::getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                 float isoValue) const
  {
    // the whole thing has 7 coarse cells and 8 fine cells ... 15 total
    activeVoxels.clear();

#if BUILD_ALL
    size_t sIdx = 0;
    size_t eIdx = this->voxels.size();  // 32;
    for (size_t i = sIdx; i < eIdx; i++) {
      if (this->voxelVRange[i].x <= isoValue &&
          this->voxelVRange[i].y >= isoValue)
        activeVoxels.push_back(i);
    }
#else
    for (int i = 0; i < this->actVoxels.size(); ++i) {
      activeVoxels.push_back(i);
    }
#endif
  }

  /*! compute world-space bounds for given voxel */
  box3fa TestTAMR::getVoxelBounds(const VoxelRef voxelRef) const
  {
#if BUILD_ALL
    return this->voxels[voxelRef].bounds;
#else
    return this->actVoxels[voxelRef].bounds;
#endif
  }

  /*! get full voxel - bounds and vertex values - for given voxel */
  Impi::Voxel TestTAMR::getVoxel(const VoxelRef voxelRef) const
  {
#if BUILD_ALL
    return this->voxels[voxelRef];
#else
    return this->actVoxels[voxelRef];
#endif
  }
}  // namespace testCase
