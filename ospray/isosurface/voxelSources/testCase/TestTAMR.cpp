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
#include "tbb/tbb.h"
#include "../../../Utils/parallel_scan.h"

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

  extern "C" void externC_getPotentialOverlapVoxels(
      void *C_potentialOverlapVoxelsIdx, const size_t idx)
  {
    auto potentialOverlapVoxelsIdx = (std::vector<size_t> *)C_potentialOverlapVoxelsIdx;

    potentialOverlapVoxelsIdx->emplace_back();
    potentialOverlapVoxelsIdx->back() = idx;
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

    ispc::build_activeVoxels(tamrVolume->getIE(),
                             (void *)this->voxels.data(),
                             (void *)&this->actVoxels,
                             this->voxels.size(),
                             isoValue);
    double calVoxelValueTime = Time(t1);
    std::cout << green << "SIMD Only! Build active voxel value takes:" << calVoxelValueTime
              << reset << std::endl;

#else
  // NOTICE: for finest method, each voxel should be divided into voxel that has
  // the same width with the octant in the finest voxels.

    std::cout << "Start to calculate the voxel value! VoxelNUM:" << numVoxels << std::endl;
    time_point t1 = Time();

    auto filterMethodEnv =
        utility::getEnvVar<std::string>("OSPRAY_TAMR_METHOD");
    std::string filterMethod = filterMethodEnv.value_or("nearest");

    int fMethod = TRILINEAR;

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
    
    time_point t2 = Time();
    // auto activeVoxelsContainer = new std::vector<Voxel>[numVoxels];
    std::vector<std::vector<Voxel>> activeVoxelsContainer(numVoxels);
    bool* isOverlapVoxelMask = new bool[numVoxels];
    tasking::parallel_for(numVoxels, [&](size_t vid) {

      float localCellWidth = inputVoxels[vid].width / tamrVolume->gridWorldSpace.x;
      // float localCellWidth = inputVoxels[vid].width;

      
      // This determines how many decents are divided from a parent cell when build the active voxels
      uint32 refineFactor = 2; 
      if(fMethod == FINEST)
        refineFactor = 2 * localCellWidth;

      ispc::build_activeVoxel(tamrVolume->getIE(),
                              (ispc::vec3f &)inputVoxels[vid].lower,
                              inputVoxels[vid].value,
                              localCellWidth,
                              refineFactor,
                              (void *)(activeVoxelsContainer.data() + vid),
                              // (void *)&activeVoxelsContainer[vid],
                              isoValue,
                              fMethod,
                              isOverlapVoxelMask + vid);
    });
    double time2 = Time(t2);
    std::cout << green << "Build active voxel takes:" << time2 << reset << std::endl;

    size_t overlapVoxelNum(0);
    overlapVoxelNum = parallel_count(isOverlapVoxelMask, numVoxels);
    PRINT(overlapVoxelNum);

    time_point t6 = Time();
    std::vector<size_t> offset;
    offset.resize(activeVoxelsContainer.size() + 1, 0);
    using range_type      = tbb::blocked_range<size_t>;
    size_t activeVoxelNum = tbb::parallel_scan(
        range_type(0, activeVoxelsContainer.size()),
        0,
        [&](const range_type &r, size_t sum, bool is_final_scan) {
          size_t tmp = sum;
          for (size_t i = r.begin(); i < r.end(); ++i) {
            tmp = tmp + activeVoxelsContainer[i].size();
            if (is_final_scan) {
              offset[i + 1] = tmp;
            }
          }
          return tmp;
        },
        [&](const size_t &a, const size_t &b) { return a + b; });
    offset.pop_back();
    double time6 = Time(t6);
    std::cout << green << "Parallel Prefix sum takes:" << time6 << reset
              << std::endl;

    this->actVoxels.resize(activeVoxelNum);

    time_point t4 = Time();

    tasking::parallel_for(numVoxels, [&](const size_t vid) {
      std::copy(activeVoxelsContainer[vid].begin(),
                activeVoxelsContainer[vid].end(),
                &this->actVoxels[offset[vid]]);
    });

    double time4 = Time(t4);
    std::cout << green << "Copy active voxel takes:" << time4 << reset << std::endl;

    time_point t5 = Time();
    // delete[] activeVoxelsContainer;
    delete[] isOverlapVoxelMask;
    double time5 = Time(t5);
    std::cout << green << "Delete buffer takes:" << time5 << reset << std::endl;

    PRINT(this->actVoxels.size());

    double calVoxelValueTime = Time(t1);
    std::cout << green
              << "SIMD and TBB! Build active voxel value takes:" << calVoxelValueTime
              << reset << std::endl;

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
    for (int i = 0; i < this->actVoxels.size(); ++i) {
      activeVoxels.push_back(i);
    }
  }

  /*! compute world-space bounds for given voxel */
  box3fa TestTAMR::getVoxelBounds(const VoxelRef voxelRef) const
  {
    return this->actVoxels[voxelRef].bounds;
  }

  /*! get full voxel - bounds and vertex values - for given voxel */
  Impi::Voxel TestTAMR::getVoxel(const VoxelRef voxelRef) const
  {
    return this->actVoxels[voxelRef];
  }
}  // namespace testCase
