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

#include "TestOctant.h"
#include "common/Data.h"
#include "compute_voxels_ispc.h"
#include "ospcommon/tasking/parallel_for.h"
#include "ospcommon/utility/getEnvVar.h"

#include <time.h>
#include <numeric>


#ifndef speedtest__
#define speedtest__(data)                                           \
  for (long blockTime = NULL;                                       \
       (blockTime == NULL ? (blockTime = clock()) != NULL : false); \
       printf(data " Took: %.9fs \n",                               \
              (double)(clock() - blockTime) / CLOCKS_PER_SEC))
#endif

typedef ospray::amr::AMRAccel::Leaf AMRLeaf;

namespace ospray {
  namespace impi {
    namespace testCase {

      // ================================================================== //
      // Main Functions
      // ================================================================== //
      /*! constructors and distroctors */
      TestOctant::TestOctant(AMRVolume *amr, float isoValue)
          : reconMethod(
                ospcommon::utility::getEnvVar<std::string>("IMPI_AMR_METHOD")
                    .value_or("octant")),
            storeMethod(
                ospcommon::utility::getEnvVar<std::string>("IMPI_AMR_STORAGE")
                    .value_or("active")),
            amrVolumePtr(amr)
      {
        /* debug */
        printf("#osp:impi: recomstruction method %s\n", reconMethod.c_str());
        printf("#osp:impi: storage strategy %s\n", storeMethod.c_str());

        /* get AMR volume pointer */
        if (!amr)
          throw std::runtime_error("Empty amr volume");
        if (amr->accel->leaf.size() <= 0)
          throw std::runtime_error("AMR Volume has no leaf");
        std::cout << "#osp:impi: Number of AMR Leaves "
                  << amr->accel->leaf.size() << std::endl;

        /* compute default bbox */
        // TODO: we should use getParamData here to set bounding boxes
        clipBoxes.push_back(box3fa(amr->accel->worldBounds.lower,
                                   amr->accel->worldBounds.upper));
      }
      TestOctant::~TestOctant() {}

      /*! compute active voxels (called in Impi.cpp file) */
      void TestOctant::getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                       float isoValue) const
      {
        if (storeMethod == "all") {
          throw std::runtime_error(storeMethod + " is not implemented");
        } else if (storeMethod == "active") {
          getActiveVoxels_active(activeVoxels, isoValue);
          ;
        } else if (storeMethod == "none") {
          getActiveVoxels_none(activeVoxels, isoValue);
          ;
        } else {
          throw std::runtime_error(storeMethod +
                                   " is not a valid storage strategy");
        }
      }

      /*! compute world-space bounds for given voxel */
      box3fa TestOctant::getVoxelBounds(const VoxelRef voxelRef) const
      {
        if (storeMethod == "all") {
          throw std::runtime_error(storeMethod + " is not implemented");
        } else if (storeMethod == "active") {
          return getVoxelBounds_active(voxelRef);
        } else if (storeMethod == "none") {
          return getVoxelBounds_none(voxelRef);
        } else {
          throw std::runtime_error(storeMethod +
                                   " is not a valid storage strategy");
        }
      }

      /*! get full voxel - bounds and vertex values - for given voxel */
      Voxel TestOctant::getVoxel(const VoxelRef voxelRef) const
      {
        if (storeMethod == "all") {
          throw std::runtime_error(storeMethod + " is not implemented");
        } else if (storeMethod == "active") {
          return getVoxel_active(voxelRef);
        } else if (storeMethod == "none") {
          return getVoxel_none(voxelRef);
        } else {
          throw std::runtime_error(storeMethod +
                                   " is not a valid storage strategy");
        }
      }

      /*! preprocess voxel list base on method */
      void TestOctant::build(float isoValue)
      {
        if (storeMethod == "all") {
          throw std::runtime_error(storeMethod + " is not implemented");
        } else if (storeMethod == "active") {
          return build_active(isoValue);
          ;
        } else if (storeMethod == "none") {
          return build_none(isoValue);
          ;
        } else {
          throw std::runtime_error(storeMethod +
                                   " is not a valid storage strategy");
        }
      }
    }  // namespace testCase
  }    // namespace impi
}  // namespace ospray

namespace ospray {
  namespace impi {
    namespace testCase {

      // ================================================================== //
      // Store Strategy: active
      // ================================================================== //
      extern "C" void externC_push_back_active(void *_c_vector,
                                               void *_c_ptr,
                                               const float v0,
                                               const float v1,
                                               const float v2,
                                               const float v3,
                                               const float v4,
                                               const float v5,
                                               const float v6,
                                               const float v7,
                                               const float c0,
                                               const float c1,
                                               const float c2,
                                               const float cellwidth)
      {
        auto c_ptr = (TestOctant *)_c_ptr;
        const vec3f coordinate(c0, c1, c2);
        const box3fa box(coordinate, coordinate + cellwidth);
        if (c_ptr->inClipBox(box)) {
          auto c_vector = (std::vector<Voxel> *)_c_vector;
          c_vector->emplace_back();
          c_vector->back().vtx[0][0][0] = v0;
          c_vector->back().vtx[0][0][1] = v1;
          c_vector->back().vtx[0][1][0] = v2;
          c_vector->back().vtx[0][1][1] = v3;
          c_vector->back().vtx[1][0][0] = v4;
          c_vector->back().vtx[1][0][1] = v5;
          c_vector->back().vtx[1][1][0] = v6;
          c_vector->back().vtx[1][1][1] = v7;
          c_vector->back().bounds       = box;
        }
      }

      /*! compute world-space bounds for given voxel */
      box3fa TestOctant::getVoxelBounds_active(const VoxelRef voxelRef) const
      {
        return voxels[voxelRef].bounds;
      }
      /*! get full voxel - bounds and vertex values - for given voxel */
      Voxel TestOctant::getVoxel_active(const VoxelRef voxelRef) const
      {
        return voxels[voxelRef];
      }

      /*! preprocess voxel list base on method */
      void TestOctant::build_active(float isoValue)
      {
        voxels.clear();
        //
        // initialization
        //
        const auto &accel = amrVolumePtr->accel;
        const auto nLeaf  = accel->leaf.size();
        //
        // Testing my implementation
        // TODO: check this function in latest ospray version
        //
        auto leafActiveOctants = new std::vector<Voxel>[nLeaf];
        speedtest__("#osp:impi: Preprocessing Voxel Values")
        {
          tasking::parallel_for(nLeaf, [&](size_t lid) {
            //
            // meta data
            //
            const ospray::amr::AMRAccel::Leaf &lf = accel->leaf[lid];
            const float w      = lf.brickList[0]->cellWidth;  // cell width
            const float s      = lf.brickList[0]->gridToWorldScale;
            const vec3f &lower = lf.bounds.lower;
            const vec3f &upper = lf.bounds.upper;
            // TODO: this is wrong, why ??
            const size_t nx = std::round((upper.x - lower.x) * s);
            const size_t ny = std::round((upper.y - lower.y) * s);
            const size_t nz = std::round((upper.z - lower.z) * s);
            const auto &rg  = lf.valueRange;
            //
            // number of octants
            //
            // add inner cells
            const auto n1 =
                (nx - size_t(1)) * (ny - size_t(1)) * (nz - size_t(1));
            // bottom top boundray cells
            const auto n2 = size_t(8) * ny * nx;
            // left right boundray cells
            const auto n3 = size_t(8) * nz * ny;
            // front back boundary cells
            const auto n4 = size_t(8) * nz * nx;
            // total number
            const auto N = (n1 + n2 + n3 + n4);
            //
            // check leaf range
            //
            const size_t b = 0;
            const size_t e = N;
            ispc::getAllVoxels_active(amrVolumePtr->getIE(),
                                      this,
                                      &leafActiveOctants[lid],
                                      isoValue,
                                      w,
                                      (ispc::vec3f &)lower,
                                      (ispc::vec3f &)upper,
                                      (uint32_t)b,
                                      (uint32_t)e,
                                      (uint32_t)nx,
                                      (uint32_t)ny,
                                      (uint32_t)nz,
                                      (uint32_t)n1,
                                      (uint32_t)(n2 + n1),
                                      (uint32_t)(n3 + n2 + n1));
          });
        }
        std::cout << "#osp:impi: Done Computing Values Values" << std::endl;

        std::vector<size_t> begin(nLeaf, size_t(0));
        size_t n(0);
        for (int lid = 0; lid < nLeaf; ++lid) {
          begin[lid] = n;
          n += leafActiveOctants[lid].size();
        }
        voxels.resize(n);
        tasking::parallel_for(nLeaf, [&](const size_t lid) {
          std::copy(leafActiveOctants[lid].begin(),
                    leafActiveOctants[lid].end(),
                    &voxels[begin[lid]]);
        });

        delete[] leafActiveOctants;

        std::cout << "Done Init Octant Value! " << voxels.size() << std::endl;
      }

      /*! compute active voxels (called in Impi.cpp file) */
      void TestOctant::getActiveVoxels_active(
          std::vector<VoxelRef> &activeVoxels, float isoValue) const
      {
        activeVoxels.clear();  // the output
        for (int i = 0; i < voxels.size(); ++i) {
          activeVoxels.push_back(i);
        }
      }

      // ================================================================== //
      // Store Strategy: none
      // ================================================================== //
      extern "C" void externC_push_back_none(void *_c_vector,
                                             const void *_c_ptr,
                                             const uint32_t lid,
                                             const uint32_t oid,
                                             const float c0,
                                             const float c1,
                                             const float c2,
                                             const float cellwidth)
      {
        const auto c_ptr = (const TestOctant *)_c_ptr;
        const auto box =
            box3fa(vec3f(c0, c1, c2), vec3f(c0, c1, c2) + cellwidth);
        if (c_ptr->inClipBox(box)) {
          auto c_vector = (std::vector<uint64_t> *)_c_vector;
          // pack indices
          // TODO: we might want to do safety check here
          uint64_t idx = (uint64_t(lid) << 32) | uint64_t(oid);
          c_vector->push_back(idx);
        }
      }

      /*! compute world-space bounds for given voxel */
      box3fa TestOctant::getVoxelBounds_none(const VoxelRef voxelRef) const
      {
        // return getVoxel_none(voxelRef).bounds;
        // unpack VoxelRef (uint64_t) into two uint32_t indices
        union
        {
          uint32_t out[2];
          uint64_t in;
        } unpack;
        unpack.in           = voxelRef;
        const uint32_t &oid = unpack.out[0];
        const uint32_t &lid = unpack.out[1];

        const AMRLeaf &lf = amrVolumePtr->accel->leaf[lid];

        const float w      = lf.brickList[0]->cellWidth;  // cell width
        const float s      = lf.brickList[0]->gridToWorldScale;
        const vec3f &lower = lf.bounds.lower;
        const vec3f &upper = lf.bounds.upper;
        const size_t nx    = std::round((upper.x - lower.x) * s);
        const size_t ny    = std::round((upper.y - lower.y) * s);
        const size_t nz    = std::round((upper.z - lower.z) * s);
        // add inner cells
        const auto n1 = (nx - size_t(1)) * (ny - size_t(1)) * (nz - size_t(1));
        // bottom top boundray cells
        const auto n2 = size_t(8) * ny * nx;
        // left right boundray cells
        const auto n3 = size_t(8) * nz * ny;
        // front back boundary cells
        // const auto n4 = size_t(8) * nz * nx;
        //
        float cellwidth;
        box3fa bounds;
        ispc::getOneVoxelBounds_octant(amrVolumePtr->getIE(),
                                       cellwidth,
                                       (ispc::vec3f &)bounds.lower,
                                       w,
                                       (ispc::vec3f &)lower,
                                       (ispc::vec3f &)upper,
                                       oid,
                                       (uint32_t)nx,
                                       (uint32_t)ny,
                                       (uint32_t)nz,
                                       (uint32_t)n1,
                                       (uint32_t)(n2 + n1),
                                       (uint32_t)(n3 + n2 + n1));
        bounds.upper = bounds.lower + cellwidth;
        return bounds;
      }

      /*! get full voxel - bounds and vertex values - for given voxel */
      Voxel TestOctant::getVoxel_none(const VoxelRef voxelRef) const
      {
        Voxel voxel;

        // unpack VoxelRef (uint64_t) into two uint32_t indices
        union
        {
          uint32_t out[2];
          uint64_t in;
        } unpack;
        unpack.in           = voxelRef;
        const uint32_t &oid = unpack.out[0];
        const uint32_t &lid = unpack.out[1];

        const AMRLeaf &lf = amrVolumePtr->accel->leaf[lid];

        const float w      = lf.brickList[0]->cellWidth;  // cell width
        const float s      = lf.brickList[0]->gridToWorldScale;
        const vec3f &lower = lf.bounds.lower;
        const vec3f &upper = lf.bounds.upper;
        const size_t nx    = std::round((upper.x - lower.x) * s);
        const size_t ny    = std::round((upper.y - lower.y) * s);
        const size_t nz    = std::round((upper.z - lower.z) * s);
        // add inner cells
        const auto n1 = (nx - size_t(1)) * (ny - size_t(1)) * (nz - size_t(1));
        // bottom top boundray cells
        const auto n2 = size_t(8) * ny * nx;
        // left right boundray cells
        const auto n3 = size_t(8) * nz * ny;
        // front back boundary cells
        // const auto n4 = size_t(8) * nz * nx;
        //
        float cellwidth;
        ispc::getOneVoxel_octant(amrVolumePtr->getIE(),
                                 cellwidth,
                                 (ispc::vec3f &)voxel.bounds.lower,
                                 &(voxel.vtx[0][0][0]),
                                 w,
                                 (ispc::vec3f &)lower,
                                 (ispc::vec3f &)upper,
                                 oid,
                                 (uint32_t)nx,
                                 (uint32_t)ny,
                                 (uint32_t)nz,
                                 (uint32_t)n1,
                                 (uint32_t)(n2 + n1),
                                 (uint32_t)(n3 + n2 + n1));
        voxel.bounds.upper = voxel.bounds.lower + cellwidth;
        return voxel;
      }

      /*! preprocess voxel list base on method */
      void TestOctant::build_none(float isoValue) {}

      /*! compute active voxels (called in Impi.cpp file) */
      void TestOctant::getActiveVoxels_none(std::vector<VoxelRef> &activeVoxels,
                                            float isoValue) const
      {
        //
        // Testing my implementation
        //
        const auto &accel      = amrVolumePtr->accel;
        const auto nLeaf       = accel->leaf.size();
        auto leafActiveOctants = new std::vector<uint64_t>[nLeaf];
        speedtest__("#osp:impi: Preprocess Voxel Values")
        {
          tasking::parallel_for(nLeaf, [&](size_t lid) {
            //
            // meta data
            //
            const ospray::amr::AMRAccel::Leaf &lf = accel->leaf[lid];
            const float w      = lf.brickList[0]->cellWidth;  // cell width
            const float s      = lf.brickList[0]->gridToWorldScale;
            const vec3f &lower = lf.bounds.lower;
            const vec3f &upper = lf.bounds.upper;
            // TODO: this is wrong, why ??
            const size_t nx = std::round((upper.x - lower.x) * s);
            const size_t ny = std::round((upper.y - lower.y) * s);
            const size_t nz = std::round((upper.z - lower.z) * s);
            const auto &rg  = lf.valueRange;
            //
            // number of octants
            //
            // add inner cells
            const auto n1 =
                (nx - size_t(1)) * (ny - size_t(1)) * (nz - size_t(1));
            // bottom top boundray cells
            const auto n2 = size_t(8) * ny * nx;
            // left right boundray cells
            const auto n3 = size_t(8) * nz * ny;
            // front back boundary cells
            const auto n4 = size_t(8) * nz * nx;
            // total number
            const auto N = (n1 + n2 + n3 + n4);
            //
            // check leaf range
            //
            // lock.lock();
            // std::cout << lid << " "
            // 		<< &leafActiveOctants[lid] << " "
            // 		<< leafActiveOctants[lid].size() << std::endl;
            // lock.unlock();
            // const size_t blockSize(32 * 16);
            // const size_t blockNum = (N + blockSize - 1) / blockSize;
            // const size_t off = numOfLeafOctants[lid];
            // tasking::parallel_for(blockNum, [&](size_t blockID) {
            // const size_t b = blockID * blockSize;
            // const size_t e = std::min(b + blockSize, N);
            const size_t b = 0;
            const size_t e = N;
            ispc::getAllVoxels_none(amrVolumePtr->getIE(),
                                    this,
                                    &leafActiveOctants[lid],
                                    isoValue,
                                    w,
                                    lid,
                                    (ispc::vec3f &)lower,
                                    (ispc::vec3f &)upper,
                                    (uint32_t)b,
                                    (uint32_t)e,
                                    (uint32_t)nx,
                                    (uint32_t)ny,
                                    (uint32_t)nz,
                                    (uint32_t)n1,
                                    (uint32_t)(n2 + n1),
                                    (uint32_t)(n3 + n2 + n1));
            //});
          });
        }
        //
        //
        //
        std::cout << "#osp:impi: Done Computing Values Values" << std::endl;
        std::vector<size_t> begin(nLeaf, size_t(0));
        size_t n(0);
        for (int lid = 0; lid < nLeaf; ++lid) {
          begin[lid] = n;
          n += leafActiveOctants[lid].size();
        }
        activeVoxels.resize(n);
        tasking::parallel_for(nLeaf, [&](const size_t lid) {
          std::copy(leafActiveOctants[lid].begin(),
                    leafActiveOctants[lid].end(),
                    &activeVoxels[begin[lid]]);
        });

        delete[] leafActiveOctants;
      }
    }  // namespace testCase
  }    // namespace impi
}  // namespace ospray


