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

//#include "ospcommon/range.h"
//#include "ospcommon/array3D/for_each.h"

#include "../../geometry/Impi.h"
#include "volume/amr/AMRAccel.h"
#include "volume/amr/AMRVolume.h"

#include <functional>
#include <limits>

namespace ospray {
  namespace impi {
    namespace testCase {

      typedef Impi::Voxel Voxel;

      /*! implements a simple (vertex-cenetred) AMR test case
          consisting of a 2x2x2-cell base level in which one of the
          cells is refined infilterActiveVoxelsto another 2x2x2-cell
    finer level */
      struct TestOctant : public Impi::VoxelSource
      {
       public:
        /*! constructors and distroctors */
        TestOctant(ospray::AMRVolume *, float);
        virtual ~TestOctant();

        /*! get full voxel - bounds and vertex values - for given voxel */
        virtual Voxel getVoxel(const VoxelRef voxelRef) const override;

        /*! compute world-space bounds for given voxel */
        virtual box3fa getVoxelBounds(const VoxelRef voxelRef) const override;

        /*! compute active voxels (called in Impi.cpp file) */
        virtual void getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                     float isoValue) const override;

        /*! preprocess voxel list base on method */
        void build(float isoValue);

       private:
        /*! =============================================================== */
        /* void (*build_fcn)(float); */
        /* typedef void (TestOctant::*Fcn_getActiveVoxels) */
        /*   (std::vector<VoxelRef> &, float) const; */
        /* Fcn_getActiveVoxels getActiveVoxels_fcn; */
        /* typedef Voxel (TestOctant::*Fcn_getVoxel) */
        /*   (const VoxelRef) const; */
        /* Fcn_getVoxel getVoxel_fcn; */
        /* typedef box3fa (TestOctant::*Fcn_getVoxelBounds) */
        /*   (const VoxelRef) const; */
        /* Fcn_getVoxelBounds getVoxelBounds_fcn; */
        /*! =============================================================== */

        /*! get full voxel - bounds and vertex values - for given voxel */
        // Voxel getVoxel_all   (const VoxelRef voxelRef) const;
        Voxel getVoxel_active(const VoxelRef voxelRef) const;
        Voxel getVoxel_none(const VoxelRef voxelRef) const;

        /*! compute world-space bounds for given voxel */
        // box3fa getVoxelBounds_all   (const VoxelRef voxelRef) const;
        box3fa getVoxelBounds_active(const VoxelRef voxelRef) const;
        box3fa getVoxelBounds_none(const VoxelRef voxelRef) const;

        /*! compute active voxels (called in Impi.cpp file) */
        // void getActiveVoxels_all   (std::vector<VoxelRef> &activeVoxels,
        //                            float isoValue) const;
        void getActiveVoxels_active(std::vector<VoxelRef> &activeVoxels,
                                    float isoValue) const;
        void getActiveVoxels_none(std::vector<VoxelRef> &activeVoxels,
                                  float isoValue) const;

        /*! preprocess voxel list base on method */
        // void build_all   (float isoValue);
        void build_active(float isoValue);
        void build_none(float isoValue);

       public:
        /*! check if the voxel is inside the clip box */
        bool inClipBox(const box3f &box) const
        {
          return inClipBox(box3fa(box.lower, box.upper));
        }
        bool inClipBox(const box3fa &box) const
        {
          for (auto clipbox : clipBoxes) {
            if (touchingOrOverlapping(clipbox, box)) {
              return true;
            }
          }
          return false;
        }

       private:
        /*! a list of voxels, it is a buffer that can have different usages
          for different implementations */
        std::vector<Voxel> voxels;

        std::vector<box3fa> clipBoxes;
        const ospray::AMRVolume *amrVolumePtr;
        const std::string reconMethod; /* octant, current, nearest */
        const std::string storeMethod; /* all, active, none */

       public:
        /*! initialization */
        // void initOctant(ospray::AMRVolume *amr);

       private:
        // TODO things below should be cleaned

        // void parseOctant(std::string fileName);
        /*! create lits of *all* voxel (refs) we want to be considered for
    interesction */
        // size_t octNum = 0;

        /* void initOctantValue(ospray::AMRVolume *amr); */

        /* void buildOctant(ospray::AMRVolume *amr); */
        /* void buildOctantByLeaf(std::unique_ptr<ospray::amr::AMRAccel> &accel,
         */
        /*                        ospray::amr::AMRAccel::Leaf &lf, */
        /*                        std::vector<vec3f> *outOctLowV, */
        /*                        std::vector<float> *outOctW); */

        /* std::vector<vec3f> octVertices; */
        /* std::vector<float> octWidth; */
        /* float *octValueBuffer; */

        /* std::vector<Range> octRange; */

        /* void getOctrange(size_t octID,Range* range) */
        /* { */
        /*   for (size_t i = 0; i < 8; i++) { */
        /*     size_t idx = octID * 8 + i; */
        /*     range->extend(this->octValueBuffer[idx]); */
        /*   } */
        /* } */
      };

    }  // namespace testCase
  }    // namespace impi
}  // namespace ospray
