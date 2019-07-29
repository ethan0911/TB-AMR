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

#include "Impi.h"
#include "Impi_ispc.h"
// ospray core:
#include <ospray/common/Data.h>

#include "../voxelSources/testCase/TestAMR.h"
#include "../voxelSources/testCase/TestTAMR.h"
#include "../voxelSources/testCase/TestVoxel.h"

#include "../voxelSources/structured/SegmentedVolumeSource.h"
#include "../voxelSources/structured/StructuredVolumeSource.h"
#include "../voxelSources/testCase/TestOctant.h"

#include "filter_nearest_ispc.h"

#include <limits>

#include <chrono>
#include <ctime>
#include "time.h"
using namespace std::chrono;

/*! constructor - will create the 'ispc equivalent' */
Impi::Impi()
{
  /*! create the 'ispc equivalent': ie, the ispc-side class that
    implements all the ispc-side code for intersection,
    postintersect, etc. See Impi.ispc */
  this->ispcEquivalent = ispc::Impi_create(this);
  // note we do _not_ yet do anything else here - the actual input
  // data isn't available to use until 'commit()' gets called
  isoValue     = std::numeric_limits<float>::infinity();
  lastIsoValue = std::numeric_limits<float>::infinity();
}

/*! destructor - supposed to clean up all alloced memory */
Impi::~Impi()
{
  ispc::Impi_destroy(ispcEquivalent);
}

/*! commit - this is the function that parses all the parameters
  that the app has proivded for this geometry. In this simple
  example we're looking for a single parameter named 'patches',
  which is supposed to contain a data array of all the patches'
  control points */
void Impi::commit()
{
  isoValue = getParam1f("isoValue", 20.f);
  isoColor = getParam4f("isoColor", vec4f(1.0f));

  const voxel* inputVoxels = (voxel*)getParamVoidPtr("inputVoxels",nullptr);
  size_t numVoxels = *(size_t *)getParamVoidPtr("numInputVoxels",nullptr);
  TAMRVolume* tamrVolume = (TAMRVolume*)getParamVoidPtr("TAMRVolume", nullptr);

  // PRINT(numVoxels);

  if (!voxelSource) {
#if 0
  isoValue = 20.f;
  voxelSource = std::make_shared<testCase::TestVoxel>();
#elif 1
    isoValue    = 6.5f;
    voxelSource = std::make_shared<testCase::TestTAMR>(
        tamrVolume, inputVoxels, numVoxels, isoValue);
#elif 0
    auto amr = (ospray::AMRVolume *)getParamObject("amrDataPtr", nullptr);
    PRINT(amr->voxelRange);
    isoValue    = getParam1f("isoValue", 0.7f);
    isoColor    = getParam4f("isoColor", vec4f(1.0f));
    voxelSource = std::make_shared<testCase::TestOctant>(amr, isoValue);
#elif 0
  /*! create a simple, amr-like data structure - just to test different-sized
   * voxels right next to each other */
  isoValue    = 3.2f;
  voxelSource = std::make_shared<testCase::TestAMR>();
#elif 0
  isoValue = 0.5f;
  std::shared_ptr<structured::LogicalVolume> volume =
      structured::createTestVolume(vec3i(64));
  voxelSource = std::make_shared<structured::StructuredVolumeSource>(volume);
#endif

    // initVoxelSourceAndIsoValue();
  }
}

/*! ispc can't directly call virtual functions on the c++ side, so we use this
 * callback instead */
extern "C" void externC_getVoxelBounds(box3fa &bounds,
                                       const Impi *self,
                                       const uint64_t voxelRef)
{
  bounds = self->voxelSource->getVoxelBounds(voxelRef);
}

extern "C" void externC_getVoxel(Impi::Voxel &voxel,
                                 const Impi *self,
                                 const uint64_t voxelRef)
{
  voxel = self->voxelSource->getVoxel(voxelRef);
}

/*! 'finalize' is what ospray calls when everything is set and
  done, and a actual user geometry has to be built */
// Why this will work ???
void Impi::finalize(World *model)
{
  Geometry::finalize(model);

  // generate list of active voxels
  // if (this->lastIsoValue != isoValue) {
  //   std::shared_ptr<testCase::TestOctant> testOct =
  //       std::dynamic_pointer_cast<testCase::TestOctant>(voxelSource);

  //   high_resolution_clock::time_point t1 = high_resolution_clock::now();

  //   testOct->build(isoValue);
  //   voxelSource->getActiveVoxels(activeVoxelRefs, isoValue);

  //   high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //   duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
  //   printf("Build Active Octants Time: %.9fs \n", time_span.count());

  //   this->lastIsoValue = isoValue;
  // }

  voxelSource->getActiveVoxels(activeVoxelRefs, isoValue);

  // and ask ispc side to build the voxels
  ispc::Impi_finalize(getIE(),
                      model->getIE(),
                      (uint64_t *)&activeVoxelRefs[0],
                      activeVoxelRefs.size(),
                      (void *)this,
                      isoValue,
                      (ispc::vec4f *)&isoColor);
}

/*! create voxel source from whatever parameters we have been passed (now
 * hardcoded) */
void Impi::initVoxelSourceAndIsoValue()
{
#if 0
  isoValue = 20.f;
  voxelSource = std::make_shared<testCase::TestVoxel>();
#elif 1
  isoValue    = 20.f;
  voxelSource = std::make_shared<testCase::TestTAMR>();
#elif 0
  auto amr = (ospray::AMRVolume *)getParamObject("amrDataPtr", nullptr);
  PRINT(amr->voxelRange);
  isoValue    = getParam1f("isoValue", 0.7f);
  isoColor    = getParam4f("isoColor", vec4f(1.0f));
  voxelSource = std::make_shared<testCase::TestOctant>(amr, isoValue);
#elif 0
  /*! create a simple, amr-like data structure - just to test different-sized
   * voxels right next to each other */
  isoValue    = 3.2f;
  voxelSource = std::make_shared<testCase::TestAMR>();
#elif 0
  isoValue = 0.5f;
  std::shared_ptr<structured::LogicalVolume> volume =
      structured::createTestVolume(vec3i(64));
  voxelSource = std::make_shared<structured::StructuredVolumeSource>(volume);
#elif 0
  std::shared_ptr<structured::LogicalVolume> volume;
  const std::string fileName = "magnetic.raw";
  try {
    isoValue = 1.f;
    std::cout << "loading test data set '" << fileName << "'" << std::endl;
    volume = structured::VolumeT<float>::loadRAW(fileName, vec3i(512));
  } catch (std::runtime_error e) {
    std::cout << "could not load '" << fileName
              << "' test file (reason: " << e.what()
              << "), using blob-testcase instead" << std::endl;
    isoValue = 0.5f;
    volume   = structured::createTestVolume(vec3i(64));
  }
  voxelSource = std::make_shared<structured::StructuredVolumeSource>(volume);
#else
  isoValue = 20;
  std::shared_ptr<structured::LogicalVolume> volume =
      structured::VolumeT<float>::loadRAW("density_064_064_2.0.raw", vec3i(64));
  std::shared_ptr<structured::LogicalVolume> segvol =
      structured::VolumeT<float>::loadRAW("density_064_064_2.0_seg.raw",
                                          vec3i(64));
  voxelSource =
      std::make_shared<structured::SegmentedVolumeSource>(volume, segvol, 128);
#endif
}

/*! maybe one of the most important parts of this example: this
  macro 'registers' the Impi class under the ospray
  geometry type name of 'bilinear_patches'.

  It is _this_ name that one can now (assuming the module has
  been loaded with ospLoadModule(), of course) create geometries
  with; i.e.,
*/
OSP_REGISTER_GEOMETRY(Impi, impi);

extern "C" void ospray_init_module_impi()
{
  std::cout
      << "#osp: initializing the 'Implicit Iso-Surfaces Geometry (IMPI)' module"
      << std::endl;
}
