#include <iostream>
#include <stdexcept>
#include "ospcommon/vec.h"
#include "ospcommon/box.h"

// Our exported ISPC functions are in this _ispc.h
#include "TAMRVolume_ispc.h"
#include "TAMRVolume.h"
#include "filter_nearest_ispc.h"
#include "filter_current_ispc.h"
#include "filter_finest_ispc.h"
#include "filter_octant_ispc.h"
#include "filter_trilinear_ispc.h"

using namespace ospcommon;

// This function is just to show an example of how we can call back
// from ISPC into C++
extern "C" void ispc_called_cpp_function_example(int val) {
  std::cout << "Called from ISPC, val: " << val << "\n";
}

/*! callback function called by ispc sampling code to compute a
  gradient at given sample pos in this (c++-only) module */
extern "C" float P4est_scalar_sample(ScalarVolumeSampler *cppSampler, const vec3f *samplePos)
{
  return cppSampler->sample(*samplePos);
}

extern "C" void P4est_scalar_batch_sample(ScalarVolumeSampler *cppSampler,
                                          const vec3f *posBuffer,
                                          const int numActivePos,
                                          const int* activeIDs,
                                          float* values)
{
  cppSampler->batch_sample(posBuffer,numActivePos,activeIDs,values);
}

/*! callback function called by ispc sampling code to compute a
  sample in this (c++-only) module */
extern "C" vec3f P4est_scalar_computeGradient(ScalarVolumeSampler *cppSampler, const vec3f *samplePos)
{
  return cppSampler->computeGradient(*samplePos);
}

TAMRVolume::TAMRVolume() {
  p4est = nullptr; //init to null for safety

  // Create our ISPC-side version of the struct
  ispcEquivalent = ispc::TAMRVolume_createISPCEquivalent(this);
}

TAMRVolume::~TAMRVolume() {
  ispc::TAMRVolume_freeVolume(ispcEquivalent);
  delete sampler;

  if(_voxelAccel)
    delete _voxelAccel;
}

std::string TAMRVolume::toString() const {
  return "TAMRVolume";
}


ScalarVolumeSampler* TAMRVolume::createSampler(){
  return new TAMRVolumeSampler(this);
}

void TAMRVolume::commit() {
  Volume::commit();
  updateEditableParameters();

  this->sampler = createSampler();

  vec3f worldOrigin = getParam3f("worldOrigin", vec3f(0.f)); 
  // Set the grid origin, default to (0,0,0).
  this->gridOrigin = getParam3f("gridOrigin", vec3f(0.f));
  // Set the grid spacing, default to (1,1,1).
  this->gridWorldSpace = getParam3f("gridWorldSpace", vec3f(1.f));
  // Get the volume dimensions.
  this->dimensions = getParam3i("dimensions", vec3i(0));


  _voxelAccel = (VoxelOctree*)getParamVoidPtr("voxelOctree",nullptr);
  if (!_voxelAccel) {
    throw std::runtime_error("TAMRVolume error: the voxelOctree must be set!");
  }


  // Pass the various parameters over to the ISPC side of the code
  ispc::TAMRVolume_set(getIE(),
                        (ispc::box3f*)&_voxelAccel->_actualBounds,
                        (ispc::vec3i &)this->dimensions,
                        (ispc::vec3f &)this->gridOrigin,
                        (ispc::vec3f &)this->gridWorldSpace,
                        (ispc::vec3f &)worldOrigin,
                        this,
                        sampler);

  auto filterMethodEnv = utility::getEnvVar<std::string>("OSPRAY_TAMR_METHOD");

  std::string filterMethod =
      filterMethodEnv.value_or(getParamString("amrMethod", "nearest"));

  if (filterMethod == "nearest")
    ispc::TAMR_install_nearest(getIE());
  else if (filterMethod == "current")
    ispc::TAMR_install_current(getIE());
  else if (filterMethod == "finest")
    ispc::TAMR_install_finest(getIE());
  else if (filterMethod == "octant")
    ispc::TAMR_install_octant(getIE());
  else if (filterMethod == "trilinear")
    ispc::TAMR_install_trilinear(getIE());

  ispc::TAMRVolume_setVoxelOctree(getIE(),
                                    _voxelAccel->_octreeNodes.data(),
                                    _voxelAccel->_octreeNodes.size(),
                                    (ispc::box3f*)&_voxelAccel->_actualBounds,
                                    (ispc::box3f*)&_voxelAccel->_virtualBounds);



  // PING;

}

// Not supported on p4est volume, throws an error
int TAMRVolume::setRegion(const void *source_pointer,
                           const vec3i &target_index,
                           const vec3i &source_count)
{
  throw std::runtime_error("P4est volume does not support set region!");
}

// This registers our volume type with the API so we can call
// ospNewVolume("p4est") to construct it
OSP_REGISTER_VOLUME(TAMRVolume, tamr);

extern "C" void ospray_init_module_tamr() {
  std::cout << "#osp: initializing the 'Tree-based AMR (TAMR)' module" << std::endl;
}

