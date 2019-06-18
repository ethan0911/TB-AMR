#include <iostream>
#include <stdexcept>
#include "ospcommon/vec.h"
#include "ospcommon/box.h"

// Our exported ISPC functions are in this _ispc.h
#include "P4estVolume_ispc.h"
#include "P4estVolume.h"

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

P4estVolume::P4estVolume() {
  p4est = nullptr; //init to null for safety

  // Create our ISPC-side version of the struct
  ispcEquivalent = ispc::P4estVolume_createISPCEquivalent(this);
}

P4estVolume::~P4estVolume() {
  ispc::P4estVolume_freeVolume(ispcEquivalent);
  delete sampler;

  if(_voxelAccel)
    delete _voxelAccel;
}

std::string P4estVolume::toString() const {
  return "P4estVolume";
}


ScalarVolumeSampler* P4estVolume::createSampler(){
  return new P4estVolumeSampler(this);
}

void P4estVolume::commit() {
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
    throw std::runtime_error("P4estVolume error: the voxelOctree must be set!");
  }

  // Pass the various parameters over to the ISPC side of the code
  ispc::P4estVolume_set(getIE(),
                        (ispc::box3f*)&_voxelAccel->_actualBounds,
                        (ispc::vec3i &)this->dimensions,
                        (ispc::vec3f &)this->gridOrigin,
                        (ispc::vec3f &)this->gridWorldSpace,
                        (ispc::vec3f &)worldOrigin,
                        this,
                        sampler);


  ispc::P4estVolume_setVoxelOctree(getIE(),
                                    _voxelAccel->_octreeNodes.data(),
                                    _voxelAccel->_octreeNodes.size(),
                                    (ispc::box3f*)&_voxelAccel->_actualBounds,
                                    (ispc::box3f*)&_voxelAccel->_virtualBounds);

  // PING;

}

// Not supported on p4est volume, throws an error
int P4estVolume::setRegion(const void *source_pointer,
                           const vec3i &target_index,
                           const vec3i &source_count)
{
  throw std::runtime_error("P4est volume does not support set region!");
}

// This registers our volume type with the API so we can call
// ospNewVolume("p4est") to construct it
OSP_REGISTER_VOLUME(P4estVolume, p4est);

extern "C" void ospray_init_module_p4est() {
  std::cout << "initializing the p4est module" << std::endl;
}

