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
  // Create our ISPC-side version of the struct
  ispcEquivalent = ispc::P4estVolume_createISPCEquivalent(this);
}
P4estVolume::~P4estVolume() {
  ispc::P4estVolume_freeVolume(ispcEquivalent);
  delete sampler;
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

  p4est = (p4est_t*)getParamVoidPtr("p4estTree",nullptr);
  if (!p4est) {
    throw std::runtime_error("P4estVolume error: A p4estTree buffer must be set");
  }

  treeID = getParam1i("treeID", -1);
  if (treeID < 0) {
    throw std::runtime_error("P4estVolume error: A treeID must be set!");
  }

  std::cout << "P4estVolume treeID = " << treeID << "\n";

  //FIXME: p4eset_ospray_data_t is not defined? Need to edit p4est_to_p8est.h ?  
  data_callback = (p8est_ospray_data_t)getParamVoidPtr("p4estDataCallback",nullptr);
  if (!data_callback) {
    throw std::runtime_error("P4estVolume error: A p4est data callback must be set");
  }

  //get the bbox of the tree
  double bbox[6] = {0.0};
  p4est_ospray_tree_aabb(p4est, treeID, bbox);

  std::cout << "tree has " << p4est->data_size << " bytes\n";

  std::cout<<"aabb:("<<bbox[0]<<","<<bbox[1]<<","<<bbox[2]<<","<<bbox[3]<<","<<bbox[4]<<","<<bbox[5]<<")" <<std::endl;

  ospcommon::box3f bounds(vec3f(bbox[0], bbox[1], bbox[2]),
                          vec3f(bbox[3], bbox[4], bbox[5]));

  this->sampler = createSampler();

  // Pass the various parameters over to the ISPC side of the code
  ispc::P4estVolume_set(getIE(),
                        /*p4estTree->data,*/
                        /*p4estTree->size(),*/
                        p4est,
                        1,
                        (ispc::box3f*)&bounds,
                        this,
                        sampler);
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

