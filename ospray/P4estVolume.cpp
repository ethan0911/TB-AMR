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

P4estVolume::P4estVolume() {
  // Create our ISPC-side version of the struct
  ispcEquivalent = ispc::P4estVolume_createISPCEquivalent(this);
}
P4estVolume::~P4estVolume() {
  ispc::P4estVolume_freeVolume(ispcEquivalent);
}

std::string P4estVolume::toString() const {
  return "P4estVolume";
}

void P4estVolume::commit() {
  // The ping macro will print the file/line/function info to cout
  PING;
  Volume::commit();
  updateEditableParameters();

  p4estTree = getParamData("p4estTree");
  if (!p4estTree) {
    throw std::runtime_error("P4estVolume error: A p4estTree buffer must be set");
  }
  std::cout << "tree has " << p4estTree->size() << "bytes\n";

  ospcommon::box3f bounds(vec3f(-1.f), vec3f(1.f));

  // Pass the various parameters over to the ISPC side of the code
  ispc::P4estVolume_set(getIE(),
                        p4estTree->data,
                        p4estTree->size(),
                        (ispc::box3f*)&bounds);
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

