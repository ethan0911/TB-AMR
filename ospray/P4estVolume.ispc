#include "ospray/math/box.ih"
#include "ospray/math/vec.ih"
#include "P4estVolume.ih"

// Functions that defined in CPP files
extern "C" unmasked uniform float P4est_scalar_sample(void *uniform cppObject,
    uniform vec3f *uniform samplePos);
// TODO gradient shading is not working
extern "C" unmasked uniform vec3f P4est_scalar_computeGradient(void *uniform cppObject,
    uniform vec3f *uniform samplePos);


// Sample the P4estVolume at the world space coordinates
varying float P4estVolume_sample(const void *uniform _self,
                                 const varying vec3f &worldCoordinates)
{
  uniform P4estVolume *uniform self = (uniform uniform P4estVolume *uniform)_self;

  uniform vec3f uSamplePos[programCount];
  uniform float uReturnValue[programCount];
  uSamplePos[programIndex] = worldCoordinates;
  foreach_active(lane)
  {
    uReturnValue[lane] =
        P4est_scalar_sample(self->cppSampler, &uSamplePos[lane]);
  }
  float result = uReturnValue[programIndex];

  // TODO: Sample the volume
  return result;
}

// Compute the gradient at the given sample location in world coordinates.
varying vec3f P4estVolume_computeGradient(const void *uniform _self,
                                          const varying vec3f &worldCoordinates)
{
  uniform P4estVolume *uniform self = (uniform uniform P4estVolume *uniform)_self;
  // TODO: This can be somewhat implemented by knowing the grid size and sampling
  // different points at +/-x,y,z then computing a central difference of each
  print("P4estVolume_computeGradient NOT IMPLEMENTED!\n");
}

// Find the next sample point in the volume and advance the ray to it
void P4estVolume_stepRay(const void *uniform _self,
                         varying Ray &ray,
                         const varying float samplingRate)
{
  uniform P4estVolume *uniform self = (uniform uniform P4estVolume *uniform)_self;
  const float len = length(self->super.boundingBox.upper - self->super.boundingBox.lower);
  // TODO: This must adapt to the size of the underlying grid at the AMR level
  // the ray is contained in so we don't miss features. Maybe intersect the
  // octree grid?
  // The step size we choose for the highest quality should then be scaled by
  // the desired sampling rate, which will allow for downsampling to remain interactive
  // For now, just take ~20 samples through the box, adjusted by the sampling rate
  
  const float stepSize = len / (20.f * samplingRate);
  ray.t0 += stepSize;
  ray.time = stepSize;
}

// Find the ray-isosurface intersection in the volume for the passed ray and
// set of isovalues to display
void P4estVolume_intersectIsosurface(const void *uniform _self,
                                     uniform float *uniform isovalues,
                                     uniform int numIsovalues,
                                     uniform size_t geomID,
                                     // ray.u contains ray-space intersection error, i.e. the epsilon
                                     varying Ray &ray)
{
  uniform P4estVolume *uniform self = (uniform uniform P4estVolume *uniform)_self;
  print("P4estVolume_intersectIsosurface NOT IMPLEMENTED!\n");
}

export void *uniform P4estVolume_createISPCEquivalent(void *uniform cppEquiv) {
  uniform P4estVolume *uniform v = uniform new uniform P4estVolume;
  v->p4estTree = NULL;
  v->p4estTreeBytes = 0;

  // Setup the parent Volume
  v->super.cppEquivalent = cppEquiv;
  v->super.sample = P4estVolume_sample;
  v->super.computeGradient = P4estVolume_computeGradient;
  v->super.stepRay = P4estVolume_stepRay;
  v->super.intersectIsosurface = P4estVolume_intersectIsosurface;
  return v;
}

export void P4estVolume_freeVolume(void *uniform _self) {
  uniform P4estVolume *uniform self = (uniform uniform P4estVolume *uniform)_self;
  delete self;
}

// Matching extern declation to call into the C++ function
extern "C" void ispc_called_cpp_function_example(uniform int val);

export void P4estVolume_set(void *uniform _self,
                            void *uniform p4estTree,
                            uniform uint64 p4estTreeBytes,
                            uniform box3f *uniform bounds,
                            /*! pointer to the c++ side object */
                            void *uniform cppObject,
                            void *uniform cppSampler)
{
  uniform P4estVolume *uniform self = (uniform uniform P4estVolume *uniform)_self;

  self->p4estTree = p4estTree;
  self->p4estTreeBytes = p4estTreeBytes;
  self->super.boundingBox = *bounds;

  self->cppObject = cppObject;
  self->cppSampler = cppSampler;

  // This is an example of how we can serialize the program execution to call
  // back in to C++ from ISPC
  // foreach_active(i) {
  //   ispc_called_cpp_function_example(i);
  // }
}

