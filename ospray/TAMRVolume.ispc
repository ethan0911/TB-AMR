// #include "TAMRVolume.ih"
// #include "FindCell.ih"
// #include "FindDualCell.ih"

#include "octant_stitch.ih"

// Functions that defined in CPP files
extern "C" unmasked uniform float P4est_scalar_sample(
    void *uniform cppObject, uniform vec3f *uniform samplePos);

extern "C" unmasked void P4est_scalar_batch_sample(
    void *uniform cppObject,
    uniform vec3f *uniform posBuffer,
    uniform int numActivePos,
    uniform int *uniform activeIDs,
    uniform float *uniform values);

// TODO gradient shading is not working
extern "C" unmasked uniform vec3f P4est_scalar_computeGradient(
    void *uniform cppObject, uniform vec3f *uniform samplePos);

// Sample the TAMRVolume at the world space coordinates
varying float TAMRVolume_sample(const void *uniform _self,
                                 const varying vec3f &worldCoordinates)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;

  uniform box3f worldBounds = make_box3f(self->worldOrigin + (self->_voxelAccel._actualBounds.lower - self->gridOrigin) * self->gridWorldSpace,
                                         self->worldOrigin + (self->_voxelAccel._actualBounds.upper  - self->gridOrigin) * self->gridWorldSpace);

  if (!box_contains(worldBounds, worldCoordinates))
    return -1.f;
  

#if 1
  // ISPC version voxel octree sampling, batch query

  float result = 0.05f;

  // float result = TAMR_nearest(_self, worldCoordinates);
  // float result = TAMR_current(_self,worldCoordinates);
  // float result = TAMR_finest(_self,worldCoordinates);
  // float result = TAMR_octant(_self,worldCoordinates);
  // float result = TAMR_Trilinear(_self, worldCoordinates);
  return result;

#elif 1
  // ISPC version voxel octree sampling, travese 8 times for a given point

  // vec3f lP;
  // self->transformWorldToLocal(self,worldCoordinates,lP);
  // CellRef cell = findLeafCell(self->_voxelAccel,lP);
  // return cell.value;

  vec3f lP;
  self->transformWorldToLocal(self, worldCoordinates, lP);

  CellRef cell = findLeafCell(self->_voxelAccel, lP);
  DualCell dCell;
  initDualCell(dCell, lP, cell.width);

  const vec3f _P0 =
      clamp(dCell.pos, make_vec3f(0.f), self->_voxelAccel._actualBounds.upper);
  const vec3f _P1 = clamp(dCell.pos + dCell.width,
                          make_vec3f(0.f),
                          self->_voxelAccel._actualBounds.upper);

  vec3f conners[8] = {_P0,
                      make_vec3f(_P1.x, _P0.y, _P0.z),
                      make_vec3f(_P0.x, _P1.y, _P0.z),
                      make_vec3f(_P1.x, _P1.y, _P0.z),
                      make_vec3f(_P0.x, _P0.y, _P1.z),
                      make_vec3f(_P1.x, _P0.y, _P1.z),
                      make_vec3f(_P0.x, _P1.y, _P1.z),
                      _P1};

  for (uniform int i = 0; i < 8; i++) {
    cell = findLeafCell(self->_voxelAccel, conners[i]);
    dCell.value[i] = cell.value;
    dCell.actualWidth[i] = cell.width;
    dCell.isLeaf[i] == (dCell.width == cell.width);
  }

  return lerp(dCell);

#elif 1
  // C++ version voxel octree sampling

  uniform vec3f uSamplePos[programCount];
  uniform float uReturnValue[programCount];

  vec3f lP;
  self->transformWorldToLocal(self, worldCoordinates, lP);

  uSamplePos[programIndex] = lP;
  foreach_active(lane)
  {
    uReturnValue[lane] =
        P4est_scalar_sample(self->cppSampler, &uSamplePos[lane]);
  }
  float result = uReturnValue[programIndex];
  return result;
#else
  // C++ version p4est travesal sampling.

  uniform vec3f uSamplePos[programCount];
  uniform float uReturnValue[programCount];

  uSamplePos[programIndex] = worldCoordinates;

  uniform int activeIDs[programCount];
  uniform int numActivePos = packed_store_active(activeIDs, programIndex);

  if (programIndex == 0) {
    // query num active values at a time
    P4est_scalar_batch_sample(
        self->cppSampler, uSamplePos, numActivePos, activeIDs, uReturnValue);
  }

  float result;
  if (isnan(uReturnValue[programIndex]))
    result = 0.0;
  else
    result = uReturnValue[programIndex];
  return result;

#endif
}

// Compute the gradient at the given sample location in world coordinates.
varying vec3f TAMRVolume_computeGradient(const void *uniform _self,
                                          const varying vec3f &worldCoordinates)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;
  // TODO: This can be somewhat implemented by knowing the grid size and
  // sampling different points at +/-x,y,z then computing a central difference
  // of each

    // Gradient step in each dimension (world coordinates).
  const uniform vec3f gradientStep = make_vec3f(1.0f);
  // The gradient will be computed using central differences.
  varying vec3f gradient;
    // Sample at gradient location.
  float sample = self->super.sample(self, worldCoordinates);
  // Gradient magnitude in the X direction.
  gradient.x = self->super.sample(self, worldCoordinates + make_vec3f(gradientStep.x, 0.0f, 0.0f)) - sample;
  // Gradient magnitude in the Y direction.
  gradient.y = self->super.sample(self, worldCoordinates + make_vec3f(0.0f, gradientStep.y, 0.0f)) - sample;
  // Gradient magnitude in the Z direction.
  gradient.z = self->super.sample(self, worldCoordinates + make_vec3f(0.0f, 0.0f, gradientStep.z)) - sample;
  // This approximation may yield image artifacts.
  return(gradient / gradientStep);

  // return make_vec3f(1f,0f,0f);
  // print("TAMRVolume_computeGradient NOT IMPLEMENTED!\n");
}

// Find the next sample point in the volume and advance the ray to it
void TAMRVolume_stepRay(const void *uniform _self,
                         varying Ray &ray,
                         const varying float samplingRate)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;

  // TODO: This must adapt to the size of the underlying grid at the AMR level
  // the ray is contained in so we don't miss features. Maybe intersect the
  // octree grid? Right now this basically assumes a uniform grid
  // The step size we choose for the highest quality should then be scaled by
  // the desired sampling rate, which will allow for downsampling to remain
  // interactive For now, just take ~20 samples through the box, adjusted by the
  // sampling rate

  // const float len =
  //     length(self->super.boundingBox.upper - self->super.boundingBox.lower);
  // float stepSize = len / (50.f * samplingRate);

  float stepSize = 0.f;

#if 0

  vec3f P = ray.org + ray.t0 * ray.dir;
  vec3f lP;
  self->transformWorldToLocal(self,P,lP);
  CellRef cell = findLeafCell(self->_voxelAccel,lP);

  stepSize = log(cell.width + 1.f) * self->super.samplingStep / samplingRate;

#else
  // skip the current sample if the opacity is 0

  CellRef cell;
  vec3f lP;
  while (ray.t0 < ray.t) {
    vec3f P = ray.org + ray.t0 * ray.dir;
    self->transformWorldToLocal(self, P, lP);
    cell = findLeafCell(self->_voxelAccel, lP);

    float opacity = self->super.transferFunction->getOpacityForValue(
        self->super.transferFunction, cell.value);
    
    stepSize = log(cell.width + 1.f) * self->super.samplingStep / samplingRate;
    
    if(opacity > 0.0f)
      break;

    ray.t0 += stepSize;
    ray.time = stepSize;
  }
#endif

  // vec3f nextP = P + stepSize * ray.dir;
  vec3f nextlP = lP + stepSize * normalize(ray.dir);
  // self->transformWorldToLocal(self,nextP,nextlP);
  CellRef nextCell = findLeafCell(self->_voxelAccel,nextlP);

  if (nextCell.width != cell.width) {
    stepSize = log(min(cell.width, nextCell.width) + 1.f) * self->super.samplingStep / samplingRate;
  }

  ray.t0 += stepSize;
  ray.time = stepSize;
}

// Find the ray-isosurface intersection in the volume for the passed ray and
// set of isovalues to display
void TAMRVolume_intersectIsosurface(
    const void *uniform _self,
    uniform float *uniform isovalues,
    uniform int numIsovalues,
    uniform size_t geomID,
    // ray.u contains ray-spacntersection error, i.e. the epsilone i
    varying Ray &ray)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;
  print("TAMRVolume_intersectIsosurface NOT IMPLEMENTED!\n");
}

export void *uniform TAMRVolume_createISPCEquivalent(void *uniform cppEquiv)
{
  uniform TAMRVolume *uniform v = uniform new uniform TAMRVolume;
  Volume_Constructor(&v->super, cppEquiv);

  // v->p4estTree      = NULL;
  // v->p4estTreeBytes = 0;

  // Setup the parent Volume
  v->super.cppEquivalent       = cppEquiv;
  // v->super.sample              = TAMRVolume_sample;
  v->super.computeGradient     = TAMRVolume_computeGradient;
  v->super.stepRay             = TAMRVolume_stepRay;
  v->super.intersectIsosurface = TAMRVolume_intersectIsosurface;
  return v;
}

export void TAMRVolume_freeVolume(void *uniform _self)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;
  delete self;
}

inline void TAMRVolume_transformLocalToWorld(const uniform TAMRVolume *uniform volume,
                                              const varying vec3f &localCoord,
                                              varying vec3f &worldCoord)
{
  worldCoord = volume->worldOrigin + (localCoord - volume->gridOrigin) * volume->gridWorldSpace;
}

inline void TAMRVolume_transformWorldToLocal(const uniform TAMRVolume *uniform volume,
                                              const varying vec3f &worldCoord,
                                              varying vec3f &localCoord)
{
  localCoord = rcp(volume->gridWorldSpace) * (worldCoord - volume->worldOrigin) + volume->gridOrigin;
}

// Matching extern declation to call into the C++ function
extern "C" void ispc_called_cpp_function_example(uniform int val);

export void TAMRVolume_set(void *uniform _self,
                            //void *uniform p4estTree,
                            //uniform uint64 p4estTreeBytes,
                            uniform box3f *uniform bounds,
                            const uniform vec3i &dimensions,
                            const uniform vec3f &gridOrigin,
                            const uniform vec3f &gridWorldSpace,
                            const uniform vec3f &worldOrigin,
                            /*! pointer to the c++ side object */
                            void *uniform cppObject,
                            void *uniform cppSampler)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;
  

  // self->p4estTree      = p4estTree;
  // self->p4estTreeBytes = p4estTreeBytes;

  self->dimensions  = dimensions;
  self->gridOrigin  = gridOrigin;
  self->gridWorldSpace = gridWorldSpace;
  self->worldOrigin = worldOrigin;

  // self->super.boundingBox = *bounds;
  self->super.boundingBox =
      make_box3f(self->worldOrigin +
                     (bounds->lower - self->gridOrigin) * self->gridWorldSpace,
                 self->worldOrigin +
                     (bounds->upper - self->gridOrigin) * self->gridWorldSpace);

  self->cppObject  = cppObject;
  self->cppSampler = cppSampler;

  self->transformLocalToWorld = TAMRVolume_transformLocalToWorld;
  self->transformWorldToLocal = TAMRVolume_transformWorldToLocal;

  // TODO: This should be set in some way to be our minimum sampling step
  // size, so whatever our finest step should be within the smallest leaf cell
  // in the dataset? const uniform float len =
  //     length(self->super.boundingBox.upper -
  //     self->super.boundingBox.lower);
  // self->super.samplingStep = len / 20.f;

  self->super.samplingStep = gridWorldSpace.x;
}

export void TAMRVolume_setVoxelOctree(void *uniform _self,
                                       void *uniform octreeNodes,
                                       uniform unsigned int64 oNodeNum,
                                       uniform box3f *uniform actualBounds,
                                       uniform box3f *uniform virtualBounds)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;

  uniform VoxelOctreeNode *uniform nodes =
      (uniform VoxelOctreeNode * uniform) octreeNodes;
  self->_voxelAccel._actualBounds        = *actualBounds;
  self->_voxelAccel._virtualBounds = *virtualBounds;
  self->_voxelAccel._octreeNodes   = nodes;
  self->_voxelAccel._oNodeNum      = oNodeNum;

  // // vec3f pos    = make_vec3f(1.5f, 3.2f, 0.1f);
  // // vec3f wpos    = make_vec3f(-0.33875f, -1.163f,-1.34381f);
  // vec3f wpos    = make_vec3f(1.5f, 1.5f, 1.5f);
  // vec3f lpos;
  // self->transformWorldToLocal(self, wpos, lpos);
  // CellRef cell = findLeafCell(self->_voxelAccel, lpos);
  // PRINT(cell.value);
  // PRINT(cell.width);

  // Octant O;
  // DualCell D;
  // doTrilinear(self, cell, lpos, O, D);

  // print("O:value:[%,%,%,%,%,%,%,%]\n",
  //       O.value[0],
  //       O.value[1],
  //       O.value[2],
  //       O.value[3],
  //       O.value[4],
  //       O.value[5],
  //       O.value[6],
  //       O.value[7]);

  // print("D::AcualWidth:[%,%,%,%,%,%,%,%]\n",
  //       D.actualWidth[0],
  //       D.actualWidth[1],
  //       D.actualWidth[2],
  //       D.actualWidth[3],
  //       D.actualWidth[4],
  //       D.actualWidth[5],
  //       D.actualWidth[6],
  //       D.actualWidth[7]);

  // vec3f wPos; 
  // self->transformLocalToWorld(self, pos, wPos);
  // float value = self->super.sample(self,wPos);
  // PRINT(value);

  // DualCell dcell;
  // initDualCell(dcell, pos, cell.width);
  // print("dual:C000:[%,%,%]\n", dcell.pos.x, dcell.pos.y, dcell.pos.z);
  // print("dual:C111:[%,%,%]\n",
  //       dcell.pos.x + dcell.width,
  //       dcell.pos.y + dcell.width,
  //       dcell.pos.z + dcell.width);

  // findDualCell(self->_voxelAccel,dcell);
  // print("dual::AcualWidth:[%,%,%,%,%,%,%,%]\n",
  //       dcell.actualWidth[0],dcell.actualWidth[1],
  //       dcell.actualWidth[2],dcell.actualWidth[3],
  //       dcell.actualWidth[4],dcell.actualWidth[5],
  //       dcell.actualWidth[6],dcell.actualWidth[7]);

  // print("dual:value:[%,%,%,%,%,%,%,%]\n",
  //         dcell.value[0],dcell.value[1],
  //         dcell.value[2],dcell.value[3],
  //         dcell.value[4],dcell.value[5],
  //         dcell.value[6],dcell.value[7]);

  //doTrilinear(_self, cell, pos);

  // unsigned int8 test = 2 * programIndex;
  // int d= 3;
  // unsigned int a = (test +1) % d;
  // PRINT(a);

}
