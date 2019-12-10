#include "transferFunction/TransferFunction.ih"
#include "octant_stitch.ih"
#include "TAMRVolumeIntegrate.ih"

// Functions that defined in CPP files
extern "C" unmasked uniform float P4est_scalar_sample(
    void *uniform cppObject, uniform vec3f *uniform samplePos);

// TODO gradient shading is not working
extern "C" unmasked uniform vec3f P4est_scalar_computeGradient(
    void *uniform cppObject, uniform vec3f *uniform samplePos);

// Sample the TAMRVolume at the world space coordinates
varying float TAMRVolume_sample(const void *uniform _self,
                                 const varying vec3f &worldCoordinates)
{
  print("SHOULD NOT BE CALLED\n");
  float result = 0.05f;
  return result;

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
                        TransferFunction *uniform transferFunction,
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


  float stepSize = 0.f;

  // TODO WILL: The below code does not traverse and sample the octree
  // correctly stepping from cell to cell. Need to re-implement
  // Also todo: We'll probably get better perf if we just do the full integration
  // traversal here instead of constantly re-calling step. We basically want
  // something more like the vkl ray iterators

  stepSize = self->super.samplingStep / samplingRate;

  uniform vec3f boundSize = box_size(self->_voxelAccel._virtualBounds);
  uniform float width     = boundSize.x;
  const vec3f ray_rdir = rcp(ray.dir);

    // sign of direction determines near/far index
  const vec3i nextCellIndex = make_vec3i(1 - (intbits(ray.dir.x) >> 31),
                                         1 - (intbits(ray.dir.y) >> 31),
                                         1 - (intbits(ray.dir.z) >> 31));

  while (ray.t0 < ray.t) {
    ray.t0 += stepSize;
    ray.time = stepSize;

    vec3f lP;
    vec3f P = ray.org + ray.t0 * ray.dir;
    self->transformWorldToLocal(self, P, lP);

    const vec3f localCoord = 
        clamp(lP,
              make_vec3f(0.f),
              self->_voxelAccel._actualBounds.upper - make_vec3f(0.000001f));

    uniform VOStack stack[32];
    uniform VOStack *uniform stackPtr =
        pushStack(&stack[0], 0, self->_voxelAccel._virtualBounds.lower, width);

    while (stackPtr > stack) {
      --stackPtr;
      if (stackPtr->active) {
        const unsigned int64 nodeID   = stackPtr->pNodeIdx;
        const vec3f pos               = stackPtr->pos;
        const uniform float cellWidth = stackPtr->width;

        if (nodeID >= self->_voxelAccel._oNodeNum) 
          return;

        const uniform VoxelOctreeNode *pNode = getOctreeNode(self->_voxelAccel, nodeID);

        vec2f vRange = make_vec2f(pNode->vRange.lower, pNode->vRange.upper);
        // Get the maximum opacity in the volumetric value range.
        float maximumOpacity =
            transferFunction->getMaxOpacityInRange(transferFunction, vRange);

        // Return the hit point if the grid cell is not fully transparent.
        // current node is fully transparent, march to the exit point
        if (maximumOpacity <= 0.0f) {
          // Exit bound of the grid cell in world coordinates.
          vec3f farBound;
          self->transformLocalToWorld(self, pos + to_float(nextCellIndex) * cellWidth,farBound);

          // Identify the distance along the ray to the exit points on the cell.
          const vec3f maximum = ray_rdir * (farBound - ray.org);
          const float exitDist = min(min(ray.t, maximum.x), min(maximum.y, maximum.z));

          // Advance the ray so the next hit point will be outside the empty cell.
          const float dist = ceil(abs(exitDist - ray.t0) / stepSize) * stepSize;
          ray.t0 += dist;
          ray.time = cellWidth;
          break;
        } else if (isLeaf(pNode)) {
          // Exit bound of the grid cell in world coordinates.
          vec3f farBound;
          self->transformLocalToWorld(self, pos + to_float(nextCellIndex) * cellWidth,farBound);

          // Identify the distance along the ray to the exit points on the cell.
          const vec3f maximum = ray_rdir * (farBound - ray.org);
          const float exitDist = min(min(ray.t, maximum.x), min(maximum.y, maximum.z));

          float dist = ceil(abs(exitDist - ray.t0) / stepSize) * stepSize;
          dist       = min((cellWidth - 1.f) * stepSize, dist);

          ray.t0 += dist;
          ray.time = cellWidth;
          return;
        } else {
          vec3f center             = pos + make_vec3f(cellWidth * 0.5f);
          unsigned int8 octantMask = 0;
          if (localCoord.x >= center.x)
            octantMask |= 1;
          if (localCoord.y >= center.y)
            octantMask |= 2;
          if (localCoord.z >= center.z)
            octantMask |= 4;

          unsigned int8 childMask    = getChildMask(pNode);
          unsigned int64 childOffset = getChildOffset(pNode);

          bool hasChild = childMask & (1 << octantMask);
          // no leaf(no voxel), return invalid value 0.0.
          if (!hasChild) {
            return;
          }

          unsigned int8 rightSibling = (1 << octantMask) - 1;
          unsigned int8 childIndex = BIT_COUNT[childMask & rightSibling];
          unsigned int64 childNodeID = nodeID + childOffset + childIndex;

          vec3f lowerPos =
              pos + make_vec3f((octantMask & 1) ? cellWidth * 0.5 : 0.0,
                               (octantMask & 2) ? cellWidth * 0.5 : 0.0,
                               (octantMask & 4) ? cellWidth * 0.5 : 0.0);
          stackPtr =
              pushStack(stackPtr, childNodeID, lowerPos, cellWidth * 0.5f);
        }
      }
    }
  }
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

  // Setup the parent Volume
  v->super.cppEquivalent       = cppEquiv;
  v->super.computeGradient     = TAMRVolume_computeGradient;
  v->super.stepRay             = TAMRVolume_stepRay;
  v->super.intersectIsosurface = TAMRVolume_intersectIsosurface;
#if 1
  v->super.integrateVolumeInterval = TAMRVolume_integrateVolumeInterval;
#endif
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
                            const uniform int samplesPerCell,
                            const uniform float opacityScaleFactor,
                            /*! pointer to the c++ side object */
                            void *uniform cppObject,
                            void *uniform cppSampler)
{
  uniform TAMRVolume *uniform self =
      (uniform uniform TAMRVolume * uniform) _self;
  

  self->dimensions  = dimensions;
  self->gridOrigin  = gridOrigin;
  self->gridWorldSpace = gridWorldSpace;
  self->worldOrigin = worldOrigin;
  self->samplesPerCell = samplesPerCell;
  self->opacityScaleFactor = opacityScaleFactor;

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
  // in the dataset? 

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
}
