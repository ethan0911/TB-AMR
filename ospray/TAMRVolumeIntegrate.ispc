#include "ospray/volume/VolumetricModel.ih"
#include "ospray/common/Ray.ih"
#include "transferFunction/TransferFunction.ih"
#include "octant_stitch.ih"
#include "TAMRVolumeIntegrate.ih"

void TAMRVolume_integrateVolumeInterval(const void *uniform _self,
                                        TransferFunction *uniform tfn,
                                        varying Ray &ray,
                                        const varying range1f &interval)
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

  // TODO: I'll send back the cell width in the "time" param, which will
  // be used as our sample distance. Then the TAMR volume will give us
  // back something like the "min cell width" which would be our base sampling rate.
  //sampleAlpha = 1.f - powf(1.f - sampleAlpha, (ray.t0 - lastSamplePos) / ray.time);

  float stepSize = 0.f;
  // TODO WILL: The below code does not traverse and sample the octree
  // correctly stepping from cell to cell. Need to re-implement
  // Also todo: We'll probably get better perf if we just do the full integration
  // traversal here instead of constantly re-calling step. We basically want
  // something more like the vkl ray iterators

  stepSize = self->super.samplingStep;

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
        float maximumOpacity = tfn->getMaxOpacityInRange(tfn, vRange);

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


