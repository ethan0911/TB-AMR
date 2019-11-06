#include "TAMRVolumeIntegrate.ih"
#include "octant_stitch.ih"
#include "ospray/common/Ray.ih"
#include "ospray/render/Renderer.ih"
#include "ospray/volume/VolumetricModel.ih"
#include "transferFunction/TransferFunction.ih"
#include "render/util.ih"

#if 1
#define DEBUG(pixel, X)
#else
#define DEBUG(pixel, X)                   \
  if ((pixel.x == 512 && pixel.y == 384) || (pixel.x == 522 && pixel.y == 384)) { \
    X;                                    \
  }
#endif

inline float get(const varying vec3f &v, const uint32 dim)
{
  return (&v.x)[dim];
}

inline int32 get(const varying vec3i &v, const uint32 dim)
{
  return (&v.x)[dim];
}

bool fastBoxIntersect(const varying float *uniform box,
    const varying vec3f &rayOrigin,
    const varying vec3f &invDir,
    const varying vec3i &negDir,
    varying range1f &interval);

struct WVOStack
{
  bool active;
  unsigned int64 pNodeIdx;
  // TODO: Would be good to not store the pos in the stack to reduce mem use
  vec3f pos;
  float width;
};

inline varying WVOStack *uniform pushWStack(varying WVOStack *uniform stackPtr,
    varying unsigned int64 pNodeIdx,
    varying vec3f pos,
    varying float width)
{
  unmasked
  {
    stackPtr->active = false;
  }

  stackPtr->active   = true;
  stackPtr->pNodeIdx = pNodeIdx;
  stackPtr->pos      = pos;
  stackPtr->width    = width;

  return stackPtr + 1;
}

vec4f TAMRVolume_integrateVolumeInterval(const void *uniform _self,
    TransferFunction *uniform tfn,
    varying Ray &ray,
    const varying range1f &interval,
    const varying ScreenSample &sample)
{
  // TODO WILL: We also want to do this traversal for the isosurface rendering

  uniform TAMRVolume *uniform self =
    (uniform uniform TAMRVolume * uniform) _self;

  float jitter = precomputedHalton2(sample.sampleID.z);
  int ix       = sample.sampleID.x % 4;
  int iy       = sample.sampleID.y % 4;

  int patternID = ix + 4 * iy;
  jitter += precomputedHalton3(patternID);

  if (jitter > 1.f) {
    jitter -= 1.f;
  }

  /* at each node we determine which sides of the splitting planes the ray
     is along (similar to a k-d tree ray traversal). The main difference is
     we have 3 planes, and they're defined implicitly by the octree.
     Each test tells us if we're above/below/both of the plane, which combines
     to give us a mask of which child nodes we should descend into. We can
     then order the nodes front to back and continue traversal. If we find
     a leaf node we sample it at the midpoint of the ray interval through
     the cell, which we should get from the t values from the plane test which
     sent us into this cell
   */

  vec3f color      = make_vec3f(0.f);
  float alpha      = 0.f;
  float prevSample = -1.f;

  const uniform unsigned int8 axisMasks[3] = {0x01, 0x02, 0x04};

  vec3f localRayOrg;
  self->transformWorldToLocal(self, ray.org, localRayOrg);

  const vec3f localRayDir = rcp(self->gridWorldSpace) * ray.dir;

  // interval has our ray's intersection with the volume bounds already
  const vec3f invDir = rcp(localRayDir);
  const vec3i negDir = make_vec3i(localRayDir.x < 0 ? 1 : 0,
      localRayDir.y < 0 ? 1 : 0,
      localRayDir.z < 0 ? 1 : 0);

  const uniform vec3f boundSize = box_size(self->_voxelAccel._virtualBounds);
  WVOStack stack[64];
  varying WVOStack *uniform stackPtr = pushWStack(
      &stack[0], 0, self->_voxelAccel._virtualBounds.lower, boundSize.x);

  vec3f tSplitPlanes;
  // TODO: Maybe keep the cell interval in the stack, then we can eliminate
  // these box tests
  varying float cellBounds[6];

  while (stackPtr > stack) {
    --stackPtr;

    if (stackPtr->active) {
      const unsigned int64 nodeID = stackPtr->pNodeIdx;
      const vec3f cellPos         = stackPtr->pos;
      const float cellWidth       = stackPtr->width;
      const float halfWidth       = 0.5f * cellWidth;
      const vec3f cellCenter      = cellPos + make_vec3f(halfWidth);
      cellBounds[0]               = cellPos.x;
      cellBounds[1]               = cellPos.y;
      cellBounds[2]               = cellPos.z;
      cellBounds[3]               = cellPos.x + cellWidth;
      cellBounds[4]               = cellPos.y + cellWidth;
      cellBounds[5]               = cellPos.z + cellWidth;

      DEBUG(sample.sampleID, {
          foreach_active(i) {
            print("Pixel %, %\n", extract(sample.sampleID.x, i), extract(sample.sampleID.y, i));
            print("Traverse cell %, pos: %, %, %, width: %\n",
                extract(nodeID, i),
                extract(cellPos.x, i),
                extract(cellPos.y, i),
                extract(cellPos.z, i),
                extract(cellWidth, i));
          }
      });

      const uniform VoxelOctreeNode *pNode =
        getOctreeNode(self->_voxelAccel, nodeID);

      const vec2f vRange = make_vec2f(pNode->vRange.lower, pNode->vRange.upper);

      // Get the maximum opacity in the volumetric value range.
      // const float maximumOpacity = tfn->getMaxOpacityInRange(tfn, vRange);
      const float maximumOpacity = 1.f;

      if (maximumOpacity > 0.f) {
        range1f cellInterval;
        if (!fastBoxIntersect(
              cellBounds, localRayOrg, invDir, negDir, cellInterval)) {
          continue;
        }
        cellInterval.lower = max(interval.lower, cellInterval.lower);
        cellInterval.upper = min(interval.upper, cellInterval.upper);

        if (!isLeaf(pNode)) {
          // Inner node: test which side(s) we're on of each of the octree
          // splitting planes and determine which children to traverse
          tSplitPlanes.x = (cellPos.x + halfWidth - localRayOrg.x) * invDir.x;
          tSplitPlanes.y = (cellPos.y + halfWidth - localRayOrg.y) * invDir.y;
          tSplitPlanes.z = (cellPos.z + halfWidth - localRayOrg.z) * invDir.z;

          const float tSplitMin =
            min(tSplitPlanes.x, tSplitPlanes.y, tSplitPlanes.z);
          const float tSplitMax =
            max(tSplitPlanes.x, tSplitPlanes.y, tSplitPlanes.z);

          // Order in which we intersect the splitting planes along the ray,
          // back to front, since we need to push on the stack in reverse order
          vec3i traverseOrder;
          if (tSplitMin == tSplitPlanes.x) {
            traverseOrder.z = 0;
          } else if (tSplitMin == tSplitPlanes.y) {
            traverseOrder.z = 1;
          } else {
            traverseOrder.z = 2;
          }
          if (tSplitMax == tSplitPlanes.x) {
            traverseOrder.x = 0;
          } else if (tSplitMax == tSplitPlanes.y) {
            traverseOrder.x = 1;
          } else {
            traverseOrder.x = 2;
          }
          if (traverseOrder.x != 0 && traverseOrder.z != 0) {
            traverseOrder.y = 0;
          } else if (traverseOrder.x != 1 && traverseOrder.z != 1) {
            traverseOrder.y = 1;
          } else {
            traverseOrder.y = 2;
          }

          // Push each child the ray needs to traverse onto the stack, in the
          // order they should be traversed from front to back.
          // Note: We push the nodes on in reverse order (back to front) so that
          // when we pop the stack we'll go front to back order.
          vec3f exitPos = localRayOrg + localRayDir * cellInterval.upper;
          unsigned int8 octantMask = 0;
          if (exitPos.x >= cellCenter.x) {
            octantMask |= 1;
          }
          if (exitPos.y >= cellCenter.y) {
            octantMask |= 2;
          }
          if (exitPos.z >= cellCenter.z) {
            octantMask |= 4;
          }

          const unsigned int8 childMask    = getChildMask(pNode);
          const unsigned int64 childOffset = getChildOffset(pNode);
          if (childMask & (1 << octantMask)) {
            unsigned int8 rightSibling = (1 << octantMask) - 1;
            // Note: just popcnt
            unsigned int8 childIndex   = BIT_COUNT[childMask & rightSibling];
            unsigned int64 childNodeID = nodeID + childOffset + childIndex;
            vec3f lowerPos =
              cellPos + make_vec3f((octantMask & 1) ? halfWidth : 0.0,
                  (octantMask & 2) ? halfWidth : 0.0,
                  (octantMask & 4) ? halfWidth : 0.0);

            DEBUG(sample.sampleID, {
                foreach_active(i)
                {
                print("Pixel %, %\n", extract(sample.sampleID.x, i), extract(sample.sampleID.y, i));
                print("push exit child %, octantmask: %\n",
                    extract(childNodeID, i),
                    extract(octantMask, i));
                }
                })

            stackPtr = pushWStack(stackPtr, childNodeID, lowerPos, halfWidth);
          }

          // To update the child mask for the traversal we can just do an XOR
          // with the mask for the axis (x: 0b001, y: 0b010, z: 0b100).
          // At most we intersect all 3 planes of the octree
          for (uniform int i = 0; i < 3; ++i) {
            // Check that the traversal intervals are within the cell range,
            // otherwise set to -1 to skip them
            const int axis = get(traverseOrder, i);
            const float t  = get(tSplitPlanes, axis);
            if (t > cellInterval.lower && t < cellInterval.upper) {
              octantMask = octantMask ^ axisMasks[axis];

              if (childMask & (1 << octantMask)) {
                unsigned int8 rightSibling = (1 << octantMask) - 1;
                unsigned int8 childIndex   = BIT_COUNT[childMask & rightSibling];
                unsigned int64 childNodeID = nodeID + childOffset + childIndex;
                vec3f lowerPos =
                  cellPos + make_vec3f((octantMask & 1) ? halfWidth : 0.0,
                      (octantMask & 2) ? halfWidth : 0.0,
                      (octantMask & 4) ? halfWidth : 0.0);

                DEBUG(sample.sampleID, {
                    foreach_active(i)
                    {
                    print("Pixel %, %\n", extract(sample.sampleID.x, i), extract(sample.sampleID.y, i));
                    print("push traverse child %, octantmask: %, t = %\n",
                        extract(childNodeID, i),
                        extract(octantMask, i),
                        extract(t, i));
                    }
                    })
                stackPtr =
                  pushWStack(stackPtr, childNodeID, lowerPos, halfWidth);
              }
            }
          }
        }
        else
        {
          // Sample the leaf at the midpoint of the ray interval along this
          // node
          // TODO: Seems like this gives some odd values for the opacity?
          // is traversal correct? interpolation?
          CellRef cell               = {cellPos, cellWidth, (float)getValue(pNode)};
          float intervalLength = cellInterval.upper - cellInterval.lower;
          // Empty intervals will end up with 0 opacity anyway, so just skip
          if (intervalLength < cellWidth * 0.001) {
            continue;
          }

          // Split the cell overlap into two intervals, and sample them
          intervalLength = intervalLength / self->samplesPerCell;
          for (uniform int i = 0; i < self->samplesPerCell; ++i) {
            const float samplet = cellInterval.lower + i * intervalLength + jitter * intervalLength;
            vec3f samplePos = localRayOrg + localRayDir * samplet;

#if 1
            Octant octant;
            DualCell dualCell;
            const float value =
              doTrilinear(self, cell, samplePos, octant, dualCell);
#else
            const float value = cell.value;
#endif
            vec3f sampleColor = tfn->getIntegratedColorForValue(tfn, prevSample, value);
            float sampleAlpha = tfn->getIntegratedOpacityForValue(tfn, prevSample, value);

            sampleAlpha = 1.f - powf(1.f - sampleAlpha, intervalLength);

            sampleAlpha = clamp(sampleAlpha);
            sampleColor = sampleColor * sampleAlpha;

            color = color + ((1.f - alpha) * sampleColor);
            alpha = alpha + ((1.f - alpha) * sampleAlpha);

            alpha      = clamp(alpha);
            prevSample = value;

            DEBUG(sample.sampleID, {
                foreach_active(i) {
                print("Pixel %, %\n", extract(sample.sampleID.x, i), extract(sample.sampleID.y, i));
                print("sampling cell %\n", extract(nodeID, i));
                print("\tinterval [%, %], length: %\n",
                    extract(cellInterval.lower, i),
                    extract(cellInterval.upper, i),
                    extract(intervalLength, i));
                print("\tsamplet %\n", extract(samplet, i));
                print("\tjitter %\n", extract(jitter, i));
                vec3f cellenter = localRayOrg + localRayDir * cellInterval.lower;
                vec3f cellexit = localRayOrg + localRayDir * cellInterval.upper;
                print("\tcellenter: %, %, %\n",
                    extract(cellenter.x, i),
                    extract(cellenter.y, i),
                    extract(cellenter.z, i));
                print("\tcellexit: %, %, %\n",
                    extract(cellexit.x, i),
                    extract(cellexit.y, i),
                    extract(cellexit.z, i));
                print("\tsamplePos: %, %, %, alpha: %\n",
                    extract(samplePos.x, i),
                    extract(samplePos.y, i),
                    extract(samplePos.z, i),
                    extract(sampleAlpha, i));
                }
            });
          }
          if (alpha >= 0.99f) {
            DEBUG(sample.sampleID, print("Opaque\n"));
            break;
          }
        }
      }
    }
  }
  DEBUG(sample.sampleID, {
      print("=================\n");
      return make_vec4f(0.f, 0.f, 0.f, 1.f);
      })
  return make_vec4f(color, alpha);
}

bool fastBoxIntersect(const varying float *uniform box,
    const varying vec3f &rayOrig,
    const varying vec3f &invDir,
    const varying vec3i &negDir,
    varying range1f &interval)
{
  // Check X & Y intersection
  float tmin = (box[3 * negDir.x] - rayOrig.x) * invDir.x;
  float tmax = (box[3 * (1 - negDir.x)] - rayOrig.x) * invDir.x;

  float tymin = (box[3 * negDir.y + 1] - rayOrig.y) * invDir.y;
  float tymax = (box[3 * (1 - negDir.y) + 1] - rayOrig.y) * invDir.y;

  if (tmin > tymax || tymin > tmax) {
    return false;
  }
  if (tymin > tmin) {
    tmin = tymin;
  }
  if (tymax < tmax) {
    tmax = tymax;
  }

  // Check Z intersection
  float tzmin = (box[3 * negDir.z + 2] - rayOrig.z) * invDir.z;
  float tzmax = (box[3 * (1 - negDir.z) + 2] - rayOrig.z) * invDir.z;
  if (tmin > tzmax || tzmin > tmax) {
    return false;
  }
  if (tzmin > tmin) {
    tmin = tzmin;
  }
  if (tzmax < tmax) {
    tmax = tzmax;
  }

  // A valid hit, just keep tmin >= 0
  if (tmin < 0) {
    tmin = 0;
  }
  interval.lower = tmin;
  interval.upper = tmax;
  return true;
}

