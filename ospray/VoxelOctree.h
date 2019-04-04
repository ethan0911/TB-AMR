
#ifndef VOXELOCTREE_H_
#define VOXELOCTREE_H_

#include <stdint.h>
#include <stdio.h>

#include "ospray/ospray.h"
#include "ospray/common/OSPCommon.h"

#include "ospcommon/vec.h"
#include <vector>

using namespace std;
using namespace ospcommon;


static inline double uintBitsToDouble(uint64_t i) {
  union { uint64_t i; double f; } unionHack;
  unionHack.i = i;
  return unionHack.f;
}

static inline uint64_t doulbeBitsToUint(double f) {
  union { uint64_t i; double f; } unionHack;
  unionHack.f = f;
  return unionHack.i;
}


static inline int roundToPow2(int x) {
    int y;
    for (y = 1; y < x; y *= 2);
    return y;
}

static inline float roundToPow2(float x) {
    float y;
    for (y = 1.0; y < x; y *= 2);
    return y;
}

static const uint32_t CHILD_BIT_COUNT[] = {
  0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
  4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};


struct voxel{
    vec3f lower;
    float width;

    float value;

    voxel(vec3f c, float w, float v){
        lower = c;
        width = w;
        value = v;
    }
};


struct VoxelOctreeNode
{
  /* data */
  uint8_t isLeaf; 

  // Store the child desctiption [childOffset|childMask] if it's a inner node
  // Store the value of the node if it's a leaf node. value must be larger than 0 ? 
  uint64_t childDescripteOrValue;  


  uint8_t getChildMask(){ return childDescripteOrValue & 0xFF; }
  uint64_t getChildOffset(){ return childDescripteOrValue >> 8; }

  double getValue() { return uintBitsToDouble(childDescripteOrValue); }

  uint32_t getChildNum() { return CHILD_BIT_COUNT[getChildMask()]; }

  // center of the current node, used for calculate sub-space
  // vec3f _center;
  // float cellWidth;
};


class VoxelOctree{
public:
  VoxelOctree();
  VoxelOctree(std::vector<voxel> voxels, box3f bounds);

  void printOctree();


  double queryData(vec3f pos);



  box3f _bounds;
  //! extend the dimension to pow of 2 to build the octree e.g. 4 x 4 x 4
  box3f _virtualBounds;

  std::vector<VoxelOctreeNode> _octreeNodes;
  
private:
  size_t buildOctree(size_t nodeID,const box3f& bounds, std::vector<voxel> voxels);

};

#endif
