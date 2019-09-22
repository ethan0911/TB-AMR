
#ifndef VOXELOCTREE_H_
#define VOXELOCTREE_H_

#include <cstdint>
#include <cstdio>
#include <mutex>

#include "ospray/ospray.h"
#include "ospray/common/OSPCommon.h"
#include "ospcommon/tasking/parallel_for.h"
#include "ospcommon/xml/XML.h"

#include "ospcommon/math/vec.h"
#include "ospcommon/math/range.h"
#include <vector>



using namespace std;
using namespace ospcommon;
using namespace ospcommon::math;


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

    voxel(){
      lower = vec3f(0.0);
      width = 0.f;
      value = 0.f;
    }

    voxel(vec3f c, float w, float v){
        lower = c;
        width = w;
        value = v;
    }
};

struct VoxelOctreeNode
{
  // Store the value range of current node, used for fast isosurface generation
  range1f vRange;
  /* data */
  uint8_t isLeaf; 

  // Store the child desctiption [childOffset|childMask] if it's a inner node
  // Store the value of the node if it's a leaf node. value must be larger than 0 ? 
  uint64_t childDescripteOrValue;  


  // VoxelOctreeNode(){
  //   vRange = range1f(1e20, -1e20);
  // }

  // VoxelOctreeNode(range1f voxelRange)
  // {
  //   vRange = voxelRange;
  // }

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
 VoxelOctree(){};
 VoxelOctree(std::vector<voxel> &voxels,
             box3f actualBounds,
             vec3f gridWorldSpace);
 VoxelOctree(const voxel *voxels,
             const size_t voxelNum,
             range1f voxelRange,
             box3f actualBounds,
             vec3f gridWorldSpace,
             vec3f worldOrigin);

 void printOctree();
 void printOctreeNode(const size_t nodeID);
 void saveOctree(const std::string &fileName);
 void mapOctreeFromFile(const std::string &fileName);

 double queryData(vec3f pos);

 box3f _actualBounds;
 //! extend the dimension to pow of 2 to build the octree e.g. 4 x 4 x 4
 box3f _virtualBounds;

 vec3f _gridWorldSpace;

 vec3f _worldOrigin;

 std::vector<VoxelOctreeNode> _octreeNodes;

private:
  const voxel *_voxels;
  size_t vNum;
  std::mutex lock;
  
private:
  size_t buildOctree(size_t nodeID,const box3f& bounds, std::vector<voxel> &voxels);
  // size_t buildOctree(size_t nodeID,const box3f& bounds, const voxel* voxels, const size_t voxelNum);
  size_t buildOctree(size_t nodeID,const box3f& bounds, const size_t* voxelIDs, const size_t voxelNum);

};

#endif
