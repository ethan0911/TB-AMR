
#ifndef VOXELOCTREE_H_
#define VOXELOCTREE_H_

#include <stdint.h>
#include <stdio.h>

#include "ospray/ospray.h"
#include "ospray/common/OSPCommon.h"

#include "ospcommon/vec.h"

using namespace std;
using namespace ospcommon;

enum OCTREE_CHILDREN{
  LEFT_LOWER_BOTTOM = 0, 
  RIGHT_LOWER_BOTTOM,
  LEFT_UPPER_BOTTOM,
  RIGHT_UPPER_BOTTOM,
  LEFT_LOWER_TOP,
  RIGHT_LOWER_TOP,
  LEFT_UPPER_TOP,
  RIGHT_UPPER_TOP
};    


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
  // Store the value of the node if it's a leaf node.
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

  VoxelOctree(std::vector<voxel> voxels){
    worldBounds = box3f(vec3f(0.0),vec3f(1.0));
    _octreeNodes.push_back(VoxelOctreeNode()); //root 
    buildOctree(0,worldBounds,voxels);
    _octreeNodes[0].childDescripteOrValue |= 0x100;
  }

  void printOctree(){
    printf("Octree Node Number: %ld\n", _octreeNodes.size());
    
    for(size_t i =0; i<_octreeNodes.size();i++){
      if(_octreeNodes[i].isLeaf){
        printf("Leaf Node: %ld, value:%lf\n", i, _octreeNodes[i].getValue());
      }else{
        printf("Inner Node: %ld, childoffset: %u, childMask: %u, childNum: %i\n", 
        i, _octreeNodes[i].getChildOffset(), _octreeNodes[i].getChildMask(), _octreeNodes[i].getChildNum());
      }
    }
      
  }

  double queryData(vec3f pos)
  {
      if(!worldBounds.contains(pos)){
        printf("Current point is beyond the octree bounding box!\n");
        return 0.0;
      }

      uint64_t parent = 0;
      VoxelOctreeNode _node = _octreeNodes[parent];
      vec3f lowerC(0.0);
      float width = worldBounds.size().x;

      while(!_node.isLeaf){
        vec3f center = lowerC + vec3f(width * 0.5);


        uint8_t octantMask = 0; 
        if(pos.x >= center.x) octantMask |= 1;
        if(pos.y >= center.y) octantMask |= 2;
        if(pos.z >= center.z) octantMask |= 4;

        uint8_t childMask = _node.getChildMask();
        uint64_t childOffset = _node.getChildOffset();

        uint8_t rightSibling = pow(2,octantMask) - 1;

        uint8_t childIndex = CHILD_BIT_COUNT[childMask & rightSibling];

        parent += childOffset + childIndex;
        _node = _octreeNodes[parent];

        lowerC += vec3f((octantMask & 1) ? width * 0.5 : 0.0,
                        (octantMask & 2) ? width * 0.5 : 0.0,
                        (octantMask & 4) ? width * 0.5 : 0.0);

        width *= 0.5;
      }

      return _node.getValue();
  }

private: 
  size_t buildOctree(size_t nodeID,const box3f& bounds, std::vector<voxel> voxels)
  {
    if(voxels.empty())
      return 0;


    vec3f center = bounds.center();
    float cellWidth = bounds.size().x;
    box3f subBounds[8];

    for(int i = 0; i < 8 ; i++){
      subBounds[i].lower = vec3f((i & 1) ? center.x : bounds.lower.x,
                                  (i & 2) ? center.y : bounds.lower.y,
                                  (i & 4) ? center.z : bounds.lower.z);
      subBounds[i].upper = subBounds[i].lower + 0.5 * cellWidth;
    }

    std::vector<voxel> subVoxels[8];
    for(size_t i = 0; i< voxels.size();i++){
      vec3f voxelCenter = voxels[i].lower + 0.5 * voxels[i].width;
      int childID = 0 ; 
      childID |= voxelCenter.x < center.x ? 0 : 1;
      childID |= voxelCenter.y < center.y ? 0 : 2;
      childID |= voxelCenter.z < center.z ? 0 : 4;
      subVoxels[childID].push_back(voxels[i]);
    }

    size_t childOffset = _octreeNodes.size() - nodeID;

    int childCount = 0;
    int childIndice[8];
    uint32_t childMask =0;
    for(int i=0; i < 8; i++){
      if(subVoxels[i].size() != 0){
        childMask |= 256 >> (8 - i);
        childIndice[childCount++] = i;
      }
    }

    // push children node into the buffer, initialize later.
    for(int i= 0 ; i < childCount; i++){
      _octreeNodes.push_back(VoxelOctreeNode());
    }

    // push the grand children into the buffer
    size_t grandChildOffsets[8];
    for(int i= 0 ; i < childCount; i++){
      int idx = childIndice[i];
      if(subVoxels[idx].size() > 1){
        grandChildOffsets[i] = buildOctree(nodeID + childOffset + i,subBounds[idx],subVoxels[idx]);
      }
    }

    //initialize children of the current node 
    for(int i = 0; i < childCount; i++){
      int idx = childIndice[i];
      size_t childIndex = nodeID + childOffset + i;
      if(subVoxels[idx].size() == 1){
        _octreeNodes[childIndex].isLeaf = 1;
        _octreeNodes[childIndex].childDescripteOrValue = doulbeBitsToUint((double)subVoxels[idx][0].value);
      }else{
        size_t offset = grandChildOffsets[i];
        _octreeNodes[childIndex].childDescripteOrValue |= offset << 8;   
      }

    }

    if(voxels.size() > 1)
      _octreeNodes[nodeID].childDescripteOrValue |= childMask;

    return childOffset;
  }

  //! world bounds of domain
  box3f worldBounds;

private:
  std::vector<VoxelOctreeNode> _octreeNodes;
};




// struct VoxelOctreeNode
// {
//     //a unique morton stytle hashcode for each node
//     uint32_t locCode;

//     vec3i   lowerLeft;
//     uint32_t     nodeWidth;
//     /* data */
//     void* pValues;
// };



// class VoxelOctree{


//     VoxelOctree(std::vector<voxel> voxels);

// public:
//     VoxelOctreeNode * GetParentNode(VoxelOctreeNode *node)
//     {
//         const uint32_t locCodeParent = node->locCode>>3;
//         return LookupNode(locCodeParent);
//     }


// private:
//     VoxelOctreeNode * LookupNode(uint32_t locCode)
//     {
//         const auto iter = _octreeNodes.find(locCode);
//         return (iter == _octreeNodes.end() ? nullptr : &(*iter));
//     }

// private:
//     umorder_map<uint32_t, VoxelOctreeNode> _octreeNodes;

// };


#endif
