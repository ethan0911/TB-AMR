
#include <iostream>
#include <stdexcept>
#include "ospcommon/vec.h"
#include "ospcommon/box.h"

#include "VoxelOctree.h"



VoxelOctree::VoxelOctree(const std::vector<voxel>& voxels, vec3f dimension){
  _dimension = dimension ;
  _virtualBounds = box3f(vec3f(0.0),
                        vec3f(std::max(std::max(roundToPow2(_dimension.x), roundToPow2(_dimension.y)),
                        roundToPow2(_dimension.z))));
  _octreeNodes.push_back(VoxelOctreeNode()); //root 
  buildOctree(0,_virtualBounds,voxels);
  _octreeNodes[0].childDescripteOrValue |= 0x100;
}

//VoxelOctree::VoxelOctree(std::vector<voxel> voxels) : VoxelOctree(voxels, vec3f(3.0,2.0,2.0)){

  //hard code the dimension, should be set by the user or code later
  /*
   *_dimension = vec3f(3.0,4.0,2.0);
   *_virtualBounds = box3f(vec3f(0.0),
   *                      vec3f(std::max(std::max(roundToPow2(_dimension.x), roundToPow2(_dimension.y)),
   *                      roundToPow2(_dimension.z))));
   *_octreeNodes.push_back(VoxelOctreeNode()); //root 
   *buildOctree(0,_virtualBounds,voxels);
   *_octreeNodes[0].childDescripteOrValue |= 0x100;
   */
  
//}


void VoxelOctree::printOctree(){
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

  double VoxelOctree::queryData(vec3f pos)
  {
      if(pos.x > _dimension.x || pos.y > _dimension.y || pos.z > _dimension.z){
        // printf("Current point is beyond the octree bounding box!\n");
        return 0.0;
      }

      uint64_t parent = 0;
      VoxelOctreeNode _node = _octreeNodes[parent];
      vec3f lowerC(0.0);
      float width = _virtualBounds.size().x;

      while(!_node.isLeaf){
        vec3f center = lowerC + vec3f(width * 0.5);


        uint8_t octantMask = 0; 
        if(pos.x >= center.x) octantMask |= 1;
        if(pos.y >= center.y) octantMask |= 2;
        if(pos.z >= center.z) octantMask |= 4;

        uint8_t childMask = _node.getChildMask();
        uint64_t childOffset = _node.getChildOffset();

        uint8_t rightSibling = pow(2,octantMask) - 1;

        // Compute the child index of the current child
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


  size_t VoxelOctree::buildOctree(size_t nodeID,const box3f& bounds, std::vector<voxel> voxels)
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
