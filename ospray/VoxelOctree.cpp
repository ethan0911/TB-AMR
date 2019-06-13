
#include <iostream>
#include <stdexcept>
#include "ospcommon/box.h"
#include "ospcommon/vec.h"

#include "VoxelOctree.h"
#include "../apps/Utils.h"

VoxelOctree::VoxelOctree(std::vector<voxel> &voxels,
                         box3f actualBounds,
                         vec3f gridWorldSpace)
{
  _actualBounds        = actualBounds;
  _virtualBounds       = actualBounds;
  _virtualBounds.upper = vec3f(max(
      max(roundToPow2(actualBounds.upper.x), roundToPow2(actualBounds.upper.y)),
      roundToPow2(actualBounds.upper.z)));

  _gridWorldSpace = gridWorldSpace;

  PRINT(_actualBounds);
  PRINT(_virtualBounds);
  PRINT(_gridWorldSpace);

  _octreeNodes.push_back(VoxelOctreeNode());  // root
  buildOctree(0, _virtualBounds, voxels);
  _octreeNodes[0].childDescripteOrValue |= 0x100;

  PRINT(_octreeNodes.size());
}

VoxelOctree::VoxelOctree(const voxel *voxels,
                         const size_t voxelNum,
                         box3f actualBounds,
                         vec3f gridWorldSpace)
{
  _actualBounds        = actualBounds;
  _virtualBounds       = actualBounds;
  _virtualBounds.upper = vec3f(max(
      max(roundToPow2(actualBounds.upper.x), roundToPow2(actualBounds.upper.y)),
      roundToPow2(actualBounds.upper.z)));

  _gridWorldSpace = gridWorldSpace;
  _voxels = voxels;
  vNum = voxelNum;
  // PRINT(_actualBounds);
  // PRINT(_virtualBounds);
  // PRINT(_gridWorldSpace);

  time_point t1 = Time();
  std::cout << green << "Building voxelOctree..." << "\n";
  _octreeNodes.push_back(VoxelOctreeNode());  // root
  // buildOctree(0, _virtualBounds, voxels, voxelNum);
  buildOctree(0, _virtualBounds, NULL, vNum);
  _octreeNodes[0].childDescripteOrValue |= 0x100;
  _voxels = NULL;

  double buildTime = Time(t1);
  std::cout <<"Building time: " << buildTime <<" s" << reset<<"\n";
}

void VoxelOctree::printOctree()
{
  printf("Octree Node Number: %ld\n", _octreeNodes.size());

  for (size_t i = 0; i < _octreeNodes.size(); i++) {
    if (_octreeNodes[i].isLeaf) {
      printf("Leaf Node: %ld, value:%lf\n", i, _octreeNodes[i].getValue());
    } else {
      printf("Inner Node: %ld, childoffset: %u, childMask: %u, childNum: %i\n",
             i,
             _octreeNodes[i].getChildOffset(),
             _octreeNodes[i].getChildMask(),
             _octreeNodes[i].getChildNum());
    }
  }
}

void VoxelOctree::printOctreeNode(const size_t nodeID)
{
  if (_octreeNodes[nodeID].isLeaf) {
    printf(
        "Leaf Node: %ld, value:%lf\n", nodeID, _octreeNodes[nodeID].getValue());
  } else {
    printf("Inner Node: %ld, childoffset: %u, childMask: %u, childNum: %i\n",
           nodeID,
           _octreeNodes[nodeID].getChildOffset(),
           _octreeNodes[nodeID].getChildMask(),
           _octreeNodes[nodeID].getChildNum());
  }
}

double VoxelOctree::queryData(vec3f pos)
{
  if (!_actualBounds.contains(pos)) {
    // printf("Current point is beyond the octree bounding box!\n");
    return 0.0;
  }

  uint64_t parent       = 0;
  VoxelOctreeNode _node = _octreeNodes[parent];
  vec3f lowerC(0.0);
  float width = _virtualBounds.size().x;

  while (!_node.isLeaf) {
    vec3f center = lowerC + vec3f(width * 0.5);

    uint8_t octantMask = 0;
    if (pos.x >= center.x)
      octantMask |= 1;
    if (pos.y >= center.y)
      octantMask |= 2;
    if (pos.z >= center.z)
      octantMask |= 4;

    uint8_t childMask    = _node.getChildMask();
    uint64_t childOffset = _node.getChildOffset();

    bool hasChild = childMask & (1 << octantMask);
    // no leaf, return invalid value 0
    if (!hasChild)
      return 0.0;

    uint8_t rightSibling = (1 << octantMask) - 1;

    // Compute the child index of the current child
    uint8_t childIndex = CHILD_BIT_COUNT[childMask & rightSibling];

    parent += childOffset + childIndex;

    if (parent >= _octreeNodes.size())
      break;

    _node = _octreeNodes[parent];

    lowerC += vec3f((octantMask & 1) ? width * 0.5 : 0.0,
                    (octantMask & 2) ? width * 0.5 : 0.0,
                    (octantMask & 4) ? width * 0.5 : 0.0);

    width *= 0.5;
  }

  return _node.getValue();
}

size_t VoxelOctree::buildOctree(size_t nodeID,
                                const box3f &bounds,
                                std::vector<voxel> &voxels)
{
  if (voxels.empty())
    return 0;

  box3f subBounds[8];

  for (int i = 0; i < 8; i++) {
    subBounds[i].lower = vec3f((i & 1) ? bounds.center().x : bounds.lower.x,
                               (i & 2) ? bounds.center().y : bounds.lower.y,
                               (i & 4) ? bounds.center().z : bounds.lower.z);
    subBounds[i].upper = subBounds[i].lower + 0.5 * bounds.size().x;
  }

  vec3f center = bounds.center() * _gridWorldSpace;
  // float cellWidth = (bounds.size() * _gridSpacing).x;

  std::vector<voxel> subVoxels[8];
  for (size_t i = 0; i < voxels.size(); i++) {
    vec3f voxelCenter = voxels[i].lower + 0.5 * voxels[i].width;
    int childID       = 0;
    childID |= voxelCenter.x < center.x ? 0 : 1;
    childID |= voxelCenter.y < center.y ? 0 : 2;
    childID |= voxelCenter.z < center.z ? 0 : 4;
    subVoxels[childID].push_back(voxels[i]);
  }

  size_t childOffset = _octreeNodes.size() - nodeID;

  int childCount = 0;
  int childIndice[8];
  uint32_t childMask = 0;
  for (int i = 0; i < 8; i++) {
    if (subVoxels[i].size() != 0) {
      childMask |= 256 >> (8 - i);
      childIndice[childCount++] = i;
    }
  }

  // push children node into the buffer, initialize later.
  for (int i = 0; i < childCount; i++) {
    _octreeNodes.push_back(VoxelOctreeNode());
  }

  // push the grand children into the buffer
  size_t grandChildOffsets[8];
  for (int i = 0; i < childCount; i++) {
    int idx = childIndice[i];
    if (subVoxels[idx].size() > 1) {
      grandChildOffsets[i] =
          buildOctree(nodeID + childOffset + i, subBounds[idx], subVoxels[idx]);
    }
  }

  // initialize children of the current node
  for (int i = 0; i < childCount; i++) {
    int idx           = childIndice[i];
    size_t childIndex = nodeID + childOffset + i;
    if (subVoxels[idx].size() == 1) {
      _octreeNodes[childIndex].isLeaf = 1;
      _octreeNodes[childIndex].childDescripteOrValue =
          doulbeBitsToUint((double)subVoxels[idx][0].value);
    } else {
      size_t offset = grandChildOffsets[i];
      _octreeNodes[childIndex].childDescripteOrValue |= offset << 8;
    }
  }

  if (voxels.size() > 1)
    _octreeNodes[nodeID].childDescripteOrValue |= childMask;

  return childOffset;
}

// size_t VoxelOctree::buildOctree(size_t nodeID,
//                                 const box3f &bounds,
//                                 const voxel *voxels,
//                                 const size_t voxelNum)
// {
//   if (voxelNum == 0)
//     return 0;

//   box3f subBounds[8];

//   for (int i = 0; i < 8; i++) {
//     subBounds[i].lower = vec3f((i & 1) ? bounds.center().x : bounds.lower.x,
//                                (i & 2) ? bounds.center().y : bounds.lower.y,
//                                (i & 4) ? bounds.center().z : bounds.lower.z);
//     subBounds[i].upper = subBounds[i].lower + 0.5 * bounds.size().x;
//   }

//   vec3f center = bounds.center() * _gridWorldSpace;

//   std::vector<voxel> subVoxels[8];
//   for (size_t i = 0; i < voxelNum; i++) {
//     vec3f voxelCenter = voxels[i].lower + 0.5 * voxels[i].width;
//     int childID       = 0;
//     childID |= voxelCenter.x < center.x ? 0 : 1;
//     childID |= voxelCenter.y < center.y ? 0 : 2;
//     childID |= voxelCenter.z < center.z ? 0 : 4;
//     subVoxels[childID].push_back(voxels[i]);
//   }

//   size_t childOffset = _octreeNodes.size() - nodeID;

//   int childCount = 0;
//   int childIndice[8];
//   uint32_t childMask = 0;
//   for (int i = 0; i < 8; i++) {
//     if (subVoxels[i].size() != 0) {
//       childMask |= 256 >> (8 - i);
//       childIndice[childCount++] = i;
//     }
//   }

//   // push children node into the buffer, initialize later.
//   for (int i = 0; i < childCount; i++) {
//     _octreeNodes.push_back(VoxelOctreeNode());
//   }

//   // push the grand children into the buffer
//   size_t grandChildOffsets[8];
//   for (int i = 0; i < childCount; i++) {
//     int idx = childIndice[i];
//     if (subVoxels[idx].size() > 1) {
//       grandChildOffsets[i] = buildOctree(nodeID + childOffset + i,
//                                          subBounds[idx],
//                                          subVoxels[idx].data(),
//                                          subVoxels[idx].size());
//     }
//   }

//   // initialize children of the current node
//   for (int i = 0; i < childCount; i++) {
//     int idx           = childIndice[i];
//     size_t childIndex = nodeID + childOffset + i;
//     if (subVoxels[idx].size() == 1) {
//       _octreeNodes[childIndex].isLeaf = 1;
//       _octreeNodes[childIndex].childDescripteOrValue =
//           doulbeBitsToUint((double)subVoxels[idx][0].value);
//     } else {
//       size_t offset = grandChildOffsets[i];
//       _octreeNodes[childIndex].childDescripteOrValue |= offset << 8;
//     }
//   }

//   if (voxelNum > 1)
//     _octreeNodes[nodeID].childDescripteOrValue |= childMask;

//   return childOffset;
// }

size_t VoxelOctree::buildOctree(size_t nodeID,
                                const box3f &bounds,
                                const size_t *voxelIDs,
                                const size_t voxelNum)
{
  if (voxelNum == 0)
    return 0;

  box3f subBounds[8];

  for (int i = 0; i < 8; i++) {
    subBounds[i].lower = vec3f((i & 1) ? bounds.center().x : bounds.lower.x,
                               (i & 2) ? bounds.center().y : bounds.lower.y,
                               (i & 4) ? bounds.center().z : bounds.lower.z);
    subBounds[i].upper = subBounds[i].lower + 0.5 * bounds.size().x;
  }

  vec3f center = bounds.center() * _gridWorldSpace;

//  std::atomic<size_t> subVoxelNum[8];
//  std::atomic<size_t> indices[8];
//  for (int i = 0; i < 8; i++) {
//    subVoxelNum[i] = 0;
//    indices[i]     = 0;
//  }
//
//  ospcommon::tasking::parallel_for(voxelNum, [&](size_t id) {
//    size_t cVoxelID = (nodeID == 0) ? id : voxelIDs[id];
//    vec3f voxelCenter =
//        this->_voxels[cVoxelID].lower + 0.5 * this->_voxels[cVoxelID].width;
//    uint8_t childID = 0;
//    childID |= voxelCenter.x < center.x ? 0 : 1;
//    childID |= voxelCenter.y < center.y ? 0 : 2;
//    childID |= voxelCenter.z < center.z ? 0 : 4;
//    subVoxelNum[childID]++;
//  });
//
//  
//  std::vector<size_t> subVoxelIDs[8];
//  for (int i = 0; i < 8; i++) {
//    subVoxelIDs[i].resize(subVoxelNum[i]);
//  }
//
//  ospcommon::tasking::parallel_for(voxelNum, [&](size_t id) {
//    size_t cVoxelID = (nodeID == 0) ? id : voxelIDs[id];
//    vec3f voxelCenter =
//        this->_voxels[cVoxelID].lower + 0.5 * this->_voxels[cVoxelID].width;
//    uint8_t childID = 0;
//    childID |= voxelCenter.x < center.x ? 0 : 1;
//    childID |= voxelCenter.y < center.y ? 0 : 2;
//    childID |= voxelCenter.z < center.z ? 0 : 4;
//    size_t idx = indices[childID]++;
//    subVoxelIDs[childID][idx] = cVoxelID;
//  });


   std::vector<size_t> subVoxelIDs[8];
   for (size_t i = 0; i < voxelNum; i++) {
     size_t cVoxelID = (nodeID == 0) ? i : voxelIDs[i];
     vec3f voxelCenter = this->_voxels[cVoxelID].lower + 0.5 * this->_voxels[cVoxelID].width;
     uint8_t childID       = 0;
     childID |= voxelCenter.x < center.x ? 0 : 1;
     childID |= voxelCenter.y < center.y ? 0 : 2;
     childID |= voxelCenter.z < center.z ? 0 : 4;
     subVoxelIDs[childID].push_back(cVoxelID);
   }

  size_t childOffset = _octreeNodes.size() - nodeID;

  int childCount = 0;
  int childIndice[8];
  uint32_t childMask = 0;
  for (int i = 0; i < 8; i++) {
    if (subVoxelIDs[i].size() != 0) {
      childMask |= 256 >> (8 - i);
      childIndice[childCount++] = i;
    }
  }

  // push children node into the buffer, initialize later.
  for (int i = 0; i < childCount; i++) {
    _octreeNodes.push_back(VoxelOctreeNode());
  }

  // push the grand children into the buffer
  size_t grandChildOffsets[8];
  for (int i = 0; i < childCount; i++) {
    int idx = childIndice[i];
    if (subVoxelIDs[idx].size() > 1) {
      grandChildOffsets[i] = buildOctree(nodeID + childOffset + i,
                                         subBounds[idx],
                                         subVoxelIDs[idx].data(),
                                         subVoxelIDs[idx].size());
    }
  }

  // initialize children of the current node
  for (int i = 0; i < childCount; i++) {
    int idx           = childIndice[i];
    size_t childIndex = nodeID + childOffset + i;
    if (subVoxelIDs[idx].size() == 1) {
      _octreeNodes[childIndex].isLeaf = 1;
      _octreeNodes[childIndex].childDescripteOrValue =
          doulbeBitsToUint((double)this->_voxels[subVoxelIDs[idx][0]].value);
    } else {
      size_t offset = grandChildOffsets[i];
      _octreeNodes[childIndex].childDescripteOrValue |= offset << 8;
    }
  }

  if (voxelNum > 1)
    _octreeNodes[nodeID].childDescripteOrValue |= childMask;

  return childOffset;
}
