
#include <iostream>
#include <stdexcept>
#include "ospcommon/box.h"
#include "ospcommon/vec.h"

#include "VoxelOctree.h"
#include "../apps/Utils.h"
#include "tbb/tbb.h"

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
  _octreeNodes[0].childDescripteOrValue = 0;
  buildOctree(0, _virtualBounds, voxels);
  _octreeNodes[0].childDescripteOrValue |= 0x100;

  PRINT(_octreeNodes.size());
}

VoxelOctree::VoxelOctree(const voxel *voxels,
                         const size_t voxelNum,
                         range1f voxelRange,
                         box3f actualBounds,
                         vec3f gridWorldSpace,
                         vec3f worldOrigin)
{
  _actualBounds        = actualBounds;
  _virtualBounds       = actualBounds;
  _virtualBounds.upper = vec3f(max(
      max(roundToPow2(actualBounds.upper.x), roundToPow2(actualBounds.upper.y)),
      roundToPow2(actualBounds.upper.z)));

  _gridWorldSpace = gridWorldSpace;
  _worldOrigin = worldOrigin;
  _voxels = voxels;
  vNum = voxelNum;

  time_point t1 = Time();
  std::cout << green << "Building voxelOctree..." << "\n";
  VoxelOctreeNode root;
  root.vRange = voxelRange;
  _octreeNodes.push_back(root);  // root
  _octreeNodes[0].childDescripteOrValue = 0;
  buildOctree(0, _virtualBounds, NULL, vNum);
  _octreeNodes[0].childDescripteOrValue |= 0x100;
  _voxels = NULL;

  printOctreeNode(0);

  double buildTime = Time(t1);
  std::cout <<"Building time: " << buildTime <<" s" << reset<<"\n";
}

void VoxelOctree::printOctree()
{
  printf("Octree Node Number: %ld\n", _octreeNodes.size());

  for (size_t i = 0; i < _octreeNodes.size(); i++) {
    if (_octreeNodes[i].isLeaf) {
      printf("Leaf Node: %ld, value:%lf, vRange:[%f,%f]\n",
             i,
             _octreeNodes[i].getValue(),
             _octreeNodes[i].vRange.lower,
             _octreeNodes[i].vRange.upper);
    } else {
      printf(
          "Inner Node: %ld, childoffset: %u, childMask: %u, childNum: %i, "
          "vRange:[%f,%f]\n",
          i,
          _octreeNodes[i].getChildOffset(),
          _octreeNodes[i].getChildMask(),
          _octreeNodes[i].getChildNum(),
          _octreeNodes[i].vRange.lower,
          _octreeNodes[i].vRange.upper);
    }
  }
}

void VoxelOctree::saveOctree(const std::string &fileName)
{
  std::string octFile = fileName + ".oct";
  const std::string binFileName = octFile + "bin";
  FILE *bin = fopen(binFileName.c_str(),"wb");

  if(!fwrite(_octreeNodes.data(), sizeof(VoxelOctreeNode), _octreeNodes.size(), bin))
    throw std::runtime_error("Could not write ... ");
  
  fclose(bin);

  FILE *oct = fopen(octFile.c_str(), "w");
  fprintf(oct, "<?xml?>\n");
  fprintf(oct, "<ospray>\n");
  {
    fprintf(oct, "  <Octree\n");
    {
      fprintf(oct, "    nodeSize=\"%li\"\n", _octreeNodes.size());
      fprintf(oct, "    actualBound=\"%f %f %f %f %f %f\"\n",
              _actualBounds.lower.x,_actualBounds.lower.y,_actualBounds.lower.z,
              _actualBounds.upper.x,_actualBounds.upper.y,_actualBounds.upper.z);
      fprintf(oct, "    virtualBound=\"%f %f %f %f %f %f\"\n",
              _virtualBounds.lower.x,_virtualBounds.lower.y,_virtualBounds.lower.z,
              _virtualBounds.upper.x,_virtualBounds.upper.y,_virtualBounds.upper.z);
      fprintf(oct, "    gridWidthInWorld=\"%f\"\n", _gridWorldSpace.x);
      fprintf(oct,"    >\n");
    }
    fprintf(oct, "  </Octree>\n");
  }
  fprintf(oct, "</ospray>\n");
  fclose(oct);

  std::cout<<"Save octree into " << octFile << std::endl;
}

void VoxelOctree::mapOctreeFromFile(const std::string &fileName)
{
  std::shared_ptr<xml::XMLDoc> doc = xml::readXML(fileName.c_str());
  if (!doc)
    throw std::runtime_error("could not read octree .oct file:" + fileName);
  std::shared_ptr<xml::Node> osprayNode = std::make_shared<xml::Node>(doc->child[0]);
  assert(osprayNode->name == "ospray");

  std::shared_ptr<xml::Node> octTreeNode = std::make_shared<xml::Node>(osprayNode->child[0]);
  assert(octTreeNode->name == "Octree");

  size_t nodeSize = std::stoll(octTreeNode->getProp("nodeSize"));
  this->_octreeNodes.clear();
  this->_octreeNodes.resize(nodeSize);

  sscanf(octTreeNode->getProp("actualBound").c_str(),"%f %f %f %f %f %f",
        &_actualBounds.lower.x, &_actualBounds.lower.y, &_actualBounds.lower.z,
        &_actualBounds.upper.x, &_actualBounds.upper.y, &_actualBounds.upper.z);

  sscanf(octTreeNode->getProp("virtualBound").c_str(),"%f %f %f %f %f %f",
        &_virtualBounds.lower.x, &_virtualBounds.lower.y, &_virtualBounds.lower.z,
        &_virtualBounds.upper.x, &_virtualBounds.upper.y, &_virtualBounds.upper.z);

  float gridWidthInWorld = std::stof(octTreeNode->getProp("gridWidthInWorld"));
  _gridWorldSpace = vec3f(gridWidthInWorld);

  std::string binFileName = fileName + "bin";
  FILE *file              = fopen(binFileName.c_str(), "rb");
  if (!file)
    throw std::runtime_error("could not open octree bin file " + binFileName);

  fread(this->_octreeNodes.data(), sizeof(VoxelOctreeNode), nodeSize, file);
  fclose(file);
}

void VoxelOctree::printOctreeNode(const size_t nodeID)
{
  if (_octreeNodes[nodeID].isLeaf) {
    printf("Leaf Node: %ld, value:%lf, vRange:[%f,%f]\n",
           nodeID,
           _octreeNodes[nodeID].getValue(),
           _octreeNodes[nodeID].vRange.lower,
           _octreeNodes[nodeID].vRange.upper);
  } else {
    printf(
        "Inner Node: %ld, childoffset: %u, childMask: %u, childNum: %i, "
        "vRange:[%f,%f]\n",
        nodeID,
        _octreeNodes[nodeID].getChildOffset(),
        _octreeNodes[nodeID].getChildMask(),
        _octreeNodes[nodeID].getChildNum(),
        _octreeNodes[nodeID].vRange.lower,
        _octreeNodes[nodeID].vRange.upper);
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

  std::vector<size_t> subVoxelIDs[8];
  range1f subVoxelRange[8];

  for (size_t i = 0; i < voxelNum; i++) {
    size_t cVoxelID   = (nodeID == 0) ? i : voxelIDs[i];
    vec3f voxelCenter = this->_voxels[cVoxelID].lower - this->_worldOrigin +
                        0.5 * this->_voxels[cVoxelID].width;
    uint8_t childID = 0;
    childID |= voxelCenter.x < center.x ? 0 : 1;
    childID |= voxelCenter.y < center.y ? 0 : 2;
    childID |= voxelCenter.z < center.z ? 0 : 4;
    subVoxelIDs[childID].push_back(cVoxelID);
    subVoxelRange[childID].extend(this->_voxels[cVoxelID].value);
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
    _octreeNodes[childIndex].vRange =  subVoxelRange[idx];
  }


  if (voxelNum > 1)
    _octreeNodes[nodeID].childDescripteOrValue |= childMask;
  
  return childOffset;
}
