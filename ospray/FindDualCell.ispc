#include "VoxelOctree.ih"
#include "FindDualCell.ih"

struct VODualStack
{
  varying bool active;

  varying unsigned int8 queryPointMask;
  varying unsigned int64 pNodeIdx;
  varying vec3f pos;
  uniform float width;
};

struct SubspaceSpliter
{
  //! check if any of the 8 conners located in a subspcae. 
  //  if none point locate in this subspace, the value is initialized to 128
  //  else the value is set to the child space ID (0 to 7)
  unsigned int8 subSpaceID;  
  //! the point bitmask indicate which corner locates in this space
  //  e.g. 10000010 (the second and 7th conner are in this space)
  unsigned int8 pointIndicator;
};


inline uniform VODualStack* uniform pushStack(uniform VODualStack* uniform stackPtr,
                                              varying unsigned int8 pointMask,
                                              varying unsigned int64 pNodeIdx,
                                              varying vec3f pos,
                                              uniform float width)
{
  unmasked{
    stackPtr->active = false;
    stackPtr->queryPointMask = pointMask;
    stackPtr->pNodeIdx = pNodeIdx;
    stackPtr->pos = pos;
  } 

  stackPtr->active = true;
  stackPtr->width = width;

  return stackPtr + 1;
}


#define STACK_SIZE 128

void findDualCell(const uniform VoxelOctree & _voxelAccel, DualCell & dCell)
{
  const vec3f _P0 = clamp(dCell.pos, make_vec3f(0.f), _voxelAccel._bounds.upper);
  const vec3f _P1 = clamp(dCell.pos + dCell.width, make_vec3f(0.f), _voxelAccel._bounds.upper- make_vec3f(0.000001f));

  const varying float *const uniform p0 = &_P0.x;
  const varying float *const uniform p1 = &_P1.x;

  const varying float *const uniform lo = p0;
  const varying float *const uniform hi = p1;

  // vec3f conners[8] = {_P0,make_vec3f(_P1.x,_P0.y,_P0.z),
  //                     make_vec3f(_P0.x,_P1.y,_P0.z),make_vec3f(_P1.x,_P1.y,_P0.z),
  //                     make_vec3f(_P0.x,_P0.y,_P1.z),make_vec3f(_P1.x,_P0.y,_P1.z),
  //                     make_vec3f(_P0.x,_P1.y,_P1.z),_P1};

  vec3f conners[8] = {make_vec3f(lo[0],lo[1],lo[2]),make_vec3f(hi[0],lo[1],lo[2]),
                      make_vec3f(lo[0],hi[1],lo[2]),make_vec3f(hi[0],hi[1],lo[2]),
                      make_vec3f(lo[0],lo[1],hi[2]),make_vec3f(hi[0],lo[1],hi[2]),
                      make_vec3f(lo[0],hi[1],hi[2]),make_vec3f(hi[0],hi[1],hi[2])};

  

  // initialize the dual cell's value
  for(uniform i = 0 ; i < 8; i++)
    dCell.value[i] = -1.0f;
  
  vec3f gridOrigin = _voxelAccel._virtualBounds.lower;
  uniform vec3f boundSize = box_size(_voxelAccel._virtualBounds);
  uniform float width = boundSize.x;
  
  uniform VODualStack stack[STACK_SIZE];
  uniform VODualStack *uniform stackPtr = pushStack(&stack[0],0xFF,0,gridOrigin,width);

  while(stackPtr > stack)
  {
    --stackPtr;
    if(stackPtr->active){
      const unsigned int64 nodeID = stackPtr->pNodeIdx;
      if(nodeID >= _voxelAccel._oNodeNum)
        break;

      const unsigned int8 queryPointMask = stackPtr->queryPointMask;
      const vec3f pos = stackPtr->pos;
      const uniform float cellWidth = stackPtr->width;

      const VoxelOctreeNode node = _voxelAccel._octreeNodes[nodeID];
      if(isLeaf(node)){
        for(uniform int i = 0; i < 8; i++){
          unsigned int bitmask = queryPointMask & (1 << i);
          if(bitmask){
            dCell.value[i] = (float)getValue(node);
            dCell.actualWidth[i] = cellWidth;
            dCell.isLeaf[i] = (dCell.width == cellWidth);
          }
        }
      }else{
        vec3f center= pos + make_vec3f(cellWidth * 0.5f);

        SubspaceSpliter spliter[8] = {{128,0},{128,0},{128,0},{128,0},{128,0},{128,0},{128,0},{128,0}};
        for(uniform int i = 0; i < 8; i++){
          unsigned int bitmask = queryPointMask & (1 << i);
          if(bitmask){
            unsigned int8 oMask = 0;
            oMask |= (conners[i].x >= center.x) ? 1 : 0;
            oMask |= (conners[i].y >= center.y) ? 2 : 0;
            oMask |= (conners[i].z >= center.z) ? 4 : 0;
            spliter[oMask].subSpaceID = oMask;
            spliter[oMask].pointIndicator |= (1 << i);
          }
        }

        unsigned int8 childMask = getChildMask(node);
        unsigned int64 childOffset = getChildOffset(node);

        for(uniform int i = 0; i < 8; i++){
          if(spliter[i].subSpaceID != 128){
            unsigned int8 octantMask = spliter[i].subSpaceID;
            bool hasChild = childMask & (1 << octantMask);
            // no leaf, return invalid value 0
            if(!hasChild)
            {
              for(uniform int i = 0; i < 8; i++){
                unsigned int bitmask = spliter[i].pointIndicator & (1 << i);
                if(bitmask){
                  dCell.value[i] = -1.0;
                  dCell.actualWidth[i] = cellWidth;
                  dCell.isLeaf[i] = (dCell.width == cellWidth);
                }
              }
            }else{
              unsigned int8 rightSibling = (1 << octantMask) - 1;
              unsigned int8 childIndex = BIT_COUNT[childMask & rightSibling];
              unsigned int64 childNodeID = nodeID + childOffset + childIndex;

              vec3f lowerPos = pos + make_vec3f((octantMask & 1) ? cellWidth * 0.5 : 0.0,
                                          (octantMask & 2) ? cellWidth * 0.5 : 0.0,
                                          (octantMask & 4) ? cellWidth * 0.5 : 0.0);
              stackPtr = pushStack(stackPtr,spliter[i].pointIndicator,childNodeID,lowerPos,cellWidth * 0.5f);
              assert(stackPtr - stack < STACK_SIZE);
            }
          }
        }

      }
    }
  }
}



void findMirroredDualCell(const uniform VoxelOctree & _voxelAccel, const vec3i &mirror, DualCell & dCell)
{
  const vec3f _P0 = clamp(dCell.pos, make_vec3f(0.f), _voxelAccel._bounds.upper);
  const vec3f _P1 = clamp(dCell.pos + dCell.width, make_vec3f(0.f), _voxelAccel._bounds.upper - make_vec3f(0.000001f));

  // print("_P0:[%,%,%]\n",_P0.x,_P0.y,_P0.z);
  // print("_p1:[%,%,%]\n",_P1.x,_P1.y,_P1.z);

  const varying float *const uniform p0 = &_P0.x;
  const varying float *const uniform p1 = &_P1.x;

  const float lo[3] = { mirror.x?_P1.x:_P0.x, mirror.y?_P1.y:_P0.y, mirror.z?_P1.z:_P0.z };
  const float hi[3] = { mirror.x?_P0.x:_P1.x, mirror.y?_P0.y:_P1.y, mirror.z?_P0.z:_P1.z };

  vec3f conners[8] = {make_vec3f(lo[0],lo[1],lo[2]),make_vec3f(hi[0],lo[1],lo[2]),
                      make_vec3f(lo[0],hi[1],lo[2]),make_vec3f(hi[0],hi[1],lo[2]),
                      make_vec3f(lo[0],lo[1],hi[2]),make_vec3f(hi[0],lo[1],hi[2]),
                      make_vec3f(lo[0],hi[1],hi[2]),make_vec3f(hi[0],hi[1],hi[2])};
                 


  // initialize the dual cell's value
  for(uniform i = 0 ; i < 8; i++)
    dCell.value[i] = -1.0f;
  
  vec3f gridOrigin = _voxelAccel._virtualBounds.lower;
  uniform vec3f boundSize = box_size(_voxelAccel._virtualBounds);
  uniform float width = boundSize.x;
  
  uniform VODualStack stack[STACK_SIZE];
  uniform VODualStack *uniform stackPtr = pushStack(&stack[0],0xFF,0,gridOrigin,width);

  while(stackPtr > stack)
  {
    --stackPtr;
    if(stackPtr->active){
      const unsigned int64 nodeID = stackPtr->pNodeIdx;
      if(nodeID >= _voxelAccel._oNodeNum)
        break;

      const unsigned int8 queryPointMask = stackPtr->queryPointMask;
      const vec3f pos = stackPtr->pos;
      const uniform float cellWidth = stackPtr->width;

      const VoxelOctreeNode node = _voxelAccel._octreeNodes[nodeID];
      if(isLeaf(node)){
        for(uniform int i = 0; i < 8; i++){
          unsigned int bitmask = queryPointMask & (1 << i);
          if(bitmask){
            dCell.value[i] = (float)getValue(node);
            dCell.actualWidth[i] = cellWidth;
            dCell.isLeaf[i] = (dCell.width == cellWidth);
          }
        }
      }else{
        vec3f center= pos + make_vec3f(cellWidth * 0.5f);

        SubspaceSpliter spliter[8] = {{128,0},{128,0},{128,0},{128,0},{128,0},{128,0},{128,0},{128,0}};
        for(uniform int i = 0; i < 8; i++){
          unsigned int bitmask = queryPointMask & (1 << i);
          if(bitmask){
            unsigned int8 oMask = 0;
            oMask |= (conners[i].x >= center.x) ? 1 : 0;
            oMask |= (conners[i].y >= center.y) ? 2 : 0;
            oMask |= (conners[i].z >= center.z) ? 4 : 0;
            spliter[oMask].subSpaceID = oMask;
            spliter[oMask].pointIndicator |= (1 << i);
          }
        }

        unsigned int8 childMask = getChildMask(node);
        unsigned int64 childOffset = getChildOffset(node);

        for(uniform int i = 0; i < 8; i++){
          if(spliter[i].subSpaceID != 128){
            unsigned int8 octantMask = spliter[i].subSpaceID;
            bool hasChild = childMask & (1 << octantMask);
            // no leaf, return invalid value 0
            if(!hasChild)
            {
              for(uniform int i = 0; i < 8; i++){
                unsigned int bitmask = spliter[i].pointIndicator & (1 << i);
                if(bitmask){
                  dCell.value[i] = -1.0;
                  dCell.actualWidth[i] = cellWidth;
                  dCell.isLeaf[i] = (dCell.width == cellWidth);
                }
              }
            }else{
              unsigned int8 rightSibling = (1 << octantMask) - 1;
              unsigned int8 childIndex = BIT_COUNT[childMask & rightSibling];
              unsigned int64 childNodeID = nodeID + childOffset + childIndex;

              vec3f lowerPos = pos + make_vec3f((octantMask & 1) ? cellWidth * 0.5 : 0.0,
                                          (octantMask & 2) ? cellWidth * 0.5 : 0.0,
                                          (octantMask & 4) ? cellWidth * 0.5 : 0.0);
              stackPtr = pushStack(stackPtr,spliter[i].pointIndicator,childNodeID,lowerPos,cellWidth * 0.5f);
              assert(stackPtr - stack < STACK_SIZE);
            }
          }
        }

      }
    }
  }
}