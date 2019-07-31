#include "TAMRVolume.ih"
// #include "FindCell.ih"
// #include "FindDualCell.ih"

/************************************************************
 *  Octant Method. Coarser side stich : Basis method
 ***********************************************************/

/*! do octant method for point P, in (leaf) cell C.  having this in a
  separate function allows for call it recursively from neighboring
  cells if so required */
varying float doOctant(const void *uniform _self, 
                       const CellRef &C,
                       const varying vec3f &P)
{
  uniform TAMRVolume *uniform self = (uniform uniform TAMRVolume *uniform)_self;

  /* first - find the given octant, dual cell, etc */
  Octant O;
  DualCell D;
  initOctantAndDual(O,D,P,C);
  
  findMirroredDualCell(self->_voxelAccel,O.mirror,D);

  if(isDualCellInSameLevel(D))
    return lerp(D);

  for(uniform int i = 0 ; i < 8; i++)
    O.value[i] = -1.0f;

  /* initialize corner computation. for each corner we compute if we
     could fill it from the current octant/dual cell ('done'), and, if
     not, which other cell it should be filled from ('needToFillFrom') */
  bool    done[8];
  CellRef needToFillFrom[8];
  /* ###################### CENTER ###################### */
  /* the center point is ALWAYS the cell value */
  O.value[C000] = C.value;
  done[C000]    = true;
  bool coarseFilled = false;

  const float delta = 0.01f;

  /* ###################### SIDES ###################### */
  /* sides touch one neighbor. we can interpolate if it's on the same
     level, will have to defer to that neighbor if that neighbor is
     coarser, and compute our own if that neighbor is finer */

  /* ----------- side C001 ----------- */ {
    if (D.actualWidth[C001] == C.width) {
      /* same level - interpolate and done */
      O.value[C001] = 0.5f * (C.value + D.value[C001]);
      done[C001]    = true;
    } else if (isCoarser(D.actualWidth[C001],C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C001].pos.x = O.center.x + (0.5f + delta) * C.width * O.signs.x;
      needToFillFrom[C001].pos.y = O.center.y;
      needToFillFrom[C001].pos.z = O.center.z;
      needToFillFrom[C001].width = D.actualWidth[C001];
      done[C001] = false;
    } else {
      /*! WE are the coarser one - use fill method */
      O.value[C001] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,O.center.y,O.center.z),C.width);
      coarseFilled = true;
      done[C001]    = true;
    }
  }

  /* ----------- side C010 ----------- */ {
    if (D.actualWidth[C010] == C.width) {
      /* same level - interpolate and done */
      O.value[C010] = 0.5f * (C.value + D.value[C010]);
      done[C010]    = true;
    } else if (isCoarser(D.actualWidth[C010],C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C010].pos.x = O.center.x;
      needToFillFrom[C010].pos.y = O.center.y + (0.5f + delta) * C.width * O.signs.y;
      needToFillFrom[C010].pos.z = O.center.z;
      needToFillFrom[C010].width = D.actualWidth[C010];
      done[C010] = false;
    } else {
      /*! WE are the coarser one - use fill method */
      O.value[C010] = coarseBoundaryValue(self,make_vec3f(O.center.x,O.vertex.y,O.center.z),C.width);
      coarseFilled = true;
      done[C010]    = true;
    }
  }

  /* ----------- side C100 ----------- */ {
    if (D.actualWidth[C100] == C.width) {
      /* same level - interpolate and done */
      O.value[C100] = 0.5f * (C.value + D.value[C100]);
      done[C100]    = true;
    } else if (isCoarser(D.actualWidth[C100],C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C100].pos.x = O.center.x;
      needToFillFrom[C100].pos.y = O.center.y;
      needToFillFrom[C100].pos.z = O.center.z + (0.5f + delta) * C.width * O.signs.z;
      needToFillFrom[C100].width = D.actualWidth[C100];
      done[C100] = false;
    } else {
      /*! WE are the coarser one - use fill method */
      O.value[C100] = coarseBoundaryValue(self,make_vec3f(O.center.x,O.center.y,O.vertex.z),C.width);
      coarseFilled = true;
      done[C100]    = true;
    }
  }

  /* ###################### EDGES ###################### */
  /* edges touch three neighbors. check if ALL are on same level, and
     average if so. if not, check if AT LEAST ONE is coarser, and if
     so, determine COARSEST neighbor and defer vertex to this. if this
     case doesn't hit, either, we know we're the coarser one to at
     least one of the neighbors, with no other neighbor begin even
     coarser - ie, 'we' (ie, this vertex) is on the bounardy, and
     we're the coarse side to fill it */

  /* ----------- edge C011 ----------- */ {
    const float maxWidth  = max(D.actualWidth[C001],D.actualWidth[C010],D.actualWidth[C011]);
    const bool  allLeaves = (D.isLeaf[C001] & D.isLeaf[C010] & D.isLeaf[C011]);
    if (isCoarser(maxWidth,C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C011] = C;
      // check if C001 is closer
      if (isCoarser(D.actualWidth[C001],needToFillFrom[C011])) {
        needToFillFrom[C011].pos.x = O.center.x + (0.5f + delta) * C.width * O.signs.x;
        needToFillFrom[C011].pos.y = O.center.y;
        needToFillFrom[C011].pos.z = O.center.z;
        needToFillFrom[C011].width = D.actualWidth[C001];
      }
      // check if C010 is closer
      if (isCoarser(D.actualWidth[C010],needToFillFrom[C011])) {
        needToFillFrom[C011].pos.x = O.center.x;
        needToFillFrom[C011].pos.y = O.center.y + (0.5f + delta) * C.width * O.signs.y;
        needToFillFrom[C011].pos.z = O.center.z;
        needToFillFrom[C011].width = D.actualWidth[C010];
      }
      // check if C011 is closer
      if (isCoarser(D.actualWidth[C011],needToFillFrom[C011])) {
        needToFillFrom[C011].pos.x = O.center.x + (0.5f + delta) * C.width * O.signs.x;
        needToFillFrom[C011].pos.y = O.center.y + (0.5f + delta) * C.width * O.signs.y;
        needToFillFrom[C011].pos.z = O.center.z;
        needToFillFrom[C011].width = D.actualWidth[C011];
      }
      done[C011]    = false;
    } else if (!allLeaves) {
      /*! WE are the coarser one - use fill method */
      O.value[C011] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,O.vertex.y,O.center.z),C.width);
      coarseFilled = true;
      done[C011]    = true;
    } else {
      O.value[C011] = 0.25f*(C.value+D.value[C001]+D.value[C010]+D.value[C011]);
      done[C011]    = true;
    }
  }

  /* ----------- edge C101 ----------- */ {
    const float maxWidth  = max(D.actualWidth[C001],D.actualWidth[C100],D.actualWidth[C101]);
    const bool  allLeaves = (D.isLeaf[C001] & D.isLeaf[C100] & D.isLeaf[C101]);
    if (isCoarser(maxWidth,C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C101] = C;
      // check if C001 is closer
      if (isCoarser(D.actualWidth[C001],needToFillFrom[C101])) {
        needToFillFrom[C101].pos.x = O.center.x + (0.5f + delta) * C.width * O.signs.x;
        needToFillFrom[C101].pos.y = O.center.y;
        needToFillFrom[C101].pos.z = O.center.z;
        needToFillFrom[C101].width = D.actualWidth[C001];
      }
      // check if C100 is closer
      if (isCoarser(D.actualWidth[C100],needToFillFrom[C101])) {
        needToFillFrom[C101].pos.x = O.center.x;
        needToFillFrom[C101].pos.y = O.center.y;
        needToFillFrom[C101].pos.z = O.center.z + (0.5f + delta) * C.width * O.signs.z;
        needToFillFrom[C101].width = D.actualWidth[C100];
      }
      // check if C101 is closer
      if (isCoarser(D.actualWidth[C101],needToFillFrom[C101])) {
        needToFillFrom[C101].pos.x = O.center.x + (0.5f + delta) * C.width * O.signs.x;
        needToFillFrom[C101].pos.y = O.center.y;
        needToFillFrom[C101].pos.z = O.center.z + (0.5f + delta) * C.width * O.signs.z;
        needToFillFrom[C101].width = D.actualWidth[C101];
      }
      done[C101]    = false;
    } else if (!allLeaves) {
      /*! WE are the coarser one - use fill method */
      O.value[C101] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,O.center.y,O.vertex.z),C.width);
      coarseFilled = true;
      done[C101]    = true;
    } else {
      O.value[C101] = 0.25f*(C.value+D.value[C001]+D.value[C100]+D.value[C101]);
      done[C101]    = true;
    }
  }

  /* ----------- edge C110 ----------- */ {
    const float maxWidth = max(D.actualWidth[C010],D.actualWidth[C100],D.actualWidth[C110]);
    const bool  allLeaves = (D.isLeaf[C010] & D.isLeaf[C100] & D.isLeaf[C110]);
    if (isCoarser(maxWidth,C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C110] = C;
      // check if C010 is closer
      if (isCoarser(D.actualWidth[C010],needToFillFrom[C110])) {
        needToFillFrom[C110].pos.x = O.center.x;
        needToFillFrom[C110].pos.y = O.center.y + (0.5f + delta) * C.width * O.signs.y;
        needToFillFrom[C110].pos.z = O.center.z;
        needToFillFrom[C110].width = D.actualWidth[C010];
      }
      // check if C100 is closer
      if (isCoarser(D.actualWidth[C100],needToFillFrom[C110])) {
        needToFillFrom[C110].pos.x = O.center.x;
        needToFillFrom[C110].pos.y = O.center.y;
        needToFillFrom[C110].pos.z = O.center.z + (0.5f + delta) * C.width * O.signs.z;
        needToFillFrom[C110].width = D.actualWidth[C100];
      }
      // check if C110 is closer
      if (isCoarser(D.actualWidth[C110],needToFillFrom[C110])) {
        needToFillFrom[C110].pos.x = O.center.x;
        needToFillFrom[C110].pos.y = O.center.y + (0.5f + delta) * C.width * O.signs.y;
        needToFillFrom[C110].pos.z = O.center.z + (0.5f + delta) * C.width * O.signs.z;
        needToFillFrom[C110].width = D.actualWidth[C110];
      }
      done[C110]    = false;
    } else if (!allLeaves) {
      /*! WE are the coarser one - use fill method */
      O.value[C110] = coarseBoundaryValue(self,make_vec3f(O.center.x,O.vertex.y,O.vertex.z),C.width);
      done[C110]    = true;
      coarseFilled = true;
    } else {
      O.value[C110] = 0.25f*(C.value+D.value[C010]+D.value[C100]+D.value[C110]);
      done[C110]    = true;
    }
  }

  /* ###################### VERTEX ###################### */
  /* the vertex touches all seven neighbors. if all are on the same
     level, then we aren't on a bounary and can average; if not, but
     at least one is coarser, we have find the coarSEST neighbor and
     defer to him; if neither of those two cases applies we're on a
     boundary but are the coarsest, so can backfill */

  /* ----------- vertex ----------- */ {
    const float maxWidth = max(D.actualWidth[0],D.actualWidth[1],D.actualWidth[2],D.actualWidth[3],
                               D.actualWidth[4],D.actualWidth[5],D.actualWidth[6],D.actualWidth[7]);
    const bool allLeaves = (D.isLeaf[0] & D.isLeaf[1] & D.isLeaf[2] & D.isLeaf[3] &
                            D.isLeaf[4] & D.isLeaf[5] & D.isLeaf[6] & D.isLeaf[7]);
    if ((maxWidth == C.width) && allLeaves) {
      /* all on same level. average, and done */
      O.value[C111] = 0.125f*(D.value[0]+D.value[1]+D.value[2]+D.value[3]+
                              D.value[4]+D.value[5]+D.value[6]+D.value[7]);
      done[C111] = true;
    } else if (isCoarser(maxWidth,C)) {
      /* at least one is coarser - find it, and fill from that neighbor */
      needToFillFrom[C111] = C;
      for (uniform int cID=1;cID<8;cID++) {
        if (isCoarser(D.actualWidth[cID],needToFillFrom[C111])) {
          needToFillFrom[C111].pos.x = (cID & 1) ? O.center.x + (0.5f + delta) * C.width * O.signs.x : O.center.x;
          needToFillFrom[C111].pos.y = (cID & 2) ? O.center.y + (0.5f + delta) * C.width * O.signs.y : O.center.y;
          needToFillFrom[C111].pos.z = (cID & 4) ? O.center.z + (0.5f + delta) * C.width * O.signs.z : O.center.z;
          needToFillFrom[C111].width = D.actualWidth[cID];
        }
      }
      done[C111]    = false;
    } else {
      /* none is coarser, but at least one is finer. boundary fill this vertex */
      O.value[C111] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,O.vertex.y,O.vertex.z),C.width);
      done[C111]    = true;
      coarseFilled = true;
    }
  }

  for (uniform int ii=0;ii<8;ii++) {
    if (done[ii]) continue;
    // print("Conner %d fill from coarser!\n");
 //   print("pos:[%,%,%]\n",P.x, P.y, P.z);
    vec3f vtxPos = make_vec3f((ii & 1) ? O.vertex.x : O.center.x,
                                    (ii & 2) ? O.vertex.y : O.center.y,
                                    (ii & 4) ? O.vertex.z : O.center.z);
    // this isn't actually necessary: in theory we already KNOW this
    // cell from the dual cell. for now, do the actual findcell again,
    // just to make sure we have all the right values initialized
    vtxPos = vtxPos * rcp(self->gridWorldSpace) * self->gridWorldSpace;
    const CellRef fillFrom = findLeafCell(self->_voxelAccel,needToFillFrom[ii].pos);
    O.value[ii] = doOctant(self,fillFrom,vtxPos);
    done[ii] = true;
  }

  //return 0.5;

  return lerp(O);
}


varying float TAMR_octant(const void *uniform _self, const varying vec3f &P)
{
  uniform TAMRVolume *uniform self = (uniform uniform TAMRVolume *uniform)_self;

  vec3f lP;  //local amr space
  self->transformWorldToLocal(self, P, lP);

//  lP = make_vec3f(2.178431,3.028119,1.840075);
//  lP = make_vec3f(1.75,3.028119,0.75);
//  lP = make_vec3f(2.19,4.6,0.3);

  const CellRef cell = findLeafCell(self->_voxelAccel,lP);
   float res;
  if(cell.value == 0.0)
    res = cell.value;
  else
    res = doOctant(_self,cell,lP);
  // PRINT(res);
  return res;
}




export void TAMR_install_octant(void *uniform _self)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume *uniform)_self;
  self->super.sample = TAMR_octant;
}
