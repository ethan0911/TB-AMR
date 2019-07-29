// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "volume/amr/CellRef.ih"
#include "volume/amr/AMRVolume.ih"
#include "volume/amr/DualCell.ih"

// ======================================================================== //
// Useful functions from AMR 
// ======================================================================== //
#define DBG(a)

struct Octant {
  // -------------------------------------------------------
  /* input parameters that specify the cell. coords must be the actual
     cell centers on the desired level */
  // vec3f centerPos;
  vec3f signs;
  vec3i mirror;
  bool left_x, left_y, left_z;

  // position of cell-center that spans this octant
  vec3f center;
  // position of cell-vertex that spans other extreme of this octant
  vec3f vertex;

  //! interpolation weights
  vec3f weights; // weight for C000 being at the CELL CENTER, and C111 
                 // being the CELL CORNER
  // -------------------------------------------------------
  /* values as found the by the query. C000 is always the CELL CENTER, 
     and C111 the CELL CORNER */
  // the respective level (width) we require for the respective i'th octant
  // corner
  float cornerWidth[8];
  float value[8];
};

inline float lerp(const Octant &O)
{
  const vec3f &w = O.weights;
  const float f000 = O.value[C000];
  const float f001 = O.value[C001];
  const float f010 = O.value[C010];
  const float f011 = O.value[C011];
  const float f100 = O.value[C100];
  const float f101 = O.value[C101];
  const float f110 = O.value[C110];
  const float f111 = O.value[C111];

  const float f00 = (1.f-w.x)*f000 + w.x*f001;
  const float f01 = (1.f-w.x)*f010 + w.x*f011;
  const float f10 = (1.f-w.x)*f100 + w.x*f101;
  const float f11 = (1.f-w.x)*f110 + w.x*f111;

  const float f0 = (1.f-w.y)*f00+w.y*f01;
  const float f1 = (1.f-w.y)*f10+w.y*f11;

  const float f = (1.f-w.z)*f0+w.z*f1;
  return f;
}

inline bool isCoarser(const float width, const CellRef &C)
{
  return width > C.width;
}

void initOctantAndDual(Octant &O, DualCell &D, const vec3f &P, 
		       const CellRef &C)
{
  const float cellWidth = C.width;
  const float halfCellWidth = cellWidth * 0.5f;
  const float rcpCellWidth  = rcp(cellWidth);
  const vec3f xfmed = (P-halfCellWidth)*rcpCellWidth;
  const vec3f f_idx = floor(xfmed);
  D.cellID.pos   = f_idx * cellWidth + halfCellWidth;

  // correction due to apparent rounding errors. in some rare cases
  // where the point is exactly ON the right-side bounding plane we
  // compute the lower-side dual cell rather than the right-side dual
  // cell, and that confuses a few things below
  if ((P.x - D.cellID.pos.x) >= C.width) D.cellID.pos.x += C.width;
  if ((P.y - D.cellID.pos.y) >= C.width) D.cellID.pos.y += C.width;
  if ((P.z - D.cellID.pos.z) >= C.width) D.cellID.pos.z += C.width;

  D.cellID.width = cellWidth;

  const vec3f CC = centerOf(C);
  O.left_x = P.x < CC.x;
  O.left_y = P.y < CC.y;
  O.left_z = P.z < CC.z;
  O.mirror.x = O.left_x ? 1 : 0;
  O.mirror.y = O.left_y ? 1 : 0;
  O.mirror.z = O.left_z ? 1 : 0;

  O.signs = make_vec3f(O.left_x ? -1.f : +1.f,
                       O.left_y ? -1.f : +1.f,
                       O.left_z ? -1.f : +1.f);

  O.center = CC;
  O.vertex = O.center + O.signs * halfCellWidth;

  O.weights = abs(P - O.center) * (2.f * rcpCellWidth);
}

inline int getCurrentLevel(const AMR *uniform amr,const float currentWidth)
{
  int nLevel = amr->finestLevel->level + 1;
  for(uniform int i = 0; i < nLevel;i++){
    if(amr->level[i].cellWidth == currentWidth)
      return i;
  }
}

//! hats from leaves only on current level
inline float coarseBoundaryValue(const AMR *uniform amr,
                                 const vec3f &P,
                                 const float currentWidth)
{

  // DualCell D;
  // initDualCell(D,P,currentWidth);
  // findDualCell(amr,D);

  // float sumWeights = 0.f;
  // float sumWeighted = 0.f;
  // for (uniform int i=0;i<8;i++) {
  //   if (D.isLeaf[i]) {
  //     sumWeights += 1.f;
  //     sumWeighted += D.value[i];
  //   }
  // }
  // return sumWeighted / sumWeights;

  // DualCell DC;
  // initDualCell(DC, P, currentWidth);
  // findDualCell(amr, DC);

  // //int currentLevel = getCurrentLevel(amr,currentWidth);
  // //DualCell DN;
  // //initDualCell(DN,P,amr->level[currentLevel+1].cellWidth);
  // //findDualCell(amr, DN);

  DualCell DF;
  initDualCell(DF, P, *amr->finestLevel);
  findDualCell(amr, DF);

  // float fc = 0.5f;
  // float ff = 0.5f;

  // //float fn = 0.4f;
  // //return fc* lerp(DC) + ff* lerp(DF); //+ fn*lerp(DN);

  return lerp(DF);
}

/*! do octant method for point P, in (leaf) cell C.  having this in a
  separate function allows for call it recursively from neighboring
  cells if so required */
varying float doOctant(const AMR *uniform self,
                       const CellRef &C,
                       const varying vec3f &P)
{
  /* first - find the given octant, dual cell, etc */
  Octant O;
  DualCell D;
  initOctantAndDual(O,D,P,C);
  findMirroredDualCell(self,O.mirror,D);

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

  /* ###################### SIDES ###################### */
  /* sides touch one neighbor. we can interpolate if it's on the same
     level, will have to defer to that neighbor if that neighbor is
     coarser, and compute our own if that neighbor is finer */

  /* ----------- side C001 ----------- */ {
    if ((D.actualWidth[C001] == C.width) & D.isLeaf[C001]) {
      /* same level - interpolate and done */
      O.value[C001] = 0.5f * (C.value + D.value[C001]);
      done[C001]    = true;
    } else if (isCoarser(D.actualWidth[C001],C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C001].pos.x = O.center.x+
	0.5f*(C.width+D.actualWidth[C001])*O.signs.x;
      needToFillFrom[C001].pos.y = O.center.y;
      needToFillFrom[C001].pos.z = O.center.z;
      needToFillFrom[C001].width = D.actualWidth[C001];
      done[C001] = false;
    } else {
      /*! WE are the coarser one - use fill method */
      O.value[C001] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,
							  O.center.y,
							  O.center.z),
					  C.width);
      coarseFilled = true;
      done[C001]    = true;
    }
  }

  /* ----------- side C010 ----------- */ {
    if ((D.actualWidth[C010] == C.width) & D.isLeaf[C010]) {
      /* same level - interpolate and done */
      O.value[C010] = 0.5f * (C.value + D.value[C010]);
      done[C010]    = true;
    } else if (isCoarser(D.actualWidth[C010],C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C010].pos.x = O.center.x;
      needToFillFrom[C010].pos.y = O.center.y+
	0.5f*(C.width+D.actualWidth[C010])*O.signs.y;
      needToFillFrom[C010].pos.z = O.center.z;
      needToFillFrom[C010].width = D.actualWidth[C010];
      done[C010] = false;
    } else {
      /*! WE are the coarser one - use fill method */
      O.value[C010] = coarseBoundaryValue(self,make_vec3f(O.center.x,
							  O.vertex.y,
							  O.center.z),
					  C.width);
      coarseFilled = true;
      done[C010]    = true;
    }
  }

  /* ----------- side C100 ----------- */ {
    if ((D.actualWidth[C100] == C.width) & D.isLeaf[C100]) {
      /* same level - interpolate and done */
      O.value[C100] = 0.5f * (C.value + D.value[C100]);
      done[C100]    = true;
    } else if (isCoarser(D.actualWidth[C100],C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C100].pos.x = O.center.x;
      needToFillFrom[C100].pos.y = O.center.y;
      needToFillFrom[C100].pos.z = O.center.z+
	0.5f*(C.width+D.actualWidth[C100])*O.signs.z;
      needToFillFrom[C100].width = D.actualWidth[C100];
      done[C100] = false;
    } else {
      /*! WE are the coarser one - use fill method */
      O.value[C100] = coarseBoundaryValue(self,make_vec3f(O.center.x,
							  O.center.y,
							  O.vertex.z),
					  C.width);
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
    const float maxWidth  = 
      max(D.actualWidth[C001],D.actualWidth[C010],D.actualWidth[C011]);
    const bool  allLeaves = 
      (D.isLeaf[C001] & D.isLeaf[C010] & D.isLeaf[C011]);
    if (isCoarser(maxWidth,C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C011] = C;
      // check if C001 is closer
      if (isCoarser(D.actualWidth[C001],needToFillFrom[C011])) {
        needToFillFrom[C011].pos.x = O.center.x+
	  0.5f*(C.width+D.actualWidth[C001])*O.signs.x;
        needToFillFrom[C011].pos.y = O.center.y;
        needToFillFrom[C011].pos.z = O.center.z;
        needToFillFrom[C011].width = D.actualWidth[C001];
      }
      // check if C010 is closer
      if (isCoarser(D.actualWidth[C010],needToFillFrom[C011])) {
        needToFillFrom[C011].pos.x = O.center.x;
        needToFillFrom[C011].pos.y = O.center.y+
	  0.5f*(C.width+D.actualWidth[C010])*O.signs.y;
        needToFillFrom[C011].pos.z = O.center.z;
        needToFillFrom[C011].width = D.actualWidth[C010];
      }
      // check if C011 is closer
      if (isCoarser(D.actualWidth[C011],needToFillFrom[C011])) {
        needToFillFrom[C011].pos.x = O.center.x+
	  0.5f*(C.width+D.actualWidth[C011])*O.signs.x;
        needToFillFrom[C011].pos.y = O.center.y+
	  0.5f*(C.width+D.actualWidth[C011])*O.signs.y;
        needToFillFrom[C011].pos.z = O.center.z;
        needToFillFrom[C011].width = D.actualWidth[C011];
      }
      done[C011]    = false;
    } else if (!allLeaves) {
      /*! WE are the coarser one - use fill method */
      O.value[C011] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,
							  O.vertex.y,
							  O.center.z),
					  C.width);
      coarseFilled = true;
      done[C011]    = true;
    } else {
      O.value[C011] = 0.25f*(C.value+
			     D.value[C001]+
			     D.value[C010]+
			     D.value[C011]);
      done[C011]    = true;
    }
  }


  /* ----------- edge C101 ----------- */ {
    const float maxWidth  = max(D.actualWidth[C001],
				D.actualWidth[C100],
				D.actualWidth[C101]);
    const bool  allLeaves = (D.isLeaf[C001] & 
			     D.isLeaf[C100] & 
			     D.isLeaf[C101]);
    if (isCoarser(maxWidth,C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C101] = C;
      // check if C001 is closer
      if (isCoarser(D.actualWidth[C001],needToFillFrom[C101])) {
        needToFillFrom[C101].pos.x = O.center.x+
	  0.5f*(C.width+D.actualWidth[C001])*O.signs.x;
        needToFillFrom[C101].pos.y = O.center.y;
        needToFillFrom[C101].pos.z = O.center.z;
        needToFillFrom[C101].width = D.actualWidth[C001];
      }
      // check if C100 is closer
      if (isCoarser(D.actualWidth[C100],needToFillFrom[C101])) {
        needToFillFrom[C101].pos.x = O.center.x;
        needToFillFrom[C101].pos.y = O.center.y;
        needToFillFrom[C101].pos.z = O.center.z+
	  0.5f*(C.width+D.actualWidth[C100])*O.signs.z;
        needToFillFrom[C101].width = D.actualWidth[C100];
      }
      // check if C101 is closer
      if (isCoarser(D.actualWidth[C101],needToFillFrom[C101])) {
        needToFillFrom[C101].pos.x = O.center.x+
	  0.5f*(C.width+D.actualWidth[C101])*O.signs.x;
        needToFillFrom[C101].pos.y = O.center.y;
        needToFillFrom[C101].pos.z = O.center.z+
	  0.5f*(C.width+D.actualWidth[C101])*O.signs.z;
        needToFillFrom[C101].width = D.actualWidth[C101];
      }
      done[C101]    = false;
    } else if (!allLeaves) {
      /*! WE are the coarser one - use fill method */
      O.value[C101] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,
							  O.center.y,
							  O.vertex.z),
					  C.width);
      coarseFilled = true;
      done[C101]    = true;
    } else {
      O.value[C101] = 0.25f*(C.value+
			     D.value[C001]+
			     D.value[C100]+
			     D.value[C101]);
      done[C101]    = true;
    }
  }


  /* ----------- edge C110 ----------- */ {
    const float maxWidth = max(D.actualWidth[C010],
			       D.actualWidth[C100],
			       D.actualWidth[C110]);
    const bool  allLeaves = (D.isLeaf[C010] & 
			     D.isLeaf[C100] & 
			     D.isLeaf[C110]);
    if (isCoarser(maxWidth,C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C110] = C;
      // check if C010 is closer
      if (isCoarser(D.actualWidth[C010],needToFillFrom[C110])) {
        needToFillFrom[C110].pos.x = O.center.x;
        needToFillFrom[C110].pos.y = O.center.y+
	  0.5f*(C.width+D.actualWidth[C010])*O.signs.y;
        needToFillFrom[C110].pos.z = O.center.z;
        needToFillFrom[C110].width = D.actualWidth[C010];
      }
      // check if C100 is closer
      if (isCoarser(D.actualWidth[C100],needToFillFrom[C110])) {
        needToFillFrom[C110].pos.x = O.center.x;
        needToFillFrom[C110].pos.y = O.center.y;
        needToFillFrom[C110].pos.z = O.center.z+
	  0.5f*(C.width+D.actualWidth[C100])*O.signs.z;
        needToFillFrom[C110].width = D.actualWidth[C100];
      }
      // check if C110 is closer
      if (isCoarser(D.actualWidth[C110],needToFillFrom[C110])) {
        needToFillFrom[C110].pos.x = O.center.x;
        needToFillFrom[C110].pos.y = O.center.y+
	  0.5f*(C.width+D.actualWidth[C110])*O.signs.y;
        needToFillFrom[C110].pos.z = O.center.z+
	  0.5f*(C.width+D.actualWidth[C110])*O.signs.z;
        needToFillFrom[C110].width = D.actualWidth[C110];
      }
      done[C110]    = false;
    } else if (!allLeaves) {
      /*! WE are the coarser one - use fill method */
      O.value[C110] = coarseBoundaryValue(self,make_vec3f(O.center.x,
							  O.vertex.y,
							  O.vertex.z),
					  C.width);
      done[C110]    = true;
      coarseFilled = true;
    } else {
      O.value[C110] = 0.25f*(C.value+D.value[C010]+
			     D.value[C100]+D.value[C110]);
      done[C110]    = true;
    }
  }

  /* ###################### VERTEX ###################### */
  /* the vertex touches all seven neighbors. if all are on the same
     level, then we aren't on a bounary and can average; if not, but
     at least one is coarser, we have find the coarSEST neighbor and
     defer to him; if neither of those two cases apllies we're on a
     boundary but are the coarsest, so can backfill */

  /* ----------- vertex ----------- */ {
    const float maxWidth = max(D.actualWidth[0],D.actualWidth[1],
			       D.actualWidth[2],D.actualWidth[3],
                               D.actualWidth[4],D.actualWidth[5],
			       D.actualWidth[6],D.actualWidth[7]);
    const bool allLeaves = (D.isLeaf[0] & D.isLeaf[1] & D.isLeaf[2] & 
			    D.isLeaf[3] & D.isLeaf[4] & D.isLeaf[5] & 
			    D.isLeaf[6] & D.isLeaf[7]);
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
          needToFillFrom[C111].pos.x
            = (cID & 1)
            ? O.center.x+0.5f*(C.width+D.actualWidth[cID])*O.signs.x
            : O.center.x;
          needToFillFrom[C111].pos.y
            = (cID & 2)
            ? O.center.y+0.5f*(C.width+D.actualWidth[cID])*O.signs.y
            : O.center.y;
          needToFillFrom[C111].pos.z
            = (cID & 4)
            ? O.center.z+0.5f*(C.width+D.actualWidth[cID])*O.signs.z
            : O.center.z;
          needToFillFrom[C111].width = D.actualWidth[cID];
        }
      }
      done[C111]    = false;
    } else {
      /* none is coarser, but at least one is finer. boundary fill this 
	 vertex */
      O.value[C111] = coarseBoundaryValue(self,make_vec3f(O.vertex.x,
							  O.vertex.y,
							  O.vertex.z),
					  C.width);
      done[C111]    = true;
      coarseFilled = true;
    }
  }

  for (uniform int ii=0;ii<8;ii++) {
    if (done[ii]) continue;
    const vec3f vtxPos = make_vec3f((ii & 1) ? O.vertex.x : O.center.x,
                                    (ii & 2) ? O.vertex.y : O.center.y,
                                    (ii & 4) ? O.vertex.z : O.center.z);
    // this isn't actually necessary: in theory we alreay KNOW this
    // cell from the dual cell. for now, do the actual findcell again,
    // just to make sure we have all the right values initialized
    const CellRef fillFrom = findCell(self,needToFillFrom[ii].pos,
                                      needToFillFrom[ii].width);
    O.value[ii] = doOctant(self,fillFrom,vtxPos);
    done[ii] = true;
  }

  return lerp(O);
}

varying float AMR_octant(void *uniform _self, const varying vec3f &P)
{
  const AMRVolume *uniform self = (AMRVolume *)_self;
  const AMR *uniform amr = &self->amr;

  vec3f lP;  //local amr space
  self->transformWorldToLocal(self, P, lP);

  const CellRef C = findLeafCell(amr,lP);
  return doOctant(amr,C,lP);
}

varying float AMR_current(void *uniform _self, const varying vec3f &P)
{
  const AMRVolume *uniform self = (AMRVolume *)_self;
  const AMR *uniform amr = &self->amr;

  vec3f lP;  //local amr space
  self->transformWorldToLocal(self, P, lP);

  const CellRef C = findLeafCell(amr, lP);
  DualCell D;
  initDualCell(D, lP, C.width);
  findDualCell(amr, D);

  return lerp(D);
}

varying float AMR_finest(void *uniform _self, const varying vec3f &P)
{
  const AMRVolume *uniform self = (AMRVolume *)_self;
  const AMR *uniform amr = &self->amr;

  vec3f lP;  //local amr space
  self->transformWorldToLocal(self, P, lP);

  const CellRef C = findLeafCell(amr, lP);
  DualCell D;
  initDualCell(D,lP,*amr->finestLevel);
  findDualCell(amr, D);

  return lerp(D);
}

varying float AMR_nearest(void *uniform _self, const varying vec3f &P)
{
  const AMRVolume *uniform self = (AMRVolume *)_self;
  const AMR *uniform amr = &self->amr;

  vec3f lP;  //local amr space
  self->transformWorldToLocal(self, P, lP);

  const CellRef C = findLeafCell(amr, lP);
  return C.value;
}

export void getAMRValue_Octant(void *uniform _self,
			       uniform float *uniform resultArray,
			       uniform vec3f *uniform samplePos,
			       uniform float *uniform octsWidth,
			       const uniform int numOct)
{
  AMRVolume *uniform self = (AMRVolume * uniform) _self;
  for(uniform int i = 0; i < numOct; i++) {
    foreach (j = 0 ... 8) {
      const vec3f vtxPos = make_vec3f((j & 1) ? octsWidth[i] : 0.f,
                                      (j & 2) ? octsWidth[i] : 0.f,
                                      (j & 4) ? octsWidth[i] : 0.f);
      vec3f pos          = samplePos[i] + vtxPos;
      int idx = i*8 + j;
      resultArray[idx] = AMR_octant(self, pos);
    }
  }
}

export void getAMRValue_Current(void *uniform _self,
				uniform float *uniform resultArray,
				uniform vec3f *uniform samplePos,
				uniform float *uniform octsWidth,
				const uniform int numOct)
{
  AMRVolume *uniform self = (AMRVolume * uniform) _self;
  for(uniform int i = 0; i < numOct; i++) {
    foreach (j = 0 ... 8) {
      const vec3f vtxPos = make_vec3f((j & 1) ? octsWidth[i] : 0.f,
                                      (j & 2) ? octsWidth[i] : 0.f,
                                      (j & 4) ? octsWidth[i] : 0.f);
      vec3f pos          = samplePos[i] + vtxPos;
      int idx = i*8 + j;
      resultArray[idx] = AMR_current(self, pos);
    }
  }
}

export void getAMRValue_Nearest(void *uniform _self,
				uniform float *uniform resultArray,
				uniform vec3f *uniform samplePos,
				uniform float *uniform octsWidth,
				const uniform int numOct)
{
  AMRVolume *uniform self = (AMRVolume * uniform) _self;
  for(uniform int i = 0; i < numOct; i++) {
    foreach (j = 0 ... 8) {
      const vec3f vtxPos = make_vec3f((j & 1) ? octsWidth[i] : 0.f,
                                      (j & 2) ? octsWidth[i] : 0.f,
                                      (j & 4) ? octsWidth[i] : 0.f);
      vec3f pos          = samplePos[i] + vtxPos;
      int idx = i * 8 + j;
      resultArray[idx] = AMR_nearest(self, pos);
    }
  }
}

// ======================================================================== //
// getOneVoxel 
// ======================================================================== //
export void getOneVoxel_octant(void *uniform _self,
			// outputs
			uniform float &oW,
			uniform vec3f &oC,
			uniform float *uniform oV,
			// inputs
			const uniform float  &cw,
			const uniform vec3f &lower,
			const uniform vec3f &upper,
			const uniform uint32 i,     // grid index
			const uniform uint32 nx,
			const uniform uint32 ny,
			const uniform uint32 nz,
			// different type of cells
			const uniform uint32 n1,
			const uniform uint32 n2,
			const uniform uint32 n3)
{
  AMRVolume *uniform self = (AMRVolume * uniform) _self;
  // so here we need to compute the point position from index
  const uniform float halfCW = 0.5f * cw;
  //
  // compute width and coordinate
  //
  /* add inner cells */
  if (i < n1) { 
    const uniform uint32 ix = i % (nx - 1);
    const uniform uint32 iy = (i % ((ny - 1) * (nx - 1))) / (nx - 1);
    const uniform uint32 iz = i / ((ny - 1) * (nx - 1));
    oW = cw;
    oC.x = lower.x + halfCW + cw * ix;
    oC.y = lower.y + halfCW + cw * iy;
    oC.z = lower.z + halfCW + cw * iz;
  }
  /* bottom top boundray cells */
  else if (i < n2) { 
    const uniform uint32 ix = ((i - n1) % (4 * nx)) / 2;
    const uniform uint32 iy = (i - n1) / (4 * nx);
    oW = halfCW;
    oC.x = lower.x + halfCW * ix;
    oC.y = lower.y + halfCW * iy;
    if ((i-n1) % 2 == 0) {
      oC.z = lower.z;
    } else {
      oC.z = upper.z - halfCW;
    }      
  } 
  /* left right boundray cells */
  else if (i < n3) { 
    const uniform uint32 iy = ((i - n2) % (4 * ny)) / 2;
    const uniform uint32 iz = (i - n2) / (4 * ny);
    oW = halfCW;
    oC.y = lower.y + halfCW * iy;
    oC.z = lower.z + halfCW * iz;
    if ((i-n2) % 2 == 0) {
      oC.x = lower.x;
    } else {
      oC.x = upper.x - halfCW;
    }
  } 
  /* front back boundary cells */
  else { 
    const uniform uint32 ix = ((i - n3) % (4 * nx)) / 2;
    const uniform uint32 iz = (i - n3) / (4 * nx);
    oW = halfCW;
    oC.x = lower.x + halfCW * ix;
    oC.z = lower.z + halfCW * iz;
    if ((i-n3) % 2 == 0) {
      oC.y = lower.y;
    } else {
      oC.y = upper.y - halfCW;
    }           
  } 
  //
  // now we compute voxel value
  //
  foreach (j = 0 ... 8) { 
    // compute coordinate
    const vec3f vp = make_vec3f((j & 1) ? oW : 0.f,
				(j & 2) ? oW : 0.f,
				(j & 4) ? oW : 0.f);
    const vec3f p = oC + vp;
    // hard code octant method here
    oV[j] = AMR_octant(self, p);
  }
}

export void getOneVoxelBounds_octant(void *uniform _self,
				     // outputs
				     uniform float &oW,
				     uniform vec3f &oC,
				     // inputs
				     const uniform float  &cw,
				     const uniform vec3f &lower,
				     const uniform vec3f &upper,
				     const uniform uint32 i, 
				     const uniform uint32 nx,
				     const uniform uint32 ny,
				     const uniform uint32 nz,
				     // different type of cells
				     const uniform uint32 n1,
				     const uniform uint32 n2,
				     const uniform uint32 n3)
{
  AMRVolume *uniform self = (AMRVolume * uniform) _self;
  // so here we need to compute the point position from index
  const uniform float halfCW = 0.5f * cw;
  //
  // compute width and coordinate
  //
  /* add inner cells */
  if (i < n1) { 
    const uniform uint32 ix = i % (nx - 1);
    const uniform uint32 iy = (i % ((ny - 1) * (nx - 1))) / (nx - 1);
    const uniform uint32 iz = i / ((ny - 1) * (nx - 1));
    oW = cw;
    oC.x = lower.x + halfCW + cw * ix;
    oC.y = lower.y + halfCW + cw * iy;
    oC.z = lower.z + halfCW + cw * iz;
  }
  /* bottom top boundray cells */
  else if (i < n2) { 
    const uniform uint32 ix = ((i - n1) % (4 * nx)) / 2;
    const uniform uint32 iy = (i - n1) / (4 * nx);
    oW = halfCW;
    oC.x = lower.x + halfCW * ix;
    oC.y = lower.y + halfCW * iy;
    if ((i-n1) % 2 == 0) {
      oC.z = lower.z;
    } else {
      oC.z = upper.z - halfCW;
    }      
  } 
  /* left right boundray cells */
  else if (i < n3) { 
    const uniform uint32 iy = ((i - n2) % (4 * ny)) / 2;
    const uniform uint32 iz = (i - n2) / (4 * ny);
    oW = halfCW;
    oC.y = lower.y + halfCW * iy;
    oC.z = lower.z + halfCW * iz;
    if ((i-n2) % 2 == 0) {
      oC.x = lower.x;
    } else {
      oC.x = upper.x - halfCW;
    }
  } 
  /* front back boundary cells */
  else { 
    const uniform uint32 ix = ((i - n3) % (4 * nx)) / 2;
    const uniform uint32 iz = (i - n3) / (4 * nx);
    oW = halfCW;
    oC.x = lower.x + halfCW * ix;
    oC.z = lower.z + halfCW * iz;
    if ((i-n3) % 2 == 0) {
      oC.y = lower.y;
    } else {
      oC.y = upper.y - halfCW;
    }           
  } 
}

// ======================================================================== //
// getOneVoxel 
// ======================================================================== //
void getAllVoxels_octant(AMRVolume *uniform self,
			 // outputs
			 float &oW,
			 vec3f &oC,
			 vec2f &oR,
			 float oV[8],
			 // index
			 const varying uint32 i,
			 // inputs
			 const uniform float &cw,    // full cell width
			 const uniform float &halfCW,// half cell width
			 const uniform vec3f &lower, // lower bbox
			 const uniform vec3f &upper, // upper bbox
			 const uniform uint32 nx,  
			 const uniform uint32 ny,
			 const uniform uint32 nz,
			 // different type of cells
			 const uniform uint32 n1,
			 const uniform uint32 n2,
			 const uniform uint32 n3)
{
  //
  // compute width and coordinate
  //
  /* add inner cells */
  if (i < n1) { 
    const uint32 ix = i % (nx - 1);
    const uint32 iy = (i % ((ny - 1) * (nx - 1))) / (nx - 1);
    const uint32 iz = i / ((ny - 1) * (nx - 1));
    oW = cw;
    oC.x = lower.x + halfCW + cw * ix;
    oC.y = lower.y + halfCW + cw * iy;
    oC.z = lower.z + halfCW + cw * iz;
  }
  /* bottom top boundray cells */
  else if (i < n2) { 
    const uint32 ix = ((i - n1) % (4 * nx)) / 2;
    const uint32 iy = (i - n1) / (4 * nx);
    oW = halfCW;
    oC.x = lower.x + halfCW * ix;
    oC.y = lower.y + halfCW * iy;
    if ((i-n1) % 2 == 0) {
      oC.z = lower.z;
    } else {
      oC.z = upper.z - halfCW;
    }      
  } 
  /* left right boundray cells */
  else if (i < n3) { 
    const uint32 iy = ((i - n2) % (4 * ny)) / 2;
    const uint32 iz = (i - n2) / (4 * ny);
    oW = halfCW;
    oC.y = lower.y + halfCW * iy;
    oC.z = lower.z + halfCW * iz;
    if ((i-n2) % 2 == 0) {
      oC.x = lower.x;
    } else {
      oC.x = upper.x - halfCW;
    }
  } 
  /* front back boundary cells */
  else { 
    const uint32 ix = ((i - n3) % (4 * nx)) / 2;
    const uint32 iz = (i - n3) / (4 * nx);
    oW = halfCW;
    oC.x = lower.x + halfCW * ix;
    oC.z = lower.z + halfCW * iz;
    if ((i-n3) % 2 == 0) {
      oC.y = lower.y;
    } else {
      oC.y = upper.y - halfCW;
    }           
  }
  //
  // now we compute voxel value
  //
  oR = make_vec2f(1e20, -1e20);
  for (uint32 j = 0; j < 8; j++) {
    // compute coordinate
    const vec3f vp = make_vec3f((j & 1) ? oW : 0.f,
				(j & 2) ? oW : 0.f,
				(j & 4) ? oW : 0.f);
    vec3f p = oC + vp;
    // hard code octant method here
    oV[j] = AMR_octant(self, p);//AMR_current(self, p);//AMR_octant(self, p);
    // compute range
    oR.x = min(oR.x, oV[j]);
    oR.y = max(oR.y, oV[j]);
  } 
}

/*! function to push back active voxel */
// ======================================================================== //
// Codes to compute voxel values
// ======================================================================== //

unmasked extern "C" externC_push_back_active(void *uniform c_vector,
					     void *uniform c_ptr,
					     const uniform float v0,
					     const uniform float v1,
					     const uniform float v2,
					     const uniform float v3,
					     const uniform float v4,
					     const uniform float v5,
					     const uniform float v6,
					     const uniform float v7,
					     const uniform float c0,
					     const uniform float c1,
					     const uniform float c2,
					     const uniform float cellwidth);

unmasked extern "C" externC_push_back_none(void *uniform c_vector, 
					   const void *uniform c_ptr,
					   const uniform uint32 lid,
					   const uniform uint32 oid,
					   const uniform float c0,
					   const uniform float c1,
					   const uniform float c2,
					   const uniform float cellwidth);

// ======================================================================== //
// Store active
// ======================================================================== //
export void getAllVoxels_active(void *uniform _self,
                                void *uniform _cptr,    // C pointer
                                void *uniform _vector,  // STL vector in C++
                                const uniform float &isovalue,
                                const uniform float &fcw,
                                const uniform vec3f &lower,
                                const uniform vec3f &upper,
                                const uniform uint32 b,  // begin
                                const uniform uint32 e,  // end
                                const uniform uint32 nx,
                                const uniform uint32 ny,
                                const uniform uint32 nz,
                                // different type of cells
                                const uniform uint32 n1,
                                const uniform uint32 n2,
                                const uniform uint32 n3)
{
  AMRVolume *uniform self = (AMRVolume * uniform) _self;
  // so here we need to compute the point position from index
  const uniform float hcw = 0.5f * fcw;
  foreach (i = b... e) {
    // compute voxels
    float oW;
    vec3f oC;
    vec2f rg;
    float oV[8];
    getAllVoxels_octant(self,
                        /* outputs */ oW,
                        oC,
                        rg,
                        oV,
                        /* index */ i,
                        /* inputs */ fcw,
                        hcw,
                        lower,
                        upper,
                        nx,
                        ny,
                        nz,
                        /* different type of cells */ n1,
                        n2,
                        n3);
    bool inRange = rg.x < isovalue && rg.y > isovalue;
    foreach_active(pid)
    {
      if (inRange) {
        externC_push_back_active(_vector,
                                 _cptr,
                                 extract(oV[0], pid),
                                 extract(oV[1], pid),
                                 extract(oV[2], pid),
                                 extract(oV[3], pid),
                                 extract(oV[4], pid),
                                 extract(oV[5], pid),
                                 extract(oV[6], pid),
                                 extract(oV[7], pid),
                                 extract(oC.x, pid),
                                 extract(oC.y, pid),
                                 extract(oC.z, pid),
                                 extract(oW, pid));
      }
    }
  }
}

// ======================================================================== //
// Store none
// ======================================================================== //
export void getAllVoxels_none(void *uniform _self,
			      const void *uniform _cptr,   // C pointer
			      void *uniform _vector, // STL vector in C++
			      const uniform float &isovalue,
			      const uniform float &fcw,
			      const uniform uint32 &lid,
			      const uniform vec3f &lower,
			      const uniform vec3f &upper,
			      const uniform uint32 b,   // begin
			      const uniform uint32 e,   // end
			      const uniform uint32 nx,
			      const uniform uint32 ny,
			      const uniform uint32 nz,
			      // different type of cells
			      const uniform uint32 n1,
			      const uniform uint32 n2,
			      const uniform uint32 n3)
{
  AMRVolume *uniform self = (AMRVolume * uniform) _self;
  // so here we need to compute the point position from index
  const uniform float hcw = 0.5f * fcw;
  foreach (i = b ... e) {    
    // compute voxels
    float oW;
    vec3f oC;
    vec2f rg;
    float oV[8];    
    getAllVoxels_octant(self,
			/* outputs */oW, oC, rg, oV,
			/* index */i,
			/* inputs */fcw, hcw, lower, upper, nx,ny, nz,
			/* different type of cells */n1, n2, n3);
    // push_back active voxels
    bool inRange = rg.x < isovalue && rg.y > isovalue;
    foreach_active(pid) {
      if (inRange) {
	externC_push_back_none(_vector, _cptr, lid, extract(i, pid),
			       extract(oC.x, pid),
			       extract(oC.y, pid),
			       extract(oC.z, pid),
			       extract(oW, pid));
      }
    }
  }
}

