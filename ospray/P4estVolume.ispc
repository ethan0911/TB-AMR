#include "P4estVolume.ih"

#define BASIS 0
#define FACE 0
#define EDGE 0
#define VERTEX 0

/*! do octant method for point P, in (leaf) cell C.  having this in a
  separate function allows for call it recursively from neighboring
  cells if so required */
varying float doTrilinear(const void *uniform _self,
                          const CellRef &C,
                          const varying vec3f &P)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;

  /* first - find the given octant, dual cell, etc */
  Octant O;
  DualCell D;
  initOctantAndDual(O, D, P, C);
  findMirroredDualCell(self->_voxelAccel, O.mirror, D);

  // print("dual::AcualWidth:[%,%,%,%,%,%,%,%]\n",
  //     D.actualWidth[0],D.actualWidth[1],
  //     D.actualWidth[2],D.actualWidth[3],
  //     D.actualWidth[4],D.actualWidth[5],
  //     D.actualWidth[6],D.actualWidth[7]);

  // print("dual:value:[%,%,%,%,%,%,%,%]\n",
  //       D.value[0],D.value[1],
  //       D.value[2],D.value[3],
  //       D.value[4],D.value[5],
  //       D.value[6],D.value[7]);

  // print("==========================\n");

  // print("pos:[%,%,%]\n",P.x, P.y, P.z);

  if (isDualCellInSameLevel(D))
    return lerp(D);

  for (uniform int i = 0; i < 8; i++)
    O.value[i] = -1.0f;

  /* initialize corner computation. for each corner we compute if we
     could fill it from the current octant/dual cell ('done'), and, if
     not, which other cell it should be filled from ('needToFillFrom') */
  bool done[8];
  CellRef needToFillFrom[8];

  /* ###################### CENTER ###################### */
  /* the center point is ALWAYS the cell value */
  O.value[C000]     = C.value;
  done[C000]        = true;
  bool coarseFilled = false;

  /* ###################### SIDES ###################### */
  /* sides touch one neighbor. we can interpolate if it's on the same
     level, will have to defer to that neighbor if that neighbor is
     coarser, and compute our own if that neighbor is finer */

  /* ----------- side C001 ----------- */ {
    if (D.actualWidth[C001] == C.width) {
      /* same level - interpolate and done */
      O.value[C001] = 0.5f * (C.value + D.value[C001]);
      done[C001]    = true;
    } else if (isCoarser(D.actualWidth[C001], C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C001].pos.x =
          O.center.x + (0.5f + delta) * C.width * O.signs.x;
      needToFillFrom[C001].pos.y = O.center.y;
      needToFillFrom[C001].pos.z = O.center.z;
      needToFillFrom[C001].width = D.actualWidth[C001];
      done[C001]                 = false;
    } else {
    /*! WE are the coarser one - use fill method */
#if FACE
      O.value[C001] = coarseBoundaryValue(
          self, make_vec3f(O.vertex.x, O.center.y, O.center.z), C.width);
#else
      vec3f P1 = make_vec3f(O.center.x + (0.5f + delta) * C.width * O.signs.x,
                            O.center.y + delta * C.width * O.signs.y,
                            O.center.z + delta * C.width * O.signs.z);
      CellRef cell = findLeafCell(self->_voxelAccel, P1);
      Octant O1;
      DualCell D1;
      initOctantAndDual(O1, D1, P1, cell);
      findMirroredDualCell(self->_voxelAccel, O1.mirror, D1);

      // print("C000:%f, C010:%f, C100: %d, C110: %f \n",
      //        D1.value[C000],
      //        D1.value[C010],
      //        D1.value[C100],
      //        D1.value[C110]);

      O.value[C001] = O.value[C000] / 3.f + D1.value[C000] / 6.f +
                      D1.value[C010] / 6.f + D1.value[C100] / 6.f +
                      D1.value[C110] / 6.f;
#endif
      coarseFilled = true;
      done[C001]   = true;
    }
  }

  /* ----------- side C010 ----------- */ {
    if (D.actualWidth[C010] == C.width) {
      /* same level - interpolate and done */
      O.value[C010] = 0.5f * (C.value + D.value[C010]);
      done[C010]    = true;
    } else if (isCoarser(D.actualWidth[C010], C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C010].pos.x = O.center.x;
      needToFillFrom[C010].pos.y =
          O.center.y + (0.5f + delta) * C.width * O.signs.y;
      needToFillFrom[C010].pos.z = O.center.z;
      needToFillFrom[C010].width = D.actualWidth[C010];
      done[C010]                 = false;
    } else {
    /*! WE are the coarser one - use fill method */
#if FACE
      O.value[C010] = coarseBoundaryValue(
          self, make_vec3f(O.center.x, O.vertex.y, O.center.z), C.width);
#else
      vec3f P1     = make_vec3f(O.center.x + delta * C.width * O.signs.x,
                            O.center.y + (0.5f + delta) * C.width * O.signs.y,
                            O.center.z + delta * C.width * O.signs.z);
      CellRef cell = findLeafCell(self->_voxelAccel, P1);
      Octant O1;
      DualCell D1;
      initOctantAndDual(O1, D1, P1, cell);
      findMirroredDualCell(self->_voxelAccel, O1.mirror, D1);

      O.value[C010] = O.value[C000] / 3.f + D1.value[C000] / 6.f +
                      D1.value[C001] / 6.f + D1.value[C100] / 6.f +
                      D1.value[C101] / 6.f;
#endif
      coarseFilled = true;
      done[C010]   = true;
    }
  }

  /* ----------- side C100 ----------- */ {
    if (D.actualWidth[C100] == C.width) {
      /* same level - interpolate and done */
      O.value[C100] = 0.5f * (C.value + D.value[C100]);
      done[C100]    = true;
    } else if (isCoarser(D.actualWidth[C100], C)) {
      /* neighbor is coarser - use the neighbor */
      needToFillFrom[C100].pos.x = O.center.x;
      needToFillFrom[C100].pos.y = O.center.y;
      needToFillFrom[C100].pos.z =
          O.center.z + (0.5f + delta) * C.width * O.signs.z;
      needToFillFrom[C100].width = D.actualWidth[C100];
      done[C100]                 = false;
    } else {
    /*! WE are the coarser one - use fill method */
#if FACE
      O.value[C100] = coarseBoundaryValue(
          self, make_vec3f(O.center.x, O.center.y, O.vertex.z), C.width);
#else
      vec3f P1     = make_vec3f(O.center.x + delta * C.width * O.signs.x,
                            O.center.y + delta * C.width * O.signs.y,
                            O.center.z + (0.5f + delta) * C.width * O.signs.z);
      CellRef cell = findLeafCell(self->_voxelAccel, P1);
      Octant O1;
      DualCell D1;
      initOctantAndDual(O1, D1, P1, cell);
      findMirroredDualCell(self->_voxelAccel, O1.mirror, D1);

      O.value[C100] = O.value[C000] / 3.f + D1.value[C000] / 6.f +
                      D1.value[C001] / 6.f + D1.value[C010] / 6.f +
                      D1.value[C011] / 6.f;
#endif
      coarseFilled = true;
      done[C100]   = true;
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
    const float maxWidth =
        max(D.actualWidth[C001], D.actualWidth[C010], D.actualWidth[C011]);
    const bool allLeaves = (D.isLeaf[C001] & D.isLeaf[C010] & D.isLeaf[C011]);
    if (isCoarser(maxWidth, C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C011] = C;
      // check if C001 is closer
      if (isCoarser(D.actualWidth[C001], needToFillFrom[C011])) {
        needToFillFrom[C011].pos =
            make_vec3f(O.center.x + (0.5f + delta) * C.width * O.signs.x,
                       O.center.y,
                       O.center.z);
        needToFillFrom[C011].width = D.actualWidth[C001];
      }
      // check if C010 is closer
      if (isCoarser(D.actualWidth[C010], needToFillFrom[C011])) {
        needToFillFrom[C011].pos =
            make_vec3f(O.center.x,
                       O.center.y + (0.5f + delta) * C.width * O.signs.y,
                       O.center.z);
        needToFillFrom[C011].width = D.actualWidth[C010];
      }
      // check if C011 is closer
      if (isCoarser(D.actualWidth[C011], needToFillFrom[C011])) {
        needToFillFrom[C011].pos =
            make_vec3f(O.center.x + (0.5f + delta) * C.width * O.signs.x,
                       O.center.y + (0.5f + delta) * C.width * O.signs.y,
                       O.center.z);
        needToFillFrom[C011].width = D.actualWidth[C011];
      }
      done[C011] = false;
    } else if (!allLeaves) {
    /*! WE are the coarser one - use fill method */
#if EDGE
      O.value[C011] = coarseBoundaryValue(
          self, make_vec3f(O.vertex.x, O.vertex.y, O.center.z), C.width);
#else
      O.value[C011] = stitchCoarserEdge(self, C, O, D, C011);
#endif
      coarseFilled = true;
      done[C011]   = true;
    } else {
      O.value[C011] = 0.25f * (C.value + D.value[C001] + D.value[C010] + D.value[C011]);
      done[C011] = true;
    }
  }

  /* ----------- edge C101 ----------- */ {
    const float maxWidth =
        max(D.actualWidth[C001], D.actualWidth[C100], D.actualWidth[C101]);
    const bool allLeaves = (D.isLeaf[C001] & D.isLeaf[C100] & D.isLeaf[C101]);
    if (isCoarser(maxWidth, C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C101] = C;
      // check if C001 is closer
      if (isCoarser(D.actualWidth[C001], needToFillFrom[C101])) {
        needToFillFrom[C101].pos =
            make_vec3f(O.center.x + (0.5f + delta) * C.width * O.signs.x,
                       O.center.y,
                       O.center.z);
        needToFillFrom[C101].width = D.actualWidth[C001];
      }
      // check if C100 is closer
      if (isCoarser(D.actualWidth[C100], needToFillFrom[C101])) {
        needToFillFrom[C101].pos =
            make_vec3f(O.center.x,
                       O.center.y,
                       O.center.z + (0.5f + delta) * C.width * O.signs.z);
        needToFillFrom[C101].width = D.actualWidth[C100];
      }
      // check if C101 is closer
      if (isCoarser(D.actualWidth[C101], needToFillFrom[C101])) {
        needToFillFrom[C101].pos =
            make_vec3f(O.center.x + (0.5f + delta) * C.width * O.signs.x,
                       O.center.y,
                       O.center.z + (0.5f + delta) * C.width * O.signs.z);
        needToFillFrom[C101].width = D.actualWidth[C101];
      }
      done[C101] = false;
    } else if (!allLeaves) {
    /*! WE are the coarser one - use fill method */
#if EDGE
      O.value[C101] = coarseBoundaryValue(
          self, make_vec3f(O.vertex.x, O.center.y, O.vertex.z), C.width);
#else
      O.value[C101] = stitchCoarserEdge(self, C, O, D, C101);
#endif

      coarseFilled = true;
      done[C101]   = true;
    } else {
      O.value[C101] =
          0.25f * (C.value + D.value[C001] + D.value[C100] + D.value[C101]);
      done[C101] = true;
    }
  }

  /* ----------- edge C110 ----------- */ {
    const float maxWidth =
        max(D.actualWidth[C010], D.actualWidth[C100], D.actualWidth[C110]);
    const bool allLeaves = (D.isLeaf[C010] & D.isLeaf[C100] & D.isLeaf[C110]);
    if (isCoarser(maxWidth, C)) {
      /* at least one is coarser. find coarsest, and defer to it */
      needToFillFrom[C110] = C;
      // check if C010 is closer
      if (isCoarser(D.actualWidth[C010], needToFillFrom[C110])) {
        needToFillFrom[C110].pos =
            make_vec3f(O.center.x,
                      O.center.y + (0.5f + delta) * C.width * O.signs.y,
                      O.center.z);
        needToFillFrom[C110].width = D.actualWidth[C010];
      }
      // check if C100 is closer
      if (isCoarser(D.actualWidth[C100], needToFillFrom[C110])) {
        needToFillFrom[C110].pos =
            make_vec3f(O.center.x,
                      O.center.y,
                      O.center.z + (0.5f + delta) * C.width * O.signs.z);
        needToFillFrom[C110].width = D.actualWidth[C100];
      }
      // check if C110 is closer
      if (isCoarser(D.actualWidth[C110], needToFillFrom[C110])) {
        needToFillFrom[C110].pos =
            make_vec3f(O.center.x,
                       O.center.y + (0.5f + delta) * C.width * O.signs.y,
                       O.center.z + (0.5f + delta) * C.width * O.signs.z);
        needToFillFrom[C110].width = D.actualWidth[C110];
      }
      done[C110] = false;
    } else if (!allLeaves) {
    /*! WE are the coarser one - use fill method */
#if EDGE
      O.value[C110] = coarseBoundaryValue(self, make_vec3f(O.center.x, O.vertex.y, O.vertex.z), C.width);
#else
      O.value[C110] = stitchCoarserEdge(self,C,O,D,C110);
  
#endif

      done[C110]   = true;
      coarseFilled = true;
    } else {
      O.value[C110] =
          0.25f * (C.value + D.value[C010] + D.value[C100] + D.value[C110]);
      done[C110] = true;
    }
  }

  /* ###################### VERTEX ###################### */
  /* the vertex touches all seven neighbors. if all are on the same
     level, then we aren't on a bounary and can average; if not, but
     at least one is coarser, we have find the coarSEST neighbor and
     defer to him; if neither of those two cases applies we're on a
     boundary but are the coarsest, so can backfill */

  /* ----------- vertex ----------- */ {
    const float maxWidth = max(D.actualWidth[0],
                               D.actualWidth[1],
                               D.actualWidth[2],
                               D.actualWidth[3],
                               D.actualWidth[4],
                               D.actualWidth[5],
                               D.actualWidth[6],
                               D.actualWidth[7]);
    const bool allLeaves =
        (D.isLeaf[0] & D.isLeaf[1] & D.isLeaf[2] & D.isLeaf[3] & D.isLeaf[4] &
         D.isLeaf[5] & D.isLeaf[6] & D.isLeaf[7]);
    if ((maxWidth == C.width) && allLeaves) {
      /* all on same level. average, and done */
      O.value[C111] =
          0.125f * (D.value[0] + D.value[1] + D.value[2] + D.value[3] +
                    D.value[4] + D.value[5] + D.value[6] + D.value[7]);
      done[C111] = true;
    } else if (isCoarser(maxWidth, C)) {
      /* at least one is coarser - find it, and fill from that neighbor */
      needToFillFrom[C111] = C;
      for (uniform int cID = 1; cID < 8; cID++) {
        if (isCoarser(D.actualWidth[cID], needToFillFrom[C111])) {
          needToFillFrom[C111].pos.x =
              (cID & 1) ? O.center.x + (0.5f + delta) * C.width * O.signs.x
                        : O.center.x;
          needToFillFrom[C111].pos.y =
              (cID & 2) ? O.center.y + (0.5f + delta) * C.width * O.signs.y
                        : O.center.y;
          needToFillFrom[C111].pos.z =
              (cID & 4) ? O.center.z + (0.5f + delta) * C.width * O.signs.z
                        : O.center.z;
          needToFillFrom[C111].width = D.actualWidth[cID];
        }
      }
      done[C111] = false;
    } else {
      /* none is coarser, but at least one is finer. boundary fill this vertex
       */
#if VERTEX
      O.value[C111] = coarseBoundaryValue(
          self, make_vec3f(O.vertex.x, O.vertex.y, O.vertex.z), C.width);
#else
      // print("==========================\n");
      // print("pos:[%,%,%]\n",P.x, P.y, P.z);
      O.value[C111] =stitchCoarserVertex(self,C,O,D,P);
#endif
      done[C111]   = true;
      coarseFilled = true;
    }
  }

  for (uniform int ii = 0; ii < 8; ii++) {
    if (done[ii])
      continue;
    const vec3f vtxPos = make_vec3f((ii & 1) ? O.vertex.x : O.center.x,
                                    (ii & 2) ? O.vertex.y : O.center.y,
                                    (ii & 4) ? O.vertex.z : O.center.z);
    // this isn't actually necessary: in theory we already KNOW this
    // cell from the dual cell. for now, do the actual findcell again,
    // just to make sure we have all the right values initialized
    const CellRef fillFrom =
        findLeafCell(self->_voxelAccel, needToFillFrom[ii].pos);
    O.value[ii] = doTrilinear(self, fillFrom, vtxPos);
    done[ii]    = true;
  }

  // for (uniform int ii = 0; ii < 8; ii++) {
  //   PRINT(O.value[ii]);
  // }


  return lerp(O);
}

varying float TAMR_Trilinear(const void *uniform _self, const varying vec3f &P)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;

  vec3f lP;  // local amr space
  self->transformWorldToLocal(self, P, lP);

  const CellRef cell = findLeafCell(self->_voxelAccel, lP);
  return doTrilinear(_self, cell, lP);
}

// Functions that defined in CPP files
extern "C" unmasked uniform float P4est_scalar_sample(
    void *uniform cppObject, uniform vec3f *uniform samplePos);

extern "C" unmasked void P4est_scalar_batch_sample(
    void *uniform cppObject,
    uniform vec3f *uniform posBuffer,
    uniform int numActivePos,
    uniform int *uniform activeIDs,
    uniform float *uniform values);

// TODO gradient shading is not working
extern "C" unmasked uniform vec3f P4est_scalar_computeGradient(
    void *uniform cppObject, uniform vec3f *uniform samplePos);

// Sample the P4estVolume at the world space coordinates
varying float P4estVolume_sample(const void *uniform _self,
                                 const varying vec3f &worldCoordinates)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;

  if (!box_contains(self->_voxelAccel._bounds, worldCoordinates))
    return -1.f;

#if 1
  // ISPC version voxel octree sampling, batch query

  // float result = TAMR_nearest(_self,worldCoordinates);
  // float result = TAMR_current(_self,worldCoordinates);
  // float result = TAMR_finest(_self,worldCoordinates);
  // float result = TAMR_octant(_self,worldCoordinates);
  float result = TAMR_Trilinear(_self, worldCoordinates);
  return result;

#elif 1
  // ISPC version voxel octree sampling, travese 8 times for a given point

  // vec3f lP;
  // self->transformWorldToLocal(self,worldCoordinates,lP);
  // CellRef cell = findLeafCell(self->_voxelAccel,lP);
  // return cell.value;

  vec3f lP;
  self->transformWorldToLocal(self, worldCoordinates, lP);

  CellRef cell = findLeafCell(self->_voxelAccel, lP);
  DualCell dCell;
  initDualCell(dCell, lP, cell.width);

  const vec3f _P0 =
      clamp(dCell.pos, make_vec3f(0.f), self->_voxelAccel._bounds.upper);
  const vec3f _P1 = clamp(dCell.pos + dCell.width,
                          make_vec3f(0.f),
                          self->_voxelAccel._bounds.upper);

  vec3f conners[8] = {_P0,
                      make_vec3f(_P1.x, _P0.y, _P0.z),
                      make_vec3f(_P0.x, _P1.y, _P0.z),
                      make_vec3f(_P1.x, _P1.y, _P0.z),
                      make_vec3f(_P0.x, _P0.y, _P1.z),
                      make_vec3f(_P1.x, _P0.y, _P1.z),
                      make_vec3f(_P0.x, _P1.y, _P1.z),
                      _P1};

  for (uniform int i = 0; i < 8; i++) {
    cell = findLeafCell(self->_voxelAccel, conners[i]);
    dCell.value[i] = cell.value;
    dCell.actualWidth[i] = cell.width;
    dCell.isLeaf[i] == (dCell.width == cell.width);
  }

  return lerp(dCell);

#elif 1
  // C++ version voxel octree sampling

  uniform vec3f uSamplePos[programCount];
  uniform float uReturnValue[programCount];

  vec3f lP;
  self->transformWorldToLocal(self, worldCoordinates, lP);

  uSamplePos[programIndex] = lP;
  foreach_active(lane)
  {
    uReturnValue[lane] =
        P4est_scalar_sample(self->cppSampler, &uSamplePos[lane]);
  }
  float result = uReturnValue[programIndex];
  return result;
#else
  // C++ version p4est travesal sampling.

  uniform vec3f uSamplePos[programCount];
  uniform float uReturnValue[programCount];

  uSamplePos[programIndex] = worldCoordinates;

  uniform int activeIDs[programCount];
  uniform int numActivePos = packed_store_active(activeIDs, programIndex);

  if (programIndex == 0) {
    // query num active values at a time
    P4est_scalar_batch_sample(
        self->cppSampler, uSamplePos, numActivePos, activeIDs, uReturnValue);
  }

  float result;
  if (isnan(uReturnValue[programIndex]))
    result = 0.0;
  else
    result = uReturnValue[programIndex];
  return result;

#endif
}

// Compute the gradient at the given sample location in world coordinates.
varying vec3f P4estVolume_computeGradient(const void *uniform _self,
                                          const varying vec3f &worldCoordinates)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;
  // TODO: This can be somewhat implemented by knowing the grid size and
  // sampling different points at +/-x,y,z then computing a central difference
  // of each
  print("P4estVolume_computeGradient NOT IMPLEMENTED!\n");
}

// Find the next sample point in the volume and advance the ray to it
void P4estVolume_stepRay(const void *uniform _self,
                         varying Ray &ray,
                         const varying float samplingRate)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;
  const float len =
      length(self->super.boundingBox.upper - self->super.boundingBox.lower);
  // TODO: This must adapt to the size of the underlying grid at the AMR level
  // the ray is contained in so we don't miss features. Maybe intersect the
  // octree grid? Right now this basically assumes a uniform grid
  // The step size we choose for the highest quality should then be scaled by
  // the desired sampling rate, which will allow for downsampling to remain
  // interactive For now, just take ~20 samples through the box, adjusted by the
  // sampling rate
  const float stepSize = len / (20.f * samplingRate);
  ray.t0 += stepSize;
  ray.time = stepSize;
}

// Find the ray-isosurface intersection in the volume for the passed ray and
// set of isovalues to display
void P4estVolume_intersectIsosurface(
    const void *uniform _self,
    uniform float *uniform isovalues,
    uniform int numIsovalues,
    uniform size_t geomID,
    // ray.u contains ray-space intersection error, i.e. the epsilon
    varying Ray &ray)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;
  print("P4estVolume_intersectIsosurface NOT IMPLEMENTED!\n");
}

export void *uniform P4estVolume_createISPCEquivalent(void *uniform cppEquiv)
{
  uniform P4estVolume *uniform v = uniform new uniform P4estVolume;
  Volume_Constructor(&v->super, cppEquiv);

  v->p4estTree      = NULL;
  v->p4estTreeBytes = 0;

  // Setup the parent Volume
  v->super.cppEquivalent       = cppEquiv;
  v->super.sample              = P4estVolume_sample;
  v->super.computeGradient     = P4estVolume_computeGradient;
  v->super.stepRay             = P4estVolume_stepRay;
  v->super.intersectIsosurface = P4estVolume_intersectIsosurface;
  return v;
}

export void P4estVolume_freeVolume(void *uniform _self)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;
  delete self;
}

inline void P4estVolume_transformLocalToWorld(const uniform P4estVolume *uniform
                                                  volume,
                                              const varying vec3f &localCoord,
                                              varying vec3f &worldCoord)
{
  worldCoord = volume->gridOrigin + localCoord * volume->gridSpacing;
}

inline void P4estVolume_transformWorldToLocal(const uniform P4estVolume *uniform
                                                  volume,
                                              const varying vec3f &worldCoord,
                                              varying vec3f &localCoord)
{
  localCoord = rcp(volume->gridSpacing) * (worldCoord - volume->gridOrigin);
}

// Matching extern declation to call into the C++ function
extern "C" void ispc_called_cpp_function_example(uniform int val);

export void P4estVolume_set(void *uniform _self,
                            void *uniform p4estTree,
                            uniform uint64 p4estTreeBytes,
                            uniform box3f *uniform bounds,
                            const uniform vec3i &dimensions,
                            const uniform vec3f &gridOrigin,
                            const uniform vec3f &gridSpacing,
                            /*! pointer to the c++ side object */
                            void *uniform cppObject,
                            void *uniform cppSampler)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;

  self->p4estTree      = p4estTree;
  self->p4estTreeBytes = p4estTreeBytes;

  self->dimensions  = dimensions;
  self->gridOrigin  = gridOrigin;
  self->gridSpacing = gridSpacing;

  // self->super.boundingBox = *bounds;
  self->super.boundingBox = make_box3f(bounds->lower * self->gridSpacing,
                                       bounds->upper * self->gridSpacing);

  self->cppObject  = cppObject;
  self->cppSampler = cppSampler;

  self->transformLocalToWorld = P4estVolume_transformLocalToWorld;
  self->transformWorldToLocal = P4estVolume_transformWorldToLocal;

  // TODO: This should be set in some way to be our minimum sampling step size,
  // so whatever our finest step should be within the smallest leaf cell in the
  // dataset?
  const uniform float len =
      length(self->super.boundingBox.upper - self->super.boundingBox.lower);
  self->super.samplingStep = len / 20.f;
}

export void P4estVolume_setVoxelOctree(void *uniform _self,
                                       void *uniform octreeNodes,
                                       uniform unsigned int64 oNodeNum,
                                       uniform box3f *uniform bounds,
                                       uniform box3f *uniform virtualBounds)
{
  uniform P4estVolume *uniform self =
      (uniform uniform P4estVolume * uniform) _self;

  uniform VoxelOctreeNode *uniform nodes =
      (uniform VoxelOctreeNode * uniform) octreeNodes;
  self->_voxelAccel._bounds        = *bounds;
  self->_voxelAccel._virtualBounds = *virtualBounds;
  self->_voxelAccel._octreeNodes   = nodes;
  self->_voxelAccel._oNodeNum      = oNodeNum;

  vec3f pos = make_vec3f(1.6, 1.5, 1.5);


  pos = make_vec3f(64.0, 24.0, 6.0);
  // print("pos:[%,%,%]\n",pos.x,pos.y,pos.z);
  CellRef cell = findLeafCell(self->_voxelAccel, pos);
  // PRINT(cell.width);

  // DualCell dcell;
  // initDualCell(dcell, pos, cell.width);
  // print("dual:C000:[%,%,%]\n",dcell.pos.x,dcell.pos.y,dcell.pos.z);
  // print("dual:C111:[%,%,%]\n",dcell.pos.x + dcell.width,dcell.pos.y+
  // dcell.width,dcell.pos.z+ dcell.width);

  // findDualCell(self->_voxelAccel,dcell);
  // print("dual::AcualWidth:[%,%,%,%,%,%,%,%]\n",
  //       dcell.actualWidth[0],dcell.actualWidth[1],
  //       dcell.actualWidth[2],dcell.actualWidth[3],
  //       dcell.actualWidth[4],dcell.actualWidth[5],
  //       dcell.actualWidth[6],dcell.actualWidth[7]);

  // print("dual:value:[%,%,%,%,%,%,%,%]\n",
  //         dcell.value[0],dcell.value[1],
  //         dcell.value[2],dcell.value[3],
  //         dcell.value[4],dcell.value[5],
  //         dcell.value[6],dcell.value[7]);

  doTrilinear(_self, cell, pos);

  // unsigned int8 test = 2 * programIndex;
  // int d= 3;
  // unsigned int a = (test +1) % d;
  // PRINT(a);

}
