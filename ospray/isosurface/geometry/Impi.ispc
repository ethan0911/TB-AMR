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

// ospray
#include "math/vec.ih"
#include "math/box.ih"
#include "math/LinearSpace.ih"
#include "common/Ray.ih"
#include "common/World.ih"
#include "geometry/Geometry.ih"
// embree
#include "embree3/rtcore.isph"

#include "Voxel.ih"
#include "../../octant_stitch.ih"


enum FILTER_METHOD{NEAREST = 0, CURRENT, FINEST, OCTANT, TRILINEAR};

/*! ISCP side code of the impi module: this code doesn't know anything
    specific about the underlying "thing" that defines voxels/voxels -
    it only gets a list of voxel "references" (each on ebeing a 64-bit
    integer), and call-back function (to the C side) to query the
    bounds and/or voxel values of said voxels. 

    All the actual implemntation of what a voxel looks like - what
    format or voxel type, where they come from (explicitly from a list
    of voxels extracted with another tool, or implicitly in a 3D
    strcutured volume, etc), lies on the C side - this code ONLY
    builds a bvh over the voxel refs, and intersects them as required
    during traversal */


/*! function to get the 3D world bounds of a given voxel reference,
  to be computed on the C++ side */
unmasked extern "C" externC_getVoxelBounds(uniform box3fa &voxelBounds,
                                           void *uniform c_self,
                                           const uniform uint64 voxelRef);

/*! function to get the corner voxel values of a given voxel (referend
    with given voxelRef), to be computed on the C++ side */
unmasked extern "C" externC_getVoxel(uniform Voxel &voxel,
                                     void *uniform c_self,
                                     const uniform uint64 voxelRef);


struct Impi {
  /*! inherit from "Geometry" class: since ISPC doesn't support
      inheritance we simply put the "parent" class as the first
      member; this way any typecast to the parent class will get the
      right members (including 'virtual' function pointers, etc) */
  Geometry   super;

  float isoValue;
  vec4f isoColor;
  Voxel *voxelArray;
  
  /*! for the case where we build an embree bvh over the hot voxels,
      this is the list of all voxels that are hot (each one is one prim
      in the embree BVH */
  uint64 *uniform activeVoxelRefs;

  /*! for the embree bvh over active voxels case this is the c-handle to
      the c-side volume. this is _probably_ a C++-side virtual class
      that implements getvoxelbounds and getvoxel */
  void *uniform c_self;

  /*! todo - add getVoxel and getVoxelBounds as member function pointers
      (and let c++ side pass them on constructor), rather than as
      global functions */
};

static void Impi_postIntersect(const Geometry *uniform geometry,
                               varying DifferentialGeometry &dg,
                               const varying Ray &ray,
                               uniform int64 flags)
{
  const Impi *uniform self = (const Impi *uniform)geometry;
  dg.Ng = dg.Ns = normalize(ray.Ng);
  if ((flags & DG_FACEFORWARD) && dot(dg.Ns, ray.dir) > 0.f) {
    dg.Ns = neg(dg.Ns);
  }
}

/*! create a new ispc equivalent for the C++ class (cppEquivalent
    points back, if we ever need this. this returns a void * that the
    c++ side can use to call back to us when needed. Note this return
    values _has_ to be a untyped 'void*' because ISPC cannot currently
    export 'real' class types - with function pointers etcpp - to a c
    header file */
export void *uniform Impi_create(void *uniform cppEquivalent)
{
  Impi *uniform self = uniform new uniform Impi;
  
  /* recursively call the 'inherited' constructor, and pass our
     'overridden' functions for intersect and postintersect. note we
     do not have to set the 'intersect' and 'occluded' functions
     because we use embree to do the traversal for us (see
     Impi_set()) below */
  Geometry_Constructor(&self->super,
                       cppEquivalent,
                       Impi_postIntersect);
  return self;
}

export void Impi_destroy(void *uniform _self)
{
  /* _actually_ this should also destroy the created embree geometry
     here; not doing this to keep the code as small as possible, but
     usually this is where it should be done */
  Impi *uniform self = (Impi *uniform)_self;
  delete self;
}


/*! to be able to have embree build a data structure over us we have
    to be able to tell it the bounding box for each primitimve. this
    is the callback embree calls for querying the (uniform) bounding
    box for a single given primitmive 'primID' */
unmasked void Impi_bounds_testVoxel(Impi *uniform self,
                                   uniform size_t primID,
                                   uniform box3fa &bbox)
{
  bbox.lower = make_vec3f(0.f);
  bbox.upper = make_vec3f(1.f);
}


/*! embree callback to compute voxel bounds - we simply pass that on to
    the C side (yes, might eventually do this completely on the
    C-side, without first going through this callback, too.... but
    don't (yet) want to deal with having to have some callbacks like
    on the C side (getbounds) and others (intersectVoxel) on the ISPC
    side) */
unmasked void Impi_bounds(const RTCBoundsFunctionArguments *uniform args)
{
  uniform Impi *uniform self = (uniform Impi *uniform)args->geometryUserPtr;
  uniform int primID = args->primID;

  box3fa *uniform out = (box3fa *uniform)args->bounds_o;
  externC_getVoxelBounds(*out,self->c_self,self->activeVoxelRefs[primID]);
}




/*! intersect callback for the 'embree over hot voxels' method */
unmasked void Impi_intersect(const struct RTCIntersectFunctionNArguments *uniform args)
{
  uniform Impi *uniform self = (uniform Impi *uniform)args->geometryUserPtr;
  uniform int primID = args->primID;

  uniform Voxel  voxel;
  externC_getVoxel(voxel,self->c_self,self->activeVoxelRefs[primID]);
  // PRINT(voxel.vtx[0][0][1]);

    // this assumes that the args->rayhit is actually a pointer toa varying ray!
  varying Ray *uniform ray = (varying Ray *uniform)args->rayhit;

  if (actualVoxelIntersect(*ray,voxel,self->isoValue)) {
    ray->geomID = self->super.geomID;
    ray->primID = primID;
    ray->instID = args->context->instID[0];
  }
}

unmasked void Impi_occluded(
    const struct RTCOccludedFunctionNArguments *uniform args)
{
  uniform Impi *uniform self = (uniform Impi *uniform)args->geometryUserPtr;
  uniform int primID = args->primID;

  uniform Voxel  voxel;
  externC_getVoxel(voxel,self->c_self,self->activeVoxelRefs[primID]);
  // PRINT(voxel.vtx[0][0][1]);

  // this assumes that the args->rayhit is actually a pointer toa varying ray!
  varying Ray *uniform ray = (varying Ray *uniform)args->ray;

  if (actualVoxelIntersect(*ray,voxel,self->isoValue)) {
    ray->t = neg_inf;
  }
}

export void Impi_finalize(void *uniform _self,
                          void *uniform _embreeGeometry,
                          uint64 *uniform activeVoxelRefs,
                          uint64 uniform numActiveVoxelRefs,
                          void *uniform c_self,
                          uniform float isoValue,
                          uniform vec4f *uniform isoColor)
{
  // first, typecast to our 'real' type. since ispc can't export real
  // types to c we have to pass 'self' in as a void*, and typecast
  // here.
  Impi *uniform self = (Impi *uniform)_self;
  
  // set our internal data.
  self->isoValue   = isoValue;
  self->activeVoxelRefs = activeVoxelRefs;
  self->c_self      = c_self;
  self->isoColor = *isoColor;
  self->super.numPrimitives = numActiveVoxelRefs;
  
  // set 'us' as user data (this will be the first arg in intersect()
  // and computebounds() callbacks
  RTCGeometry geom = (RTCGeometry)_embreeGeometry;
  Geometry_setEmbreeUserGeometry(&self->super,
      geom,
      Impi_bounds,
      Impi_intersect,
      Impi_occluded);
}

unmasked extern "C" externC_ExatracAllVoxelValues(void *uniform _cVoxels,
                                               void *uniform _cVoxelRange,
                                               const uniform int index,
                                               const uniform float v0,
                                               const uniform float v1,
                                               const uniform float v2,
                                               const uniform float v3,
                                               const uniform float v4,
                                               const uniform float v5,
                                               const uniform float v6,
                                               const uniform float v7,
                                               const uniform float r1,
                                               const uniform float r2);

unmasked extern "C" externC_ExatracActiveVoxels(void *uniform C_actVoxels,
                                                const uniform float v0,
                                                const uniform float v1,
                                                const uniform float v2,
                                                const uniform float v3,
                                                const uniform float v4,
                                                const uniform float v5,
                                                const uniform float v6,
                                                const uniform float v7,                                           
                                                const uniform float b1,
                                                const uniform float b2,
                                                const uniform float b3,
                                                const uniform float b4,
                                                const uniform float b5,
                                                const uniform float b6);

unmasked extern "C" void externC_getPotentialOverlapVoxels(
    void *uniform C_potentialOverlapVoxelsIdx,
    const uniform unsigned int64 idx);

export void build_allVoxels(const void *uniform _self,
                            void *uniform inputVoxels,
                            void *uniform outputVoxelVRange,
                            const uniform uint32 numVoxels)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume * uniform) _self;
  uniform Voxel *voxels            = (uniform Voxel *)inputVoxels;

  foreach (i = 0 ... numVoxels) {
    const varying vec3f lo = make_vec3f(voxels[i].bounds.lower);
    const varying vec3f hi = make_vec3f(voxels[i].bounds.upper);

    const varying vec3f conners[8] = {make_vec3f(lo.x, lo.y, lo.z),
                                      make_vec3f(hi.x, lo.y, lo.z),
                                      make_vec3f(lo.x, hi.y, lo.z),
                                      make_vec3f(hi.x, hi.y, lo.z),
                                      make_vec3f(lo.x, lo.y, hi.z),
                                      make_vec3f(hi.x, lo.y, hi.z),
                                      make_vec3f(lo.x, hi.y, hi.z),
                                      make_vec3f(hi.x, hi.y, hi.z)};

    float oV[8];
    vec2f oR = make_vec2f(1e20, -1e20);

    for (uint32 j = 0; j < 8; j++) {
      const varying vec3f P = conners[j];
      float value           = self->super.sample(_self, P);
      oR.x                  = min(oR.x, value);
      oR.y                  = max(oR.y, value);
      oV[j]                 = value;

    }

    foreach_active(pid)
    {
      externC_ExatracAllVoxelValues(inputVoxels,
                                 outputVoxelVRange,
                                 extract(i, pid),
                                 extract(oV[0], pid),
                                 extract(oV[1], pid),
                                 extract(oV[2], pid),
                                 extract(oV[3], pid),
                                 extract(oV[4], pid),
                                 extract(oV[5], pid),
                                 extract(oV[6], pid),
                                 extract(oV[7], pid),
                                 extract(oR.x, pid),
                                 extract(oR.y, pid));
    }
  }
}

export void build_activeVoxels(const void *uniform _self,
                               void *uniform inputVoxels,
                               void *uniform C_activeVoxels,
                               const uniform uint32 numVoxels,
                               const uniform float isoValue)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume * uniform) _self;
  uniform Voxel *uniform voxels    = (uniform Voxel * uniform) inputVoxels;

  foreach (i = 0 ... numVoxels) {
    const varying vec3f lo     = make_vec3f(voxels[i].bounds.lower);
    const varying vec3f hi     = make_vec3f(voxels[i].bounds.upper);
    const varying float cW     = hi.x - lo.x;
    const varying float halfCW = 0.5 * cW;

    for (uniform uint32 j = 0; j < 8; j++) {
      varying vec3f childLo =
          lo + halfCW * make_vec3f(j & 1 ? 1 : 0, j & 2 ? 1 : 0, j & 4 ? 1 : 0);
      varying vec3f childHi = childLo + make_vec3f(halfCW);

      const varying vec3f conners[8] = {
          make_vec3f(childLo.x, childLo.y, childLo.z),
          make_vec3f(childHi.x, childLo.y, childLo.z),
          make_vec3f(childLo.x, childHi.y, childLo.z),
          make_vec3f(childHi.x, childHi.y, childLo.z),
          make_vec3f(childLo.x, childLo.y, childHi.z),
          make_vec3f(childHi.x, childLo.y, childHi.z),
          make_vec3f(childLo.x, childHi.y, childHi.z),
          make_vec3f(childHi.x, childHi.y, childHi.z)};
      float oV[8];
      vec2f oR = make_vec2f(1e20, -1e20);

      for (uniform uint32 k = 0; k < 8; k++) {
        const varying vec3f P = conners[k];
        float value           = self->super.sample(_self, P);
        oR.x                  = min(oR.x, value);
        oR.y                  = max(oR.y, value);
        oV[k]                 = value;
      }
      bool inRange = oR.x < isoValue && oR.y > isoValue;
      foreach_active(pid)
      {
        if (inRange) {
          externC_ExatracActiveVoxels(C_activeVoxels,
                                      extract(oV[0], pid),
                                      extract(oV[1], pid),
                                      extract(oV[2], pid),
                                      extract(oV[3], pid),
                                      extract(oV[4], pid),
                                      extract(oV[5], pid),
                                      extract(oV[6], pid),
                                      extract(oV[7], pid),
                                      extract(childLo.x, pid),
                                      extract(childLo.y, pid),
                                      extract(childLo.z, pid),
                                      extract(childHi.x, pid),
                                      extract(childHi.y, pid),
                                      extract(childHi.z, pid));
        }
      }
    }
  }
}

export void getVoxelValue(const void *uniform _self,
                          uniform Voxel *uniform iVoxel,
                          uniform vec2f *uniform vRange)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume * uniform) _self;

  const uniform vec3f lo = make_vec3f(iVoxel->bounds.lower);
  const uniform vec3f hi = make_vec3f(iVoxel->bounds.upper);

  const uniform vec3f conners[8] = {make_vec3f(lo.x, lo.y, lo.z),
                                    make_vec3f(hi.x, lo.y, lo.z),
                                    make_vec3f(lo.x, hi.y, lo.z),
                                    make_vec3f(hi.x, hi.y, lo.z),
                                    make_vec3f(lo.x, lo.y, hi.z),
                                    make_vec3f(hi.x, lo.y, hi.z),
                                    make_vec3f(lo.x, hi.y, hi.z),
                                    make_vec3f(hi.x, hi.y, hi.z)};

  uniform vec2f oR = make_vec2f(1e20, -1e20);

  foreach (i = 0 ... 8) 
  {
    const varying vec3f P = conners[i];
    float value           = self->super.sample(_self, P);

    foreach_unique(val in value)
    {
      oR.x = min(oR.x, val);
      oR.y = max(oR.y, val);
    }
    iVoxel->vtx[(i & 4) ? 1 : 0][(i & 2) ? 1 : 0][(i & 1) ? 1 : 0] = value;
  }

  vRange->x = oR.x;
  vRange->y = oR.y;
}


export void build_activeVoxel(const void *uniform _self,
                              const uniform vec3f &pos,
                              const uniform float value,
                              const uniform float localCellWidth,
                              const uniform uint32 refineFactor,
                              void *uniform C_activeVoxels,
                              const uniform float isoValue,
                              const uniform int filterMethod,
                              uniform bool* uniform isOverlapVoxelMask)
{
  const uniform TAMRVolume *uniform self = (uniform TAMRVolume * uniform) _self;
  uniform bool* uniform isOverlapVoxel = (uniform bool* uniform)isOverlapVoxelMask;

  vec3f lo = pos;
  vec3f lP;
  self->transformWorldToLocal(self, lo, lP);

  const varying box3f querybox = make_box3f(
      lP - make_vec3f(localCellWidth), lP + make_vec3f(localCellWidth * 2.f));

  // const varying box3f bbox        = self->_voxelAccel._virtualBounds;
  // const unsigned int64 rootNodeId = 0;
  // bool isOverlap = isOverlapIsoValue(
  //     self->_voxelAccel, rootNodeId, bbox, querybox, value, isoValue);


#if 1
  // toggle turn on the potential-overlapped voxel filter
  bool isOverlap =
      isPotentialOverlapVoxel(self->_voxelAccel, querybox, value, isoValue);

  if (!isOverlap) {
    *isOverlapVoxelMask = false;
    return;
  }
#endif
  
  *isOverlapVoxelMask = true;

  if (filterMethod == OCTANT || filterMethod == TRILINEAR) {

    float halfCW     = 0.5 * localCellWidth;
    vec3f cellCenter = lP + make_vec3f(halfCW);

    CellRef curCell;
    curCell.pos   = lP;
    curCell.value = value;
    curCell.width = localCellWidth;

    foreach (i = 0 ... 8) {
      vec3f oSign = make_vec3f(
          i & 1 ? 1.f : -1.f, i & 2 ? 1.f : -1.f, i & 4 ? 1.f : -1.f);
      vec3f tP = cellCenter + 0.5 * halfCW * oSign;

      Octant O;
      DualCell D;
      if (filterMethod == OCTANT)
        doOctant(self, curCell, tP, O, D);

      if (filterMethod == TRILINEAR)
        doTrilinear(self, curCell, tP, O, D);

      bool isInSameLvl = (D.actualWidth[0] == localCellWidth) &&
                         (D.actualWidth[1] == localCellWidth) &&
                         (D.actualWidth[2] == localCellWidth) &&
                         (D.actualWidth[3] == localCellWidth) &&
                         (D.actualWidth[4] == localCellWidth) &&
                         (D.actualWidth[5] == localCellWidth) &&
                         (D.actualWidth[6] == localCellWidth) &&
                         (D.actualWidth[7] == localCellWidth);

      box3f dBbox   = make_box3f(D.pos, D.pos + make_vec3f(D.width));
      bool inDomain = box_contains(self->_voxelAccel._actualBounds, dBbox);

      vec3f childLo, childHi;
      bool insertDual    = isInSameLvl && inDomain;
      vec3f localChildLo = insertDual ? D.pos
                                      : make_vec3f(min(O.center.x, O.vertex.x),
                                                   min(O.center.y, O.vertex.y),
                                                   min(O.center.z, O.vertex.z));
      vec3f localChildHi = insertDual ? D.pos + make_vec3f(D.width)
                                      : make_vec3f(max(O.center.x, O.vertex.x),
                                                   max(O.center.y, O.vertex.y),
                                                   max(O.center.z, O.vertex.z));

      self->transformLocalToWorld(self, localChildLo, childLo);
      self->transformLocalToWorld(self, localChildHi, childHi);

      const varying vec3f conners[8] = {
          make_vec3f(childLo.x, childLo.y, childLo.z),
          make_vec3f(childHi.x, childLo.y, childLo.z),
          make_vec3f(childLo.x, childHi.y, childLo.z),
          make_vec3f(childHi.x, childHi.y, childLo.z),
          make_vec3f(childLo.x, childLo.y, childHi.z),
          make_vec3f(childHi.x, childLo.y, childHi.z),
          make_vec3f(childLo.x, childHi.y, childHi.z),
          make_vec3f(childHi.x, childHi.y, childHi.z)};

      float oV[8];
      vec2f oR = make_vec2f(1e20, -1e20);

      for (uniform uint8 j = 0; j < 8; j++) {
        float value = 0.f;
        if (insertDual) {
          value = D.value[j];
        } 
        else if (inDomain) {
          uint8 octIdx = j;
          octIdx       = O.signs.x > 0 ? octIdx : octIdx ^ 1;
          octIdx       = O.signs.y > 0 ? octIdx : octIdx ^ 2;
          octIdx       = O.signs.z > 0 ? octIdx : octIdx ^ 4;
          value        = O.value[octIdx];
        } 
        else {
          value = self->super.sample(_self, conners[j]);
        }
        oR.x  = min(oR.x, value);
        oR.y  = max(oR.y, value);
        oV[j] = value;
      }

      bool inRange = oR.x < isoValue && oR.y > isoValue;

      foreach_active(pid)
      {
        if (inRange && (!insertDual || (insertDual && i == 7))) {
          externC_ExatracActiveVoxels(C_activeVoxels,
                                      extract(oV[0], pid),
                                      extract(oV[1], pid),
                                      extract(oV[2], pid),
                                      extract(oV[3], pid),
                                      extract(oV[4], pid),
                                      extract(oV[5], pid),
                                      extract(oV[6], pid),
                                      extract(oV[7], pid),
                                      extract(childLo.x, pid),
                                      extract(childLo.y, pid),
                                      extract(childLo.z, pid),
                                      extract(childHi.x, pid),
                                      extract(childHi.y, pid),
                                      extract(childHi.z, pid));
        }
      }
    }

  } else {
    // for nearst, current, finest method, we use the naive way that generating
    // 8 octants for each cells. 

    // const uniform vec3f lo        = pos;
    const uniform float cellWidth = localCellWidth * self->gridWorldSpace.x;
    const uniform vec3f hi        = pos + make_vec3f(cellWidth);

    const uniform uint32 childCellNums =
        refineFactor * refineFactor * refineFactor;
    const uniform float childCellWidth = cellWidth / refineFactor;

    foreach (i = 0 ... childCellNums) {
      int xIdx = (i % (refineFactor * refineFactor)) % refineFactor;
      int yIdx = (i % (refineFactor * refineFactor)) / refineFactor;
      int zIdx = i / (refineFactor * refineFactor);

      vec3f childLo = lo + childCellWidth * make_vec3f(xIdx, yIdx, zIdx);
      vec3f childHi = childLo + make_vec3f(childCellWidth);

      const varying vec3f conners[8] = {
          make_vec3f(childLo.x, childLo.y, childLo.z),
          make_vec3f(childHi.x, childLo.y, childLo.z),
          make_vec3f(childLo.x, childHi.y, childLo.z),
          make_vec3f(childHi.x, childHi.y, childLo.z),
          make_vec3f(childLo.x, childLo.y, childHi.z),
          make_vec3f(childHi.x, childLo.y, childHi.z),
          make_vec3f(childLo.x, childHi.y, childHi.z),
          make_vec3f(childHi.x, childHi.y, childHi.z)};

      float oV[8];
      vec2f oR = make_vec2f(1e20, -1e20);

      for (uniform uint32 j = 0; j < 8; j++) {
        const varying vec3f P = conners[j];
        float value           = self->super.sample(_self, P);
        oR.x                  = min(oR.x, value);
        oR.y                  = max(oR.y, value);
        oV[j]                 = value;
      }

      bool inRange = oR.x < isoValue && oR.y > isoValue;
      foreach_active(pid)
      {
        if (inRange) {
          externC_ExatracActiveVoxels(C_activeVoxels,
                                      extract(oV[0], pid),
                                      extract(oV[1], pid),
                                      extract(oV[2], pid),
                                      extract(oV[3], pid),
                                      extract(oV[4], pid),
                                      extract(oV[5], pid),
                                      extract(oV[6], pid),
                                      extract(oV[7], pid),
                                      extract(childLo.x, pid),
                                      extract(childLo.y, pid),
                                      extract(childLo.z, pid),
                                      extract(childHi.x, pid),
                                      extract(childHi.y, pid),
                                      extract(childHi.z, pid));
        }
      }
    }
  }
}
