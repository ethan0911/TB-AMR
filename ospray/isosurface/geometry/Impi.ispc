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
#include "../../TAMRVolume.ih"

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
                               const World *uniform model,
                               varying DifferentialGeometry &dg,
                               const varying Ray &ray,
                               uniform int64 flags)
{
  const Impi *uniform self = (const Impi *uniform)geometry;
  dg.Ng = dg.Ns = normalize(ray.Ng);

  if (flags & DG_MATERIALID) {
    dg.materialID = -1;
    // dg.material   = self->super.materialList[0];
  }
  if (flags & DG_COLOR) {
    dg.color = self->isoColor;  // make_vec4f(1.0f,0.0f,0.0f,0.5f);
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
  Geometry_Constructor(&self->super,cppEquivalent,
                       Impi_postIntersect,
                       NULL,0,NULL);
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
  bbox.lower = make_vec3fa(0.f);
  bbox.upper = make_vec3fa(1.f);
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
  }
  return;
}

export void Impi_finalize(void *uniform _self,
                          void *uniform _model,
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
  // print("active voxel number: [%]\n", activeVoxelRefs[0]);
  
  // ... and let embree build a bvh, with 'numPatches' primitmives and
  // our function pointers for ray-prim interseciton and
  // prim-boundingbox computation callsbacks
  World *uniform model  = (World *uniform)_model;

  // create a new embree geometry with numpathces prims, in the model
  // that this goemetry is in.
  RTCGeometry geom = rtcNewGeometry(ispc_embreeDevice(),RTC_GEOMETRY_TYPE_USER);
  uniform uint32 geomID = rtcAttachGeometry(model->embreeSceneHandle,geom);
  self->super.geomID = geomID;

  // set 'us' as user data (this will be the first arg in intersect()
  // and computebounds() callbacks
  rtcSetGeometryUserData(geom, self);
  rtcSetGeometryUserPrimitiveCount(geom,numActiveVoxelRefs);

  rtcSetGeometryBoundsFunction(geom,(uniform RTCBoundsFunction)&Impi_bounds, self);
  rtcSetGeometryIntersectFunction(geom,(uniform RTCIntersectFunctionN)&Impi_intersect);
  rtcSetGeometryOccludedFunction(geom,(uniform RTCOccludedFunctionN)&Impi_intersect);
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);

}




unmasked extern "C" externC_ExatracVoxelValues(void *uniform _cVoxels,
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

unmasked extern "C" externC_ExatracActiveVoxels(void *uniform _activeVoxels,
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


export void getVoxelsValues(const void *uniform _self,
                            void *uniform inputVoxels,
                            void *uniform outputVoxelVRange,
                            void *uniform activeVoxels,
                            const uniform uint32 numVoxels,
                            const uniform float isoValue)
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

    //print("vb:(%,%,%,%,%,%\n)", lo.x,lo.y,lo.z,hi.x,hi.y,hi.z);
    float oV[8];
    vec2f oR = make_vec2f(1e20, -1e20);

    for (uint32 j = 0; j < 8; j++) {
      const varying vec3f P = conners[j];
      float value           = self->super.sample(_self, P);
      oR.x                  = min(oR.x, value);
      oR.y                  = max(oR.y, value);
      oV[j]                 = value;
      //print("vtx:(%f,%f,%f), value: %f\n", P.x,P.y,P.z,value);
    }
    //print("value:(%,%,%,%,%,%,%,%\n)", oV[0], oV[1], oV[2], oV[3],oV[4], oV[5], oV[6], oV[7]);
    //print("range:(%,%)\n", oR.x, oR.y);

    // PRINT(i);
    bool inRange = oR.x < isoValue && oR.y > isoValue;
    foreach_active(pid)
    {
      externC_ExatracVoxelValues(inputVoxels,
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
      // if (inRange) {
      //   externC_ExatracActiveVoxels(activeVoxels,
      //                               extract(oV[0], pid),
      //                               extract(oV[1], pid),
      //                               extract(oV[2], pid),
      //                               extract(oV[3], pid),
      //                               extract(oV[4], pid),
      //                               extract(oV[5], pid),
      //                               extract(oV[6], pid),
      //                               extract(oV[7], pid),
      //                               extract(lo.x, pid),
      //                               extract(lo.y, pid),
      //                               extract(lo.z, pid),
      //                               extract(hi.x, pid),
      //                               extract(hi.y, pid),
      //                               extract(hi.z, pid));
      // }
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



export void getVoxelValue2(const void *uniform _self,
                          void *uniform iVoxel,
                          void *uniform vRange)
{
  uniform TAMRVolume *uniform self = (uniform TAMRVolume * uniform) _self;
  uniform Voxel *voxel             = (uniform Voxel *)iVoxel;
  uniform vec2f *range             = (uniform vec2f *)vRange;
  const varying vec3f lo           = make_vec3f(voxel->bounds.lower);
  const varying vec3f hi           = make_vec3f(voxel->bounds.upper);
  const varying vec3f conners[8]   = {make_vec3f(lo.x, lo.y, lo.z),
                                    make_vec3f(hi.x, lo.y, lo.z),
                                    make_vec3f(lo.x, hi.y, lo.z),
                                    make_vec3f(hi.x, hi.y, lo.z),
                                    make_vec3f(lo.x, lo.y, hi.z),
                                    make_vec3f(hi.x, lo.y, hi.z),
                                    make_vec3f(lo.x, hi.y, hi.z),
                                    make_vec3f(hi.x, hi.y, hi.z)};

  vec2f oR = make_vec2f(1e20, -1e20);
  // foreach (i = 0 ... 8)
  for (uint32 i = 0; i < 8; i++) {
    const varying vec3f P = conners[i];
    float value           = self->super.sample(_self, P);
    oR.x                  = min(oR.x, value);
    oR.y                  = max(oR.y, value);
    voxel->vtx[(i & 4) ? 1 : 0][(i & 2) ? 1 : 0][(i & 1) ? 1 : 0] = value;
  }
  range->x = oR.x;
  range->y = oR.y;
}