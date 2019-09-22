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

#pragma once

/*! \file ospray/geometry/Impi.h Defines a new ospray
  geometry type (of name 'bilinear_patches'). Input to the geometry is
  a single data array (named 'patches') that consists of for vec3fs
  per patch. */

// ospcomon: vec3f, box3f, etcpp - generic helper stuff
#include <ospcommon/math/box.h>
#include <ospcommon/math/vec.h>
// ospray: everything that's related to the ospray ray tracing core
#include "ospray/geometry/Geometry.h"
#include "ospray/common/World.h"
#include "../../VoxelOctree.h"
#include "../../TAMRVolume.h"

using namespace ospray;
// import ospcommon component - vec3f etc
using namespace ospcommon;

/*! a geometry type that implements implicit iso-surfaces within
  3D, trilinearly interpolated voxels. _where_ these voxels come
  from is completely abstracted in this class, so it can be
  instantiated with multiple 'voxel sources' (such as, for
  example, a plain list of voxels to consider, or a list of active
  voxels being extracted on-the-fly from a structued volume,
  etc */
struct Impi : public ospray::Geometry
{
  /*! a voxel made up of 8 floating point corner values, and the world
space bounding box of this voxel */
  struct Voxel
  {
    float vtx[2][2][2];
    box3fa bounds;
  };

  /*! interace that abstracts where the Impi is getting its voxels from*/
  struct VoxelSource
  {
    typedef uint64_t VoxelRef;

    /*! create lits of *all* voxel (refs) we want to be considered for interesction */
    virtual void getActiveVoxels(std::vector<VoxelRef> &activeVoxels,
                                 float isoValue) const = 0;

    /*! compute world-space bounds for given voxel */
    virtual box3fa getVoxelBounds(const VoxelRef voxelRef) const = 0;

    /*! get full voxel - bounds and vertex values - for given voxel */
    virtual Impi::Voxel getVoxel(const VoxelRef voxelRef) const = 0;
  };

  /*! constructor - will create the 'ispc equivalent' */
  Impi();

  /*! destructor - supposed to clean up all alloced memory */
  virtual ~Impi() override;

  /*! the commit() message that gets called upon the app calling
      "ospCommit(<thisGeometry>)" */
  virtual void commit() override;

  /*! create voxel source from whatever parameters we have been passed */
  void initVoxelSourceAndIsoValue();

  LiveGeometry createEmbreeGeometry() override;

  size_t numPrimitives() const override;

  /*! list of all active voxel references we are supposed to build the BVH over */
  std::vector<VoxelSource::VoxelRef> activeVoxelRefs;

  /*! the voxelsource that generates the actal voxels we need to intersect*/
  std::shared_ptr<VoxelSource> voxelSource;

  /*! the isovalue we're intersecting with */
  float isoValue;
  float lastIsoValue;
  vec4f isoColor;
};
