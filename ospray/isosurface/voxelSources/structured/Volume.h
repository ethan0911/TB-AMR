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

#include "ospcommon/array3D/Array3D.h"
#include "ospcommon/array3D/for_each.h"
#include "ospcommon/range.h"

namespace structured {

  using namespace ospcommon;

  typedef ospcommon::range_t<float> Range;

  /*! 21:21:21 encoding of a voxel inside a structured volume */
  struct VoxelRef
  {
    VoxelRef(){};
    VoxelRef(const uint64_t bits) : bits(bits) {}
    VoxelRef(const vec3i &idx) : x(idx.x), y(idx.y), z(idx.z) {}

    inline vec3i asVec3i() const
    {
      return vec3i(x, y, z);
    }

    union
    {
      uint64_t bits;
      struct
      {
        uint64_t x : 21;
        uint64_t y : 21;
        uint64_t z : 21;
      };
    };
  };

  /*! a voxel with 8 corner voxels */
  struct Voxel
  {
    float vtx[2][2][2];

    inline Range getRange() const
    {
      Range range;
      array3D::for_each(vec3i(2), [&](const vec3i idx) {
        range.extend(vtx[idx.z][idx.y][idx.x]);
      });
      return range;
    }
  };

  /*! defines basic logcial abstraction for a volume class we can
    build a iso-dat structure over. should _probably_ at some time
    get merged into some more ospray/common like volume thingy */
  struct LogicalVolume
  {
    /*! return dimensions (in voxels, not voxels!) of the underlying volume */
    virtual vec3i getDims() const = 0;

    /* query get the eight corner voxels (in float) of given voxel */
    virtual void getVoxel(Voxel &voxel, const vec3i &voxelIdx) const = 0;

    /* query get the eight corner voxels (in float) of given voxel */
    inline Voxel getVoxel(const vec3i &idx) const
    {
      Voxel c;
      getVoxel(c, idx);
      return c;
    }

    /*! create a list of all the voxel references in [lower,upper)
      whose value range overlaps the given iso-value */
    virtual void filterVoxelsThatOverLapIsoValue(std::vector<VoxelRef> &out,
                                                 const vec3i &lower,
                                                 const vec3i &upper,
                                                 const float iso) const = 0;

    /*! build list of all voxels that fulfill the given filter lambda */
    template <typename Lambda>
    void filterVoxels(std::vector<VoxelRef> &out, Lambda filter);

    /*! create a list of *all* the voxel references in the entire volume
      whose value range overlaps the given iso-value */
    void filterAllVoxelsThatOverLapIsoValue(std::vector<VoxelRef> &out,
                                            const float iso) const;
  };

  /*! create a test data set of some blobs splatted into a volume of given size
   */
  std::shared_ptr<LogicalVolume> createTestVolume(const vec3i &dims);

  /*! defines an _actual_ implementation of a volume */
  template <typename T>
  struct VolumeT : public LogicalVolume, array3D::ActualArray3D<T>
  {
    /*! constructor */
    VolumeT(const vec3i &dims) : array3D::ActualArray3D<T>(dims) {}

    /*! return dimensions (in voxels, not voxels!) of the underlying volume */
    virtual vec3i getDims() const override
    {
      return this->size();
    }

    static std::shared_ptr<LogicalVolume> loadRAW(const std::string fileName,
                                                  const vec3i &dims);

    inline Range getRangeOfVoxel(const vec3i &voxelIdx) const
    {
      Voxel c;
      getVoxel(c, voxelIdx);
      return c.getRange();
    }

    /* query get the eight corner voxels (in float) of given voxel */
    virtual void getVoxel(Voxel &voxel, const vec3i &voxelIdx) const override
    {
      array3D::for_each(vec3i(2), [&](const vec3i vtxIdx) {
        voxel.vtx[vtxIdx.z][vtxIdx.y][vtxIdx.x] = this->get(voxelIdx + vtxIdx);
      });
    }

    /*! create a list of all the voxel references in [lower,upper)
      whose value range overlaps the given iso-value */
    virtual void filterVoxelsThatOverLapIsoValue(std::vector<VoxelRef> &out,
                                                 const vec3i &lower,
                                                 const vec3i &upper,
                                                 const float iso) const override
    {
      array3D::for_each(lower, upper, [&](const vec3i &idx) {
        if (this->getRangeOfVoxel(idx).contains(iso))
          out.push_back(VoxelRef(idx));
      });
    }
  };

  /*! build list of all voxels that fulfill the given filter lambda */
  template <typename Lambda>
  inline void LogicalVolume::filterVoxels(std::vector<VoxelRef> &out,
                                          Lambda filter)
  {
    /*! for now, do this single-threaded: \todo use tasksys ... */
    out.clear();
    array3D::for_each(getDims(), [&](const vec3i &idx) {
      if (filter(this, idx))
        out.push_back(VoxelRef(idx));
    });
  }

}  // namespace structured
