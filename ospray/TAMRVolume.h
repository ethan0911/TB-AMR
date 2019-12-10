#pragma once

// #include "DataQueryCallBack.h"
#include "ospcommon/math/vec.h"
#include "ospcommon/utility/getEnvVar.h"
#include "ospray/common/Data.h"
#include "ospray/volume/Volume.h"

#include <limits>
#include "VoxelOctree.h"

using namespace ospcommon;

/*! abstract base class for any type of scalar volume sampler
 * we will eventually specialize this for bricktree further below
 */
class ScalarVolumeSampler
{
 public:
  virtual ~ScalarVolumeSampler() {}
  /*! compute sample at given position */
  virtual float sample(const vec3f &pos) const = 0;

  /*! compute gradient at given position */
  virtual vec3f computeGradient(const vec3f &pos) const = 0;
};

class TAMRVolume : public ospray::Volume
{
 public:
  TAMRVolume();
  TAMRVolume(VoxelOctree *voxelOctree);
  virtual ~TAMRVolume() override;

  // Return a string description of this class.
  virtual std::string toString() const override;

  // Populate the volume and set it up based on the params set through
  // the OSPRay API
  virtual void commit() override;

  ScalarVolumeSampler *createSampler();
  ScalarVolumeSampler *sampler;

  //! Volume size in voxels per dimension. e.g. (4 x 4 x 2)
  vec3i dimensions;
  //! Grid origin.
  vec3f gridOrigin;
  //! Grid spacing in each dimension in world coordinate.
  vec3f gridWorldSpace;

  // Feng's code to test the voxeloctree.
  VoxelOctree *_voxelAccel;
};

class TAMRVolumeSampler : public ScalarVolumeSampler
{
 public:
  TAMRVolumeSampler(TAMRVolume *v) : p4estv(v) {}

  // Please refer to p4est_search.h for reference.
  // We have access to the p4est_t structure via ospray::data *p4estTree
  // I will call this structure "p4est" in my comments / pseudocode
  virtual float sample(const vec3f &pos) const override
  {
    return (float)p4estv->_voxelAccel->queryData(pos);
  }

  virtual vec3f computeGradient(const vec3f &pos) const override
  {
    return vec3f(1, 0, 0);
  }

 private:
  TAMRVolume *p4estv;
};
