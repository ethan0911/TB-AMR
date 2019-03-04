#pragma once

#include "ospcommon/vec.h"
#include "ospray/volume/Volume.h"
#include "ospray/common/Data.h"

using namespace ospcommon;


/*! abstract base class for any type of scalar volume sampler
* we will eventually specialize this for bricktree further below
*/
class ScalarVolumeSampler
{
public:
  /*! compute sample at given position */
  virtual float sample(const vec3f &pos) const = 0;

  /*! compute gradient at given position */
  virtual vec3f computeGradient(const vec3f &pos) const = 0;
};



class P4estVolume : public ospray::Volume {
public:
  P4estVolume();
  virtual ~P4estVolume() override;

  // Return a string description of this class.
  virtual std::string toString() const override;

  // Populate the volume and set it up based on the params set through
  // the OSPRay API
  virtual void commit() override;

  // Not supported on p4est volume, throws an error
  virtual int setRegion(const void *source_pointer,
                        const ospcommon::vec3i &target_index,
                        const ospcommon::vec3i &source_count) override;

  ScalarVolumeSampler *createSampler();

protected:
  // The raw p4est data buffer passed to use through the API for the octree
  ospray::Data *p4estTree;

  ScalarVolumeSampler *sampler;
};


class P4estVolumeSampler : public ScalarVolumeSampler
{
public:
  P4estVolumeSampler(P4estVolume *v):p4estv(v)
  {
  }

  virtual float sample(const vec3f &pos) const override
  {
    return 0.2f;
  }
  
  virtual vec3f computeGradient(const vec3f &pos) const override
  {
    return vec3f(1,0,0);
  }


private:
  P4estVolume *p4estv;
};

