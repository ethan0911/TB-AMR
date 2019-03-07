#pragma once

#include "ospcommon/vec.h"
#include "ospray/volume/Volume.h"
#include "ospray/common/Data.h"

#include <p4est_to_p8est.h>

/* p4est has two separate interfaces for 2D and 3D, p4est*.h and p8est*.h.
 * Most API functions are available for both dimensions.  The header file
 * p4est_to_p8est.h #define's the 2D names to the 3D names such that most code
 * only needs to be written once.  In this example, we rely on this. */
#ifndef P4_TO_P8
#include <p4est_extended.h>
#include <p4est_ospray.h>
#else
#include <p8est_extended.h>
#include <p8est_ospray.h>
#endif


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
  //ospray::Data *p4estTree;

  ScalarVolumeSampler *sampler;

  //p4est tree handler
  p4est_t            *p4est;
};


class P4estVolumeSampler : public ScalarVolumeSampler
{
public:
  P4estVolumeSampler(P4estVolume *v):p4estv(v)
  {
  }

/*
 *  int search_and_interp(p4est_t * p4est,
 *                        p4est_topidx_t which_tree,
 *                        p4est_quadrant_t * quadrant,
 *                        p4est_locidx_t local_num,
 *                        void *point){
 *
 *    //Return 0 if *point is ouside quadrant
 *    
 *    if(local_num == -1){ //we are at a non-leaf node
 *      //do nothing
 *    } else{ //we are at a leaf!
 *      //read the 8 data points out of quadrant->user_data 
 *      //do bilinear interpolation
 *      //store the iterpolated value inside *(p4est->user_pointer) 
 *    }
 *  }
 */

  //Please refer to p4est_search.h for reference.
  //
  //We have access to the p4est_t structure via ospray::data *p4estTree
  //I will call this structure "p4est" in my comments / pseudocode
  virtual float sample(const vec3f &pos) const override
  {
    //float interpolated_value; //allocate space for the output of our search on the stack.
    //p4est->user_pointer = &interpolated_value;
    
    //Create an sc_array of size 1. The single member of this array is the position "pos". 
    
    //call p4est_search
    
    //Recall that in the search_point_fn callback, we wrote our result into the stack variable at address p4est->user_pointer
    //return *(p4est->user_pointer); 

    // //WILL SHALLOW COPY P4EST STRUCUTURE
    // p4est_t tmp = *v->p4est;
    // // now use tmp to swap user pointer
    // double result = 0;
    // tmp->userPointer = &result;
    // // now call the thing to search


    return 0.2f; //For test purposes only
  }
  
  virtual vec3f computeGradient(const vec3f &pos) const override
  {
    return vec3f(1,0,0);
  }


private:
  P4estVolume *p4estv;
};

