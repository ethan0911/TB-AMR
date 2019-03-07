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
* TODO: Don't think we really need this, maybe to combined with the hybrid
* isosurface rendering in the future?
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

  // The raw p4est data buffer passed to use through the API for the octree
  //ospray::Data *p4estTree;

  ScalarVolumeSampler *sampler;

  //p4est tree handler
  p4est_t            *p4est;
};

int pt_search_callback(p4est_t * p4est,
                       p4est_topidx_t which_tree,
                       p4est_quadrant_t * quadrant,
                       p4est_locidx_t local_num,
                       void *point){
  //ASSUME that our point is a length 3 / length 2 array of doubles.
  vec3f *pt = (vec3f*)point;
  double lower_corner[3];
  double upper_corner[3];

  //obtain the upper and lower corner location of the octant/quadrant in world space
	p4est_qcoord_t	int_len = P4EST_QUADRANT_LEN (quadrant->level); //lengh in integer coords
  
  // TODO: Replace with the AABB fetching functions

  p4est_qcoord_to_vertex(p4est->connectivity,
                         which_tree,
                         quadrant->x,
                         quadrant->y,
#ifdef P4_TO_P8
                         quadrant->z,
#endif
                         lower_corner);

  p4est_qcoord_to_vertex(p4est->connectivity,
                         which_tree,
                         quadrant->x + int_len,
                         quadrant->y + int_len,
#ifdef P4_TO_P8
                         quadrant->z + int_len,
#endif
                         upper_corner);

  //assuming that the point is in world space, determine if the point is inside or outside the octant/quadrant
  // TODO Some potential round-off error here if we're sitting on the boundary
  // so track if we've already found a hit for this point.
	if ( pt->x < lower_corner[0] ||  pt->x > upper_corner[0]
			|| pt->y < lower_corner[1] ||  pt->y > upper_corner[1]
#ifdef P4_TO_P8
			|| pt->z < lower_corner[2] ||  pt->z > upper_corner[2]
#endif
	) {
		return 0;	//outside, tell p4est to terminate traversal
	} else { //point may be contained in the octant/quadrant 
    if(local_num >= 0){ //reached a leaf
      // TODO: This should take a local data access function pointer
      // Here, we would do bilinear interpolation of the data value
      // Right now I am just storing the coordinates of the quadrant, because in
      // our toy quadtree example there is no data
      // BOLD ASSUMPTION: the user pointer points to the head of an array of doubles with length at least 3.
      vec3f *result_out = (vec3f*)p4est->user_pointer;
      result_out->x = quadrant->x;
      result_out->y = quadrant->y;
#ifdef P4_TO_P8
      result_out->z = quadrant->z;
#endif
    }
    return 1; //tells p4est point may be contained in the octant/quadrant 
  }
}


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
    p4est_t local = *p4estv->p4est;
    vec3f result(0.f);
    local.user_pointer = (char*)&result;
    sc_array_t search_pt_array;
    search_pt_array.elem_size = 3*sizeof(float); 
    search_pt_array.elem_count = 1;
    search_pt_array.array = (char*)&pos;
    // TODO put the tree ID in here
    p4est_ospray_search_local(&local, 0, 0, NULL, pt_search_callback, &search_pt_array);
    //PING;
    return result[0];
  }
  
  virtual vec3f computeGradient(const vec3f &pos) const override
  {
    return vec3f(1,0,0);
  }


private:
  P4estVolume *p4estv;
};

