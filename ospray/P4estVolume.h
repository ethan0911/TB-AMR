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

class P4estVolume;

struct SharedP4estContext{
  P4estVolume* p4estVolume;
  double data;

  SharedP4estContext(P4estVolume* p4estv, double _data):p4estVolume(p4estv), data(_data){}
};

/*! abstract base class for any type of scalar volume sampler
* we will eventually specialize this for bricktree further below
* TODO: Don't think we really need this, maybe to combined with the hybrid
* isosurface rendering in the future?
*/
class ScalarVolumeSampler
{
public:
  virtual ~ScalarVolumeSampler(){}
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

  //Our local shallow copy... I believe this is "renderer.osp_p4est" in the pseudocode
  //p4est_t            local;

  //FIXME: p4eset_ospray_data_t is not defined? Need to edit p4est_to_p8est.h ?  
  p8est_ospray_data_t data_callback;
};

int pt_search_callback(p4est_t * p4est,
                       p4est_topidx_t which_tree,
                       p4est_quadrant_t * quadrant,
                       p4est_locidx_t local_num,
                       void *point){

  //ASSUME that our point is a length 3 / length 2 array of doubles.
  vec3f *pt = (vec3f*)point;
  double aabb[6];
  double *lower_corner = &aabb[0];
  double *upper_corner = &aabb[3];
  
  //Pseudocode: "renderer = p4est->user_pointer". Where renderer is some handle to our ospray module's state. 
  //P4estVolume * p4estv = (P4estVolume *) p4est->user_pointer;
  SharedP4estContext * sContext = (SharedP4estContext *)p4est->user_pointer;
  P4estVolume * p4estv = sContext->p4estVolume;

  p4est_ospray_quadrant_aabb (p4estv->p4est, which_tree, quadrant, aabb);

  double query_point[3];
  query_point[0] = (double)pt->x;
  query_point[1] = (double)pt->y;
  query_point[2] = (double)pt->z;

  //test if the point located in the aabb
	if ( pt->x < lower_corner[0] ||  pt->x > upper_corner[0]
			|| pt->y < lower_corner[1] ||  pt->y > upper_corner[1]
#ifdef P4_TO_P8
			|| pt->z < lower_corner[2] ||  pt->z > upper_corner[2]
#endif
	) {
		return 0;	//outside, tell p4est to terminate traversal
	} else { //point may be contained in the octant/quadrant 
    if(local_num >= 0){ 
      /* TODO: give the callback a point as const double xyz[3] (transform pt)
               note we *always* need 3 entries of xyz, even in 2D. */
      p4estv->data_callback (p4estv->p4est, which_tree, quadrant,
                              query_point, &sContext->data);
//                             query_point, &p4estv->data_result);
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

  //Please refer to p4est_search.h for reference.
  //We have access to the p4est_t structure via ospray::data *p4estTree
  //I will call this structure "p4est" in my comments / pseudocode
  virtual float sample(const vec3f &pos) const override
  {
    SharedP4estContext sP4estContext(p4estv, 0.0);

    p4est_t local = *p4estv->p4est;
    local.user_pointer = (void *)(&sP4estContext);

    sc_array_t search_pt_array;
    search_pt_array.elem_size = 3*sizeof(float); 
    search_pt_array.elem_count = 1;
    search_pt_array.array = (char*)&pos;
    // TODO put the tree ID in here
    p4est_ospray_search_local(&local, 0, 0, NULL, pt_search_callback, &search_pt_array);
    //PING;

    /* TODO: make sure the result is thread-safe (multiple buffers, one per tid) */
    return (float)sP4estContext.data;
  }
  
  virtual vec3f computeGradient(const vec3f &pos) const override
  {
    return vec3f(1,0,0);
  }


private:
  P4estVolume *p4estv;
};

