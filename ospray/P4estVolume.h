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

#include <limits>


using namespace ospcommon;

/*! abstract base class for any type of scalar volume sampler
* we will eventually specialize this for bricktree further below
*/
class ScalarVolumeSampler
{
public:
  virtual ~ScalarVolumeSampler(){}
  /*! compute sample at given position */
  virtual float sample(const vec3f &pos) const = 0;

  virtual void batch_sample(const vec3f* posBuffer,
                            const int numActivePos,
                            const int* activeIDs, 
                            float* values) const =0;

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

  // The tree ID of this tree within the total p4est
  int treeID;

  //FIXME: p4eset_ospray_data_t is not defined? Need to edit p4est_to_p8est.h ?  
  p8est_ospray_data_t data_callback;
};


struct P4estThreadContext {
  P4estVolume *volume;
  p4est_ospray_search_context_t *ctx;
  p4est_t local;
  double data;

  const vec3f* queryPosBuffer;

  //local variable to store the query value
  float* sampleValues;

  P4estThreadContext() : volume(nullptr), ctx(nullptr) {
    data = std::numeric_limits<double>::infinity();
  }
  ~P4estThreadContext() {
    if (ctx) {
      p4est_ospray_search_context_destroy(ctx);
    }
  }
};

thread_local P4estThreadContext thread_search_ctx;

int pt_search_callback(p4est_t * p4est,
                       p4est_topidx_t which_tree,
                       p4est_quadrant_t * quadrant,
                       p4est_locidx_t local_num,
                       void *point){
  double aabb[6];
  double *lower_corner = &aabb[0];
  double *upper_corner = &aabb[3];
  
  //Pseudocode: "renderer = p4est->user_pointer". Where renderer is some handle to our ospray module's state. 
  P4estThreadContext * sContext = (P4estThreadContext *)p4est->user_pointer;
  p4est_t *p4est_orig = sContext->volume->p4est;

  p4est_ospray_quadrant_aabb (p4est_orig, which_tree, quadrant, aabb);

  double* pt = (double *)point;
  
  //test if the point located in the aabb
  if ( pt[0] < lower_corner[0] ||  pt[0] > upper_corner[0]
      || pt[1] < lower_corner[1] ||  pt[1] > upper_corner[1]
#ifdef P4_TO_P8
      || pt[2] < lower_corner[2] ||  pt[2] > upper_corner[2]
#endif
  ) {
    //sContext->data = 0.0; //assuming miss means zero density.
    return 0;	//outside, tell p4est to terminate traversal
  } else { //point may be contained in the octant/quadrant 
    if(local_num >= 0){ 
     //note we *always* need 3 entries of xyz, even in 2D. 
      sContext->volume->data_callback (p4est_orig, which_tree, quadrant,
          pt, &sContext->data);
    }
    return 1; //tells p4est point may be contained in the octant/quadrant 
  }
}

int pt_batch_search_callback(p4est_t * p4est,
                       p4est_topidx_t which_tree,
                       p4est_quadrant_t * quadrant,
                       p4est_locidx_t local_num,
                       void *activeID){


  double aabb[6];
  double *lower_corner = &aabb[0];
  double *upper_corner = &aabb[3];
  
  //Pseudocode: "renderer = p4est->user_pointer". Where renderer is some handle to our ospray module's state. 
  P4estThreadContext * sContext = (P4estThreadContext *)p4est->user_pointer;
  p4est_t *p4est_orig = sContext->volume->p4est;

  p4est_ospray_quadrant_aabb (p4est_orig, which_tree, quadrant, aabb);

  const int* actID = (int*)activeID;
  vec3f point = sContext->queryPosBuffer[*actID];

  double pt[3];
  pt[0] = point.x;
  pt[1] = point.y;
  pt[2] = point.z;
  
  //test if the point located in the aabb
  if (pt[0] < lower_corner[0] || pt[0] > upper_corner[0]
      || pt[1] < lower_corner[1] || pt[1] > upper_corner[1]
#ifdef P4_TO_P8
      || pt[2] < lower_corner[2] || pt[2] > upper_corner[2]
#endif
  ) {
    return 0;	//outside, tell p4est to terminate traversal
  } else { //point may be contained in the octant/quadrant 
    if(local_num >= 0){ 
      //note we *always* need 3 entries of xyz, even in 2D. 
      double queryValue;
      sContext->volume->data_callback (p4est_orig, which_tree, quadrant,pt,&queryValue);
      sContext->sampleValues[*actID] = (float)queryValue;
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
    if (!thread_search_ctx.ctx) {
      thread_search_ctx.volume = p4estv;
      thread_search_ctx.local = *p4estv->p4est; 
      // Pass the context itself through as user data
      thread_search_ctx.local.user_pointer = (void *)(&thread_search_ctx);
      thread_search_ctx.ctx = p4est_ospray_search_context_new(&thread_search_ctx.local,
          P4EST_OSPRAY_SEARCH_REUSE_MULTIPLE);
    }

    //TODO: don't initialize search_pt_array in every call to sample(). Ideally, we should initialize beforehand. 
    double xyz[3];
    xyz[0] = pos.x;
    xyz[1] = pos.y;
    xyz[2] = pos.z;


    sc_array_t search_pt_array;
    sc_array_init_data(&search_pt_array, (void *)(&xyz[0]), 3*sizeof(double), 1);

    thread_search_ctx.data = std::numeric_limits<double>::quiet_NaN(); //hack

    //synchronous search function 
    p4est_ospray_search_local(thread_search_ctx.ctx, p4estv->treeID,
        0, NULL, pt_search_callback, &search_pt_array);
    
    if(std::isnan(thread_search_ctx.data)){ //miss
      return 0.0; //"miss" value
    } 

    /* TODO: make sure the result is thread-safe (multiple buffers, one per tid) */
    return (float)thread_search_ctx.data;
    //return 0.5f; //debug only
  }


  virtual void batch_sample(const vec3f* posBuffer,
                            const int numActivePos,
                            const int* activeIDs, 
                            float* values) const override
  {
    if (!thread_search_ctx.ctx) {
      thread_search_ctx.volume = p4estv;
      thread_search_ctx.local = *p4estv->p4est; 

      // Pass the context itself through as user data
      thread_search_ctx.local.user_pointer = (void *)(&thread_search_ctx);

      thread_search_ctx.ctx = p4est_ospray_search_context_new(&thread_search_ctx.local,
          P4EST_OSPRAY_SEARCH_REUSE_MULTIPLE);
    }

    thread_search_ctx.queryPosBuffer = posBuffer;
    thread_search_ctx.sampleValues = values; 

    sc_array_t search_pt_array;
    sc_array_init_data(&search_pt_array, (void *)(activeIDs), sizeof(int), numActivePos);

    // TODO: put the tree ID in here
    p4est_ospray_search_local(thread_search_ctx.ctx, p4estv->treeID,
        0, NULL, pt_batch_search_callback, &search_pt_array);
   
  }

  
  virtual vec3f computeGradient(const vec3f &pos) const override
  {
    return vec3f(1,0,0);
  }


private:
  P4estVolume *p4estv;
};

