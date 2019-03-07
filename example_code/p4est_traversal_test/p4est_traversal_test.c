/*
  This file is part of p4est.
  p4est is a C library to manage a collection (a forest) of multiple
  connected adaptive quadtrees or octrees in parallel.

  Copyright (C) 2010 The University of Texas System
  Additional copyright (C) 2011 individual authors
  Written by Carsten Burstedde, Lucas C. Wilcox, and Tobin Isaac

  p4est is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  p4est is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with p4est; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

/** \file p4est_step1.c
 *
 * This 2D example program refines a domain based on given image data.
 * The image file hw32.h has been created with the GIMP and is compiled in.
 */

/* p4est has two separate interfaces for 2D and 3D, p4est*.h and p8est*.h.
 * Most API functions are available for both dimensions.  The header file
 * p4est_to_p8est.h #define's the 2D names to the 3D names such that most code
 * only needs to be written once.  In this example, we rely on this. */
#ifndef P4_TO_P8
#include <p4est_vtk.h>
#include <p4est_extended.h>
#include <p4est_bits.h>
#include <p4est_search.h>
#else
#include <p8est_vtk.h>
#include <p8est_extended.h>
#include <p8est_bits.h>
#include <p8est_search.h>
#endif
#include "hw32.h"

/** The resolution of the image data in powers of two. */
#define P4EST_STEP1_PATTERN_LEVEL 5
/** The dimension of the image data. */
#define P4EST_STEP1_PATTERN_LENGTH (1 << P4EST_STEP1_PATTERN_LEVEL)
static const int    plv = P4EST_STEP1_PATTERN_LEVEL;    /**< Shortcut */
static const int    ple = P4EST_STEP1_PATTERN_LENGTH;   /**< Shortcut */
#ifdef P4_TO_P8
static const p4est_qcoord_t eighth = P4EST_QUADRANT_LEN (3);
#endif

//H1, H2 are the length of one side of a quadrant at level 1, 2
#define H1 (1 << (P4EST_MAXLEVEL - 1))
#define H2 (1 << (P4EST_MAXLEVEL - 2))

int pt_search_callback(p4est_t * p4est,
                       p4est_topidx_t which_tree,
                       p4est_quadrant_t * quadrant,
                       p4est_locidx_t local_num,
                       void *point){


  //ASSUME that our point is a length 3 / length 2 array of doubles.
  double *pt_xyz = (double*)point;
  double lower_corner[3];
  double upper_corner[3];

  //obtain the upper and lower corner location of the octant/quadrant in world space
	p4est_qcoord_t	int_len = P4EST_QUADRANT_LEN (quadrant->level); //lengh in integer coords

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
	if ( pt_xyz[0] < lower_corner[0] ||  pt_xyz[0] > upper_corner[0]
			|| pt_xyz[1] < lower_corner[1] ||  pt_xyz[1] > upper_corner[1]
#ifdef P4_TO_P8
			|| pt_xyz[2] < lower_corner[2] ||  pt_xyz[2] > upper_corner[2]
#endif
	) {
		return 0;	//outside, tell p4est to terminate traversal
	} else { //point may be contained in the octant/quadrant 
    if(local_num > 0){ //reached a leaf

      // Here, we would do bilinear interpolation of the data value
      // Right now I am just storing the coordinates of the quadrant, because in
      // our toy quadtree example there is no data
      // BOLD ASSUMPTION: the user pointer points to the head of an array of doubles with length at least 3.
      p4est_qcoord_t* out_array = (p4est_qcoord_t*)p4est->user_pointer;
      out_array[0] = quadrant->x;
      out_array[1] = quadrant->y;
#ifdef P4_TO_P8
      out_array[2] = quadrant->z;
#endif
    }
    return 1; //tells p4est point may be contained in the octant/quadrant 
  }
}

static void
volume_callback (p4est_iter_volume_info_t * info, void *user_data)
{
  p4est_quadrant_t* o = info->quad; //o is the current quadrant
  p4est_qcoord_t oct_len = P4EST_QUADRANT_LEN(o->level);
  p4est_qcoord_t x = o->x;
  p4est_qcoord_t y = o->y;

  double lower_corner[3];
  double upper_corner[3];

  //obtain the upper and lower corner location of the octant/quadrant in world space
	p4est_qcoord_t	int_len = P4EST_QUADRANT_LEN (o->level); //lengh in integer coords

  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         o->x,
                         o->y,
#ifdef P4_TO_P8
                         quadrant->z,
#endif
                         lower_corner);

  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         o->x + oct_len,
                         o->y + oct_len,
#ifdef P4_TO_P8
                         o->z + oct_len,
#endif
                         upper_corner);

  printf("World lower corner: %f %f\n", lower_corner[0], lower_corner[1]);
  printf("World upper corner: %f %f\n", upper_corner[0], upper_corner[1]);

  //In this toy example, look for the quadrant that we want.
  //Then get its parent, then get its parent's parent.
  /*
   *if(x == (H1 + H2) && y == H2){
   *  //get parent
   *  p4est_quadrant_t dummy_quadrant;  
   *  p4est_quadrant_parent(o, &dummy_quadrant);
   *  printf("Parent x,y: %d %d\n", dummy_quadrant.x, dummy_quadrant.y);
   *  //get parent's parent
   *  p4est_quadrant_t dummy2;  
   *  p4est_quadrant_parent(&dummy_quadrant, &dummy2);
   *  printf("Grandparent x,y: %d %d\n", dummy2.x, dummy2.y);
   *}
   */
}
      
/** Callback function to decide on refinement.
 *
 * Refinement and coarsening is controlled by callback functions.
 * This function is called for every processor-local quadrant in order; its
 * return value is understood as a boolean refinement flag.
 */
//This exists to test my handwritten notes at this URL: 
//https://amr-project.tumblr.com/post/183203445294/notes-on-p4est-and-morton-indices-and-parent
static int
refine_fn (p4est_t * p4est, p4est_topidx_t which_tree,
           p4est_quadrant_t * quadrant){

  p4est_quadrant_t * o = quadrant;
  if (o->level ==0){
    return 1;
  } else if (o->level == 1 && o->x == H1 && o->y == 0){
    return 1;
  } else {
    return 0;
  }
}


/** The main function of the step1 example program.
 *
 * It creates a connectivity and forest, refines it, and writes a VTK file.
 */
int
main (int argc, char **argv)
{
  int                 mpiret;
  int                 recursive, partforcoarsen, balance;
  sc_MPI_Comm         mpicomm;
  p4est_t            *p4est;
  p4est_connectivity_t *conn;
  
  if(argc != 3){
    printf("Usage: ./p4est_traversal_test <x_coord> <y_coord>\n");
    exit(1);
  }

  double query_x = atof(argv[1]);
  double query_y = atof(argv[2]);

  printf("Queried coordinate: (%f %f)\n", query_x, query_y);

  /* Initialize MPI; see sc_mpi.h.
   * If configure --enable-mpi is given these are true MPI calls.
   * Else these are dummy functions that simulate a single-processor run. */
  mpiret = sc_MPI_Init (&argc, &argv);
  SC_CHECK_MPI (mpiret);
  mpicomm = sc_MPI_COMM_WORLD;

  /* These functions are optional.  If called they store the MPI rank as a
   * static variable so subsequent global p4est log messages are only issued
   * from processor zero.  Here we turn off most of the logging; see sc.h. */
  sc_init (mpicomm, 1, 1, NULL, SC_LP_ESSENTIAL);
  p4est_init (NULL, SC_LP_PRODUCTION);
  P4EST_GLOBAL_PRODUCTIONF
    ("This is the p4est %dD demo example/steps/%s_step1\n",
     P4EST_DIM, P4EST_STRING);

  /* Create a forest that consists of just one quadtree/octree.
   * This file is compiled for both 2D and 3D: the macro P4_TO_P8 can be
   * checked to execute dimension-dependent code. */
#ifndef P4_TO_P8
  conn = p4est_connectivity_new_unitsquare ();
#else
  conn = p8est_connectivity_new_unitcube ();
#endif

  /* Create a forest that is not refined; it consists of the root octant. */
  p4est = p4est_new (mpicomm, conn, 0, NULL, NULL);

  /* Refine the forest recursively in parallel.
   * Since refinement does not change the partition boundary, this call
   * must not create an overly large number of quadrants.  A numerical
   * application would call p4est_refine non-recursively in a loop,
   * repartitioning in each iteration.
   * The P4EST_ASSERT macro only activates with --enable-debug.
   * We check against the data dimensions in example/steps/hw32.h. */
  P4EST_ASSERT (P4EST_STEP1_PATTERN_LENGTH == width);
  P4EST_ASSERT (P4EST_STEP1_PATTERN_LENGTH == height);
  recursive = 1;
  p4est_refine (p4est, recursive, refine_fn, NULL);

  /* Partition: The quadrants are redistributed for equal element count.  The
   * partition can optionally be modified such that a family of octants, which
   * are possibly ready for coarsening, are never split between processors. */
  partforcoarsen = 0;
  p4est_partition (p4est, partforcoarsen, NULL);

  /* If we call the 2:1 balance we ensure that neighbors do not differ in size
   * by more than a factor of 2.  This can optionally include diagonal
   * neighbors across edges or corners as well; see p4est.h. */
  balance = 1;
  if (balance) {
    p4est_balance (p4est, P4EST_CONNECT_FACE, NULL);
    p4est_partition (p4est, partforcoarsen, NULL);
  }

  //TODO: Call p4est_iterate here.
//  p4est_iterate (p4est,       /* the forest */
//           NULL,      /* the ghost layer */
//           NULL,        /* user data */
//           volume_callback, /* callback to compute each quad's
//                       interior contribution to du/dt */                
//           NULL,        /* callback to compute each quads'
//                       faces' contributions to du/du */
//#ifdef P4_TO_P8              
//           NULL,           /* there is no callback for the 
//                    edges between quadrants */
//#endif                      
//           NULL);          /* there is no callback for the
//                    corners between quadrants */

  /*double search_pt[3] = {0.6, 0.375, 0.0};*/
  double search_pt[3];
  search_pt[0] = query_x;
  search_pt[1] = query_y;
  search_pt[2] = 0.0;

  //result_storage is an array of length 3 that stores the result of our p4est_search. 
  int result_storage[3] = {-1337, -1337, -1337}; 
  p4est->user_pointer = result_storage;

  //TODO: put the search_pt into an sc_array_t, because that's what p4est_search expects.

  sc_array_t search_pt_array;
  search_pt_array.elem_size = 3*sizeof(double); 
  search_pt_array.elem_count = 1;
  search_pt_array.array = (char*)search_pt;
  p4est_search(p4est, NULL, pt_search_callback, &search_pt_array);

  printf("Coord of quadrant/octant found by search: %d %d\n", result_storage[0], result_storage[1]);

  /* Write the forest to disk for visualization, one file per processor. */
  p4est_vtk_write_file (p4est, NULL, P4EST_STRING "_step1");

  /* Destroy the p4est and the connectivity structure. */
  p4est_destroy (p4est);
  p4est_connectivity_destroy (conn);

  /* Verify that allocations internal to p4est and sc do not leak memory.
   * This should be called if sc_init () has been called earlier. */
  sc_finalize ();

  /* This is standard MPI programs.  Without --enable-mpi, this is a dummy. */
  mpiret = sc_MPI_Finalize ();
  SC_CHECK_MPI (mpiret);
  return 0;
}
