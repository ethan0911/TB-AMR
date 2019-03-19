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
#include <p4est_to_p8est.h>

/* p4est has two separate interfaces for 2D and 3D, p4est*.h and p8est*.h.
 * Most API functions are available for both dimensions.  The header file
 * p4est_to_p8est.h #define's the 2D names to the 3D names such that most code
 * only needs to be written once.  In this example, we rely on this. */
#ifndef P4_TO_P8
#include <p4est_vtk.h>
#include <p4est_extended.h>
#else
#include <p8est_vtk.h>
#include <p8est_extended.h>
#endif

#include <stdio.h>

static double get_data_from_quadrant(const p4est_quadrant_t *o,
                                     const p4est_t *p4est)
{
  size_t data_size = p4est->data_size;
  if (data_size > 0) {
    // TODO: Check if static_cast is the right type of cast
    char *curr_data = (char *)(o->p.user_data);

    // Loop through the buffer, and print the contents at a hex
    // string (one hex character per nibble, 68 bytes = 136 nibbles)
    // Concatenate the hex characters to a stringstream

    // Interpret the most significant 8 bytes as floats. Ignore the least
    // significant 4 bytes.

    double *double1 = (double *)(curr_data + 60);
    double *double2 = (double *)(curr_data + 52);
    double *double3 = (double *)(curr_data + 44);
    double *double4 = (double *)(curr_data + 36);
    double *double5 = (double *)(curr_data + 28);
    double *double6 = (double *)(curr_data + 20);
    double *double7 = (double *)(curr_data + 12);
    double *double8 = (double *)(curr_data + 4);

    // printf("(%d, %d, %d): %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g\n",
    // //printf("(%d, %d, %d): %f %f %f %f %f %f %f %f\n",
    //        x,
    //        y,
    //        z,
    //        *double1,
    //        *double2,
    //        *double3,
    //        *double4,
    //        *double5,
    //        *double6,
    //        *double7,
    //        *double8);

    double avg = (*double1 + *double2 + *double3 + *double4 + *double5 +
                  *double6 + *double7 + *double8) /
                 8;
    return avg;
  } else { //No data
    return -1.337; 
  }
}

/** Nathan's function to iterate thru each cell 
 *
 * Like step3_interpolate_solution(), this function matches the
 * p4est_iter_volume_t prototype used by p4est_iterate().
 *
 * \param [in] info          the information about the quadrant populated by
 *                           p4est_iterate()
 * \param [in] user_data     not used
 */
static void
volume_callback (p4est_iter_volume_info_t * info, void *user_data)
{
	p4est_quadrant_t* o = info->quad; //o is the current octant
	//line of code below from p4est_step3.h, step3_get_midpoint() function
	p4est_qcoord_t half_length = P4EST_QUADRANT_LEN (o->level) / 2;
	p4est_qcoord_t x = o->x;
	p4est_qcoord_t y = o->y;
	p4est_qcoord_t z = o->z;

	double world_xyz[3]; //coordinates in world space
	p4est_qcoord_to_vertex(info->p4est->connectivity, 
						   info->treeid,
						   x, y, z,
						   world_xyz);
  double data = get_data_from_quadrant(o, info->p4est);

	printf("Radius: %d Integer coordinates: (%d, %d, %d) World coordinates: (%f, %f, %f) Data: %f\n", half_length, o->x, o->y, o->z, world_xyz[0], world_xyz[1], world_xyz[2], data);

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


	//See p4est_extended.h
	int data_size = 68; //HACK
	int load_data = 1; //HACK

	int autopartition = 1;
	int broadcasthead = 0;
	int* user_ptr = NULL;
	char* input_fname = argv[1];

	//NATHAN: Read p4est from file.
	p4est = p4est_load_ext(input_fname, mpicomm, data_size,
			load_data, autopartition, broadcasthead, user_ptr, &conn);

	p4est_iterate (p4est, 			/* the forest */
				   NULL, 			/* the ghost layer */
				   NULL,  			/* user data */
				   volume_callback, /* callback to compute each quad's
											 interior contribution to du/dt */
				   NULL,    		/* callback to compute each quads'
											 faces' contributions to du/du */
#ifdef P4_TO_P8
				   NULL,           /* there is no callback for the
									  edges between quadrants */
#endif
				   NULL);          /* there is no callback for the
									  corners between quadrants */
	
	/* Write the forest to disk for visualization, one file per processor. */
	p4est_vtk_write_file (p4est, NULL, P4EST_STRING "_loadTest");
	/*p4est_vtk_write_file (p4est, NULL, P4EST_STRING input_fname);*/

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
