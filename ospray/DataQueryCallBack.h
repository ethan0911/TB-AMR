#include <stdint.h>
#include <stdio.h>
#ifdef _WIN32
#  include <malloc.h>
#else
#  include <alloca.h>
#endif

#include <iterator>
#include <memory>
#include <random>
#include <imgui.h>

#include "ospray/ospray.h"
#include "ospray/common/OSPCommon.h"

#include <p4est_to_p8est.h>

/* p4est has two separate interfaces for 2D and 3D, p4est*.h and p8est*.h.
 * Most API functions are available for both dimensions.  The header file
 * p4est_to_p8est.h #define's the 2D names to the 3D names such that most code
 * only needs to be written once.  In this example, we rely on this. */
#ifndef P4_TO_P8
#include <p4est_extended.h>
#else
#include <p8est_extended.h>
#endif

#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>

using namespace ospcommon;

//TODO: The following variables probably shouldn't be "globals"
std::vector<vec3f> verts;
std::vector<float> cellField;
std::vector<vec4i> indices;

static void
volume_callback (p4est_iter_volume_info_t * info, void *user_data)
{
  p4est_quadrant_t* o = info->quad; //o is the current octant
  //line of code below from p4est_step3.h, step3_get_midpoint() function
  p4est_qcoord_t oct_len = P4EST_QUADRANT_LEN(o->level);
  p4est_qcoord_t x = o->x;
  p4est_qcoord_t y = o->y;
  p4est_qcoord_t z = o->z;


  std::vector<int> lower_idxs; //indices 0 thru 3 for the current hex
  std::vector<int> upper_idxs; //indices 4 thru 7 for the current hex

  double curr_pt[3];

  // Following the "winding order" found in Hexahedron.cxx from:
  // https://vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/Hexahedron
  // Because the OSPRay docs say that "for hexahedral cells... vertex ordering
  // is the same as VTK_HEXAHEDRON: four bottom vertices counterclockwise, then
  // top four counterclockwise."

  //Vertex 0
  lower_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x, y, z,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Vertex 1
  lower_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x + oct_len, y, z,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Vertex 2
  lower_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x + oct_len, y + oct_len, z,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Vertex 3
  lower_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x, y + oct_len, z,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Vertex 4
  upper_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x, y, z + oct_len,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Vertex 5
  upper_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x + oct_len, y, z + oct_len,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Vertex 6
  upper_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x + oct_len, y + oct_len, z + oct_len,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Vertex 7
  upper_idxs.push_back(verts.size());
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x, y + oct_len, z + oct_len,
                         curr_pt);
  verts.push_back(vec3f(static_cast<float>(curr_pt[0]), static_cast<float>(curr_pt[1]), static_cast<float>(curr_pt[2])));

  //Push indices to the index buffer
  indices.push_back(vec4i(lower_idxs[0], lower_idxs[1], lower_idxs[2], lower_idxs[3]));
  indices.push_back(vec4i(upper_idxs[0], upper_idxs[1], upper_idxs[2], upper_idxs[3]));

  size_t data_size = info->p4est->data_size;
  if(data_size > 0){
    //TODO: Check if static_cast is the right type of cast
    char* curr_data = static_cast<char*>(o->p.user_data);

    // Loop through the buffer, and print the contents at a hex
    // string (one hex character per nibble, 68 bytes = 136 nibbles)
    // Concatenate the hex characters to a stringstream

    //Interpret the most significant 8 bytes as floats. Ignore the least significant 4 bytes.

    double *double1 = reinterpret_cast<double*>(curr_data + 60); 
    double *double2 = reinterpret_cast<double*>(curr_data + 52); 
    double *double3 = reinterpret_cast<double*>(curr_data + 44); 
    double *double4 = reinterpret_cast<double*>(curr_data + 36); 
    double *double5 = reinterpret_cast<double*>(curr_data + 28); 
    double *double6 = reinterpret_cast<double*>(curr_data + 20); 
    double *double7 = reinterpret_cast<double*>(curr_data + 12); 
    double *double8 = reinterpret_cast<double*>(curr_data + 4); 

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
    
    double avg = (*double1 + *double2 + *double3 + *double4 + *double5 + *double6 + *double7 + *double8)/8;
    cellField.push_back(static_cast<float>(avg));
  } else {
    std::cout << "No data!" << std::endl;
  }
}


static void
load_data_callback (p4est_t * p4est,
                                            p4est_topidx_t which_tree,
                                            p4est_quadrant_t * quadrant,
                                            const double xyz[3],
                                            double *result)
{
#if 0
  void * data = quadrant->p.user_data;
#endif
  *result = (double) quadrant->level;
}
