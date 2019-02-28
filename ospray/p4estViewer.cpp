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

#include "GLFWOSPRayWindow.h"
#include "ospray/ospray.h"
#include "ospray/common/OSPCommon.h"

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

#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>

/** The resolution of the image data in powers of two. */
#define P4EST_STEP1_PATTERN_LEVEL 5
/** The dimension of the image data. */
#define P4EST_STEP1_PATTERN_LENGTH (1 << P4EST_STEP1_PATTERN_LEVEL)
static const int    plv = P4EST_STEP1_PATTERN_LEVEL;    /**< Shortcut */
static const int    ple = P4EST_STEP1_PATTERN_LENGTH;   /**< Shortcut */
#ifdef P4_TO_P8
static const p4est_qcoord_t eighth = P4EST_QUADRANT_LEN (3);
#endif

using namespace ospcommon;

//TODO: The following variables probably shouldn't be "globals"
std::vector<vec3f> verts;
std::vector<float> cellField;
std::vector<vec4i> indices;

//From http://www.martinbroadhurst.com/how-to-split-a-string-in-c.html
template <class Container>
void split_string(const std::string& str, Container& cont, char delim = ' ')
{
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        cont.push_back(token);
    }
}

/*
 *struct Sphere {
 *  vec3f pos;
 *  float radius;
 *
 *  Sphere(const vec3f &p, float r) : pos(p), radius(r) {}
 *};
 */

//std::vector<Sphere> spheres;

/** Get the coordinates of the midpoint of a quadrant.
 *
 * NATHAN: Originally from p4est_step3.h 
 *
 * \param [in]  p4est      the forest
 * \param [in]  which_tree the tree in the forest containing \a q
 * \param [in]  q          the quadrant
 * \param [out] xyz        the coordinates of the midpoint of \a q
 */
/*
 *static void
 *get_midpoint (p4est_t * p4est, p4est_topidx_t which_tree,
 *                    p4est_quadrant_t * q, double xyz[3])
 *{
 *  p4est_qcoord_t      half_length = P4EST_QUADRANT_LEN (q->level) / 2;
 *
 *  p4est_qcoord_to_vertex (p4est->connectivity, which_tree,
 *                          q->x + half_length, q->y + half_length,
 *#ifdef P4_TO_P8
 *                          q->z + half_length,
 *#endif
 *                          xyz);
 *}
 */

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
  p4est_qcoord_t oct_len = P4EST_QUADRANT_LEN(o->level);
  p4est_qcoord_t x = o->x;
  p4est_qcoord_t y = o->y;
  p4est_qcoord_t z = o->z;

/*
 *  double midpoint_xyz[3];
 *  get_midpoint(info->p4est, info->treeid, info->quad, midpoint_xyz);
 *  double face_point_xyz[3];
 *  p4est_qcoord_to_vertex(info->p4est->connectivity,
 *                         info->treeid,
 *                         x + oct_len,
 *                         y + oct_len / 2,
 *                         z + oct_len / 2,
 *                         face_point_xyz);
 *  
 *  vec3d p = vec3d(face_point_xyz[0], face_point_xyz[1], face_point_xyz[2]);
 *  vec3d m = vec3d(midpoint_xyz[0], midpoint_xyz[1], midpoint_xyz[2]);
 *  double radius = length(p - m); 
 *
 *
 *  double world_xyz[3]; //coordinates in world space
 *  p4est_qcoord_to_vertex(info->p4est->connectivity,
 *                         info->treeid,
 *                         x, y, z,
 *                         world_xyz);
 */

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

    printf("(%d, %d, %d): %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g\n",
    //printf("(%d, %d, %d): %f %f %f %f %f %f %f %f\n",
           x,
           y,
           z,
           *double1,
           *double2,
           *double3,
           *double4,
           *double5,
           *double6,
           *double7,
           *double8);
    
    double avg = (*double1 + *double2 + *double3 + *double4 + *double5 + *double6 + *double7 + *double8)/8;
    cellField.push_back(static_cast<float>(avg));
  } else {
    std::cout << "No data!" << std::endl;
  }

    //printf("World radius: %f Midpoint world coordinates: (%f, %f, %f)\n", radius, m.x, m.y, m.z);
    //spheres.push_back(Sphere(vec3f(m.x, m.y, m.z), radius));
  
    //Below code from Will
    //spheres.push_back(Sphere(vec3f(world_xyz[0], world_xyz[1], world_xyz[2]),
   //0.1));
}

int main(int argc, char **argv) {
  if( argc != 2 ){
    std::cout << "Usage: p4estViewer <path_to_p4est_file>" << std::endl;
    exit(1);
  }
  int                 mpiret;
  //int                 recursive, partforcoarsen, balance;
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

  // initialize OSPRay; OSPRay parses (and removes) its commandline parameters,
  // e.g. "--osp:debug"
  OSPError initError = ospInit(&argc, (const char**)argv);

  if (initError != OSP_NO_ERROR)
    return initError;

  // set an error callback to catch any OSPRay errors and exit the application
  ospDeviceSetErrorFunc(
      ospGetCurrentDevice(), [](OSPError error, const char *errorDetails) {
        std::cerr << "OSPRay error: " << errorDetails << std::endl;
        exit(error);
      });

  //See p4est_extended.h
	//int data_size = 0;
	//int load_data = 0;
	int autopartition = 1;
	int broadcasthead = 0;
	int* user_ptr = NULL;
	//char* input_fname = argv[1];
  std::string input_fname = std::string(argv[1]);

  //NATHAN: Read info file. Use this file to decide if we want to load data, and if so, how much.
  std::string info_fname = input_fname + ".info";
  std::ifstream info_fstream(info_fname, std::ios::in);
  std::string currLine;
  // Below is hack that assumes that info files are two lines long, and in the
  // format of Dr. Heister's sample info files located in ../data/cube_*/
  std::getline(info_fstream, currLine); //first line is a human-readable header
  std::getline(info_fstream, currLine); //second line contains the info that we want
  info_fstream.close();
  std::vector<std::string> str_tokens;
  split_string<std::vector<std::string>>(currLine, str_tokens);

  /*
   *for (std::string str : str_tokens) {
   *  std::cout << str << std::endl;
   *}
   */

  int num_bytes = std::stoi(str_tokens[2]);
  int load_data  = (num_bytes > 0); 

  if (load_data) {
    std::cout << "Loading data..." << std::endl;
  } else {
    std::cout << "No data to load." << std::endl;
  } 

  //NATHAN: Read p4est from file.
	p4est = p4est_load_ext(input_fname.c_str(), mpicomm, num_bytes,
			load_data, autopartition, broadcasthead, user_ptr, &conn);



  // TODO: compute world bounds or read it from p4est
  box3f worldBounds(vec3f(0.f), vec3f(1.f));

  // NATHAN: from the docs: There is also the possibility to aggregate many
  // values of the same type into an array, which then itself can be used as a
  // parameter to objects. To create such a new data buffer, holding numItems
  // elements of the given type, from the initialization data pointed to by
  // source and optional creation flags, use ospNewData()
  
  //Below we aren't passing any creation flags (optional last argument).
  //OSP_UCHAR denotes an 8-bit unsigned character scalar.
  //OSPData sphereData =
      //ospNewData(spheres.size() * sizeof(Sphere), OSP_UCHAR, spheres.data());

  // NATHAN: From the docs: "parameters DO NOT get passed to objects
  // immediately. Instead, parameters are not visible at all to objects until
  // they get explicitly committed to a given object via a call to
  // ospCommit(OSPObject); at which time all previously additions or changes to
  // parameters are visible at the same time."
  // ...
  // "The commit semantic allow for batching up multiple small changes, and
  // specifies exactly when changes to objects will occur."
  // ...
  // "To indicate that the application does not need and does not access the
  // given object anymore, call ospRelease(OSPObject);" 
  // NATHAN: We call ospRelease() on sphereData later.
  //ospCommit(sphereData);

  //OSPMaterial material = ospNewMaterial2("scivis", "OBJMaterial");
  //ospSet3f(material, "Kd", 0.f, 0.f, 1.f);
  //ospSet3f(material, "Ks", 1.f, 1.f, 1.f);
  //ospCommit(material);

  //OSPGeometry sphereGeom = ospNewGeometry("spheres");
  //ospSet1i(sphereGeom, "bytes_per_sphere", int(sizeof(Sphere)));
  //ospSet1i(sphereGeom, "offset_radius", int(sizeof(vec3f)));
  //ospSetData(sphereGeom, "spheres", sphereData);
  //ospSetMaterial(sphereGeom, material);
  //ospRelease(material);
  //ospRelease(sphereData);
  //ospCommit(sphereGeom);

  //*********************************************************
  //Set up us the transfer function
	OSPTransferFunction transferFcn = ospNewTransferFunction("piecewise_linear");
	const std::vector<vec3f> colors = {
		vec3f(0, 0, 0.563),
		vec3f(0, 0, 1),
		vec3f(0, 1, 1),
		vec3f(0.5, 1, 0.5),
		vec3f(1, 1, 0),
		vec3f(1, 0, 0),
		vec3f(0.5, 0, 0)
	};
	const std::vector<float> opacities = {0.01, 0.05, 0.01};
	OSPData colorsData = ospNewData(colors.size(), OSP_FLOAT3, colors.data());
	ospCommit(colorsData);
	OSPData opacityData = ospNewData(opacities.size(), OSP_FLOAT, opacities.data());
	ospCommit(opacityData);

  //The value range here will be different from Will's code. It will need to match Timo's data.
  const vec2f valueRange(0.0f, 2.0f);
  ospSetData(transferFcn, "colors", colorsData);
	ospSetData(transferFcn, "opacities", opacityData);
  ospSet2f(transferFcn, "valueRange", valueRange.x, valueRange.y);
	ospCommit(transferFcn);
  //End transfer function setup
  //*********************************************************

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


  OSPData vtxData = ospNewData(verts.size(), OSP_FLOAT3, verts.data());
  OSPData idxData = ospNewData(indices.size(), OSP_INT4, indices.data());
  OSPData cellFieldData = ospNewData(cellField.size(), OSP_FLOAT, cellField.data());

  std::cout << "verts size: " << verts.size() << std::endl;
  for(size_t i = 0; i < verts.size(); i++){
    vec3f currVert = verts.data()[i];
    printf("Vert: (%f, %f, %f)\n", currVert[0], currVert[1], currVert[2]);
  }
  std::cout << "indices size: " << indices.size() << std::endl;
  for(size_t i = 0; i < indices.size(); i++){
    vec4i currIdx = indices.data()[i];
    printf("Indices: (%d, %d, %d, %d)\n", currIdx[0], currIdx[1], currIdx[2], currIdx[3]);
  }
  std::cout << "cellField size: " << cellField.size() << std::endl;
  for(size_t i = 0; i < cellField.size(); i++){
    float currCellValue = cellField.data()[i];
    std::cout << currCellValue << std::endl;
  }

  ospCommit(vtxData);
  ospCommit(idxData);
  ospCommit(cellFieldData);

  OSPVolume volume = ospNewVolume("unstructured_volume");
	ospSetObject(volume, "transferFunction", transferFcn);
  ospSetData(volume, "vertices", vtxData);
  ospSetData(volume, "cellField", cellFieldData);
  ospSetData(volume, "indices", idxData);

  //ospSet3f(volume, "volumeClippingBoxLower", 0.0f, 0.0f, 0.0f);
  //ospSet3f(volume, "volumeClippingBoxUpper", 0.5f, 0.5f, 0.5f);
  ospCommit(volume);

  // create the "world" model which will contain all of our geometries
  OSPModel world = ospNewModel();
  ospAddVolume(world, volume);
  //ospAddGeometry(world, sphereGeom);
  ospCommit(world);
  //ospRelease(sphereGeom);
  //ospRelease(volume);

  // create and setup an ambient light
  std::array<OSPLight, 2> lights = {
    ospNewLight3("ambient"),
    ospNewLight3("distant")
  };
  ospCommit(lights[0]);

  ospSet3f(lights[1], "direction", -1.f, -1.f, 0.5f);
  ospCommit(lights[1]);

  OSPData lightData = ospNewData(lights.size(), OSP_LIGHT, lights.data(), 0);
  ospCommit(lightData);

  // create OSPRay renderer
  OSPRenderer renderer = ospNewRenderer("scivis");
  ospSetObject(renderer, "lights", lightData);
  ospSet3f(renderer, "bgColor", 1.0, 1.0, 1.0);
  ospCommit(renderer);
  ospRelease(lightData);

  // create a GLFW OSPRay window: this object will create and manage the OSPRay
  // frame buffer and camera directly
  auto glfwOSPRayWindow =
      std::unique_ptr<GLFWOSPRayWindow>(new GLFWOSPRayWindow(
          vec2i{1024, 768}, worldBounds, world, renderer));

  glfwOSPRayWindow->registerImGuiCallback([&]() {
    static int spp = 1;
    if (ImGui::SliderInt("spp", &spp, 1, 64)) {
      ospSet1i(renderer, "spp", spp);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  // cleanup remaining objects
  ospRelease(world);
  ospRelease(renderer);

  // cleanly shut OSPRay down
  ospShutdown();

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
