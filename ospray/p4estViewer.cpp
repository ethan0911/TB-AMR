#include <iterator>
#include <memory>
#include <random>
#include <imgui.h>
#include "GLFWOSPRayWindow.h"

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

struct Sphere {
  vec3f pos;
  float radius;

  Sphere(const vec3f &p, float r) : pos(p), radius(r) {}
};

std::vector<Sphere> spheres;

/** Get the coordinates of the midpoint of a quadrant.
 *
 * NATHAN: Originally from p4est_step3.h 
 *
 * \param [in]  p4est      the forest
 * \param [in]  which_tree the tree in the forest containing \a q
 * \param [in]  q          the quadrant
 * \param [out] xyz        the coordinates of the midpoint of \a q
 */
static void
get_midpoint (p4est_t * p4est, p4est_topidx_t which_tree,
                    p4est_quadrant_t * q, double xyz[3])
{
  p4est_qcoord_t      half_length = P4EST_QUADRANT_LEN (q->level) / 2;

  p4est_qcoord_to_vertex (p4est->connectivity, which_tree,
                          q->x + half_length, q->y + half_length,
#ifdef P4_TO_P8
                          q->z + half_length,
#endif
                          xyz);
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
  p4est_qcoord_t oct_len = P4EST_QUADRANT_LEN(o->level);
  p4est_qcoord_t x = o->x;
  p4est_qcoord_t y = o->y;
  p4est_qcoord_t z = o->z;

  double midpoint_xyz[3];
  get_midpoint(info->p4est, info->treeid, info->quad, midpoint_xyz);
  double face_point_xyz[3];
  p4est_qcoord_to_vertex(info->p4est->connectivity,
                         info->treeid,
                         x + oct_len,
                         y + oct_len / 2,
                         z + oct_len / 2,
                         face_point_xyz);
  
  vec3d p = vec3d(face_point_xyz[0], face_point_xyz[1], face_point_xyz[2]);
  vec3d m = vec3d(midpoint_xyz[0], midpoint_xyz[1], midpoint_xyz[2]);
  double radius = length(p - m); 


    double world_xyz[3]; //coordinates in world space
    p4est_qcoord_to_vertex(info->p4est->connectivity,
                           info->treeid,
                           x, y, z,
                           world_xyz);

    size_t data_size = info->p4est->data_size;
    if(data_size > 0){
      //TODO: Check if static_cast is the right type of cast
      char* curr_data = static_cast<char*>(o->p.user_data);

      // Loop through the buffer, and print the contents at a hex
      // string (one hex character per nibble, 68 bytes = 136 nibbles)
      // Concatenate the hex characters to a stringstream

/*
 *      std::stringstream ss;
 *      //The most significant byte gets appended to the stringbuffer first.
 *      //The least significant byte gets appended to the stringbuffer last.
 *      for (int i = data_size - 1; i >= 0; i--) {
 *        char curr_byte = curr_data[i]; 
 *        char char_pair[2];
 *        snprintf(char_pair, 2, "%x", curr_byte);
 *        ss << char_pair;
 *      }
 *
 *      std::cout << "(" << x << "," << y << "," << z << "): 0x" << ss.str() << std::endl;
 */

      //Interpret the most significant 8 bytes as floats. Ignore the last 4 bytes.
      double double1 = *(curr_data + 60); 
      double double2 = *(curr_data + 53); 
      double double3 = *(curr_data + 46); 
      double double4 = *(curr_data + 39); 
      double double5 = *(curr_data + 32); 
      double double6 = *(curr_data + 25); 
      double double7 = *(curr_data + 18); 
      double double8 = *(curr_data + 11); 

      //std::cout << "(" << x << "," << y << "," << z << "): 0x" << ss.str() << std::endl;
      //printf("(%d, %d, %d): %.4g %.4g %.4g %.4g %.4g %.4g %.4g %.4g\n",
      printf("(%d, %d, %d): %f %f %f %f %f %f %f %f\n",
             //x,
             //y,
             //z,
             //double1,
             //double2,
             //double3,
             //double4,
             //double5,
             //double6,
             //double7,
             //double8);

      //Question: Winding order?

      // Then start looking at how to put an unstructured volume together, and coloring the cells.
      // Could begin by coloring by level.
      // Hopefully the volume renderer in OSPRay lets us set the transfer function, etc.
    }

    //printf("World radius: %f Midpoint world coordinates: (%f, %f, %f)\n", radius, m.x, m.y, m.z);
    spheres.push_back(Sphere(vec3f(m.x, m.y, m.z), radius));
  
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


  // TODO: compute world bounds or read it from p4est
  box3f worldBounds(vec3f(0.f), vec3f(1.f));

  // NATHAN: from the docs: There is also the possibility to aggregate many
  // values of the same type into an array, which then itself can be used as a
  // parameter to objects. To create such a new data buffer, holding numItems
  // elements of the given type, from the initialization data pointed to by
  // source and optional creation flags, use ospNewData()
  
  //Below we aren't passing any creation flags (optional last argument).
  //OSP_UCHAR denotes an 8-bit unsigned character scalar.
  OSPData sphereData =
      ospNewData(spheres.size() * sizeof(Sphere), OSP_UCHAR, spheres.data());

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
  ospCommit(sphereData);

  OSPMaterial material = ospNewMaterial2("scivis", "OBJMaterial");
  ospSet3f(material, "Kd", 0.f, 0.f, 1.f);
  ospSet3f(material, "Ks", 1.f, 1.f, 1.f);
  ospCommit(material);

  OSPGeometry sphereGeom = ospNewGeometry("spheres");
  ospSet1i(sphereGeom, "bytes_per_sphere", int(sizeof(Sphere)));
  ospSet1i(sphereGeom, "offset_radius", int(sizeof(vec3f)));
  ospSetData(sphereGeom, "spheres", sphereData);
  ospSetMaterial(sphereGeom, material);
  ospRelease(material);
  ospRelease(sphereData);
  ospCommit(sphereGeom);

  // create the "world" model which will contain all of our geometries
  OSPModel world = ospNewModel();
  ospAddGeometry(world, sphereGeom);
  ospCommit(world);
  ospRelease(sphereGeom);

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
