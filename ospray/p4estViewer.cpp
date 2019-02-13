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

struct Sphere {
  vec3f pos;
  float radius;

  Sphere(const vec3f &p, float r) : pos(p), radius(r) {}
};


std::vector<Sphere> spheres;

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

	printf("Radius: %d Integer coordinates: (%d, %d, %d)"
         " World coordinates: (%f, %f, %f)\n",
         half_length, o->x, o->y, o->z, world_xyz[0], world_xyz[1], world_xyz[2]);

  spheres.push_back(Sphere(vec3f(world_xyz[0], world_xyz[1], world_xyz[2]), 0.1));
}

int main(int argc, char **argv) {
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
	int data_size = 0;
	int load_data = 0;
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


  // TODO: compute world bounds or read it from p4est
  box3f worldBounds(vec3f(0.f), vec3f(1.f));

  OSPData sphereData = ospNewData(spheres.size() * sizeof(Sphere), OSP_UCHAR,
                                  spheres.data());
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

