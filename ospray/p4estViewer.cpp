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
};

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

  std::vector<Sphere> spheres(10);

  std::random_device rd;
  std::mt19937 rng(rd());

  const vec3f brickLower(-1.f);
  const vec3f brickUpper(1.f);

  // Generate spheres within the box padded by the radius, so we don't need
  // to worry about ghost bounds
  std::uniform_real_distribution<float> distX(brickLower.x, brickUpper.x);
  std::uniform_real_distribution<float> distY(brickLower.y, brickUpper.y);
  std::uniform_real_distribution<float> distZ(brickLower.z, brickUpper.z);
  std::uniform_real_distribution<float> radii(0.05, 0.25);

  for (auto &s : spheres) {
    s.pos.x = distX(rng);
    s.pos.y = distY(rng);
    s.pos.z = distZ(rng);
    s.radius = radii(rng);
  }

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
          vec2i{1024, 768}, box3f(vec3f(-1.f), vec3f(1.f)), world, renderer));

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

  /* Verify that allocations internal to p4est and sc do not leak memory.
   * This should be called if sc_init () has been called earlier. */
  sc_finalize ();

  /* This is standard MPI programs.  Without --enable-mpi, this is a dummy. */
  mpiret = sc_MPI_Finalize ();
  SC_CHECK_MPI (mpiret);

  return 0;
}

