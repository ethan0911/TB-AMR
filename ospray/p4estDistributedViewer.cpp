#include <iterator>
#include <cmath>
#include <vector>
#include <memory>
#include <random>
#include <mpi.h>
#include <p4est_to_p8est.h>
#ifndef P4_TO_P8
#include <p4est_bits.h>
#include <p4est_extended.h>
#include <p4est_ospray.h>
#include <p4est_vtk.h>
const char         *default_prefix = "p4est_ospray";
#else
#include <p8est_bits.h>
#include <p8est_extended.h>
#include <p8est_ospray.h>
#include <p8est_vtk.h>
const char         *default_prefix = "p8est_ospray";
#endif
#include <sc_options.h>
#include "GLFWDistribP4estWindow.h"

#include <imgui.h>

using namespace ospcommon;

struct VolumeBrick {
  // the volume data itself
  OSPVolume brick;
  // the bounds of the owned portion of data
  box3f bounds;
  // the full bounds of the owned portion + ghost voxels
  box3f ghostBounds;
};

// From Carsten's p4est_ospray example
typedef struct ospex_global
{
  sc_MPI_Comm         mpicomm;

  /* variables set from command line */
  sc_options_t       *options;
  int                 tilt;
  int                 minlevel, maxlevel;
  int                 scaling;
  int                 write_vtk;
  double              nearness;
  const char         *prefix;

  /* other parameters */
  int                 mpiwrap;

  /* setup an ellispoid to refine around and render */
  double              center[3];
  double              halfax[3];
  double              halfmax;

  /* mesh */
  int                 curlevel;
  p4est_connectivity_t *conn;
  p4est_t            *p4est;

  /* data */
  sc_array_t         *pcw_const;
}
ospex_global_t;

static int
usagerr (sc_options_t * opt, const char *msg)
{
  P4EST_GLOBAL_LERRORF ("Usage required: %s\n", msg);
  return 1;
}

static int
ospex_parse_options (ospex_global_t * g, int argc, char **argv)
{
  int                 first_argc;
  int                 ue;
  sc_options_t       *opt = g->options;

  sc_options_add_bool (opt, 'T', "tilt", &g->tilt, 0,
                       "Tilt geometry away from axes");
  sc_options_add_int (opt, 'l', "minlevel", &g->minlevel, 0, "Lowest level");
  sc_options_add_int (opt, 'L', "maxlevel", &g->maxlevel, -1,
                      "Highest level");
  sc_options_add_double (opt, 'n', "nearness", &g->nearness, 1.7,
                         "Relative refinement distance");
  sc_options_add_bool (opt, 'S', "scaling", &g->scaling, 0,
                       "Reduce output for scalings");
  sc_options_add_string (opt, 'P', "prefix", &g->prefix, default_prefix,
                         "Prefix for file output");
  sc_options_add_bool (opt, 'V', "write-vtk", &g->write_vtk, 0,
                       "Output VTK files");

  /* set other parameters */
  g->mpiwrap = 16;

  /* proceed in run-once loop for clean abort */
  ue = 0;
  do {
    /* parse command line and assign configuration variables */

    first_argc = sc_options_parse (p4est_package_id, SC_LP_DEFAULT,
                                   opt, argc, argv);
    if (first_argc < 0 || first_argc != argc) {
      ue = usagerr (opt, "Invalid option format or non-option arguments");
      break;
    }
    P4EST_GLOBAL_PRODUCTIONF ("Dimension is %d\n", P4EST_DIM);
    sc_options_print_summary (p4est_package_id, SC_LP_ESSENTIAL, opt);

    /* check consistency of parameters */

    if (g->minlevel < 0 || g->minlevel > P4EST_QMAXLEVEL) {
      ue = usagerr (opt, "Minlevel between 0 and P4EST_QMAXLEVEL");
    }
    if (g->maxlevel == -1) {
      g->maxlevel = g->minlevel;
      P4EST_GLOBAL_ESSENTIALF ("Maxlevel set to %d\n", g->maxlevel);
    }
    if (g->maxlevel < g->minlevel || g->maxlevel > P4EST_QMAXLEVEL) {
      ue = usagerr (opt, "Maxlevel between minlevel and P4EST_QMAXLEVEL");
    }
    if (ue) {
      break;
    }

    /* prepare program run */

    if (g->scaling) {
      sc_package_set_verbosity (sc_package_id, SC_LP_PRODUCTION);
      sc_package_set_verbosity (p4est_package_id, SC_LP_PRODUCTION);
    }
  }
  while (0);
  if (ue) {
    sc_options_print_usage (p4est_package_id, SC_LP_ERROR, opt, NULL);
    return -1;
  }

  return 0;
}

static void
ospex_data_callback (p4est_t * p4est, p4est_topidx_t which_tree,
                     p4est_quadrant_t * quadrant, const double xyz[3],
                     double *result)
{
  ospex_global_t     *g = (ospex_global_t *) p4est->user_pointer;
  size_t              zz;
  p4est_locidx_t      li;
  p4est_tree_t       *tree;

  P4EST_ASSERT (p4est == g->p4est);
  P4EST_ASSERT (g->pcw_const->elem_count ==
                (size_t) p4est->local_num_quadrants);

  tree = p4est_tree_array_index (p4est->trees, which_tree);
  zz = sc_array_position (&tree->quadrants, quadrant);
  li = tree->quadrants_offset + (p4est_locidx_t) zz;
  P4EST_ASSERT (0 <= li && li < p4est->local_num_quadrants);

  *result = *(double *) sc_array_index_int (g->pcw_const, li);
}

static void
ospex_setup_params (ospex_global_t * g)
{
  int                 k;

  g->center[0] = .1;
  g->center[1] = .2;
  g->center[2] = .4;

  g->halfax[0] = 1.2;
  g->halfax[1] = .6;
  g->halfax[2] = 1.5;
  g->halfmax = 0.;
  for (k = 0; k < 3; ++k) {
    g->halfmax = SC_MAX (g->halfmax, g->halfax[k]);
  }
}

static double
ospex_distance_ellipsoid (ospex_global_t * g, const double vxyz[3])
{
  int                 k;
  double              a, d;

  /* compute normalized distance of given point to ellipsoid */
  d = 0.;
  for (k = 0; k < 3; ++k) {
    a = (vxyz[k] - g->center[k]) / g->halfax[k];
    d += a * a;
  }
  return std::fabs (std::sqrt (d) - 1.0);
}

static int
ospex_refine_fn (p4est_t * p4est,
                 p4est_topidx_t which_tree, p4est_quadrant_t * quadrant)
{
  ospex_global_t     *g = (ospex_global_t *) p4est->user_pointer;
  int                 c;
  int                 k;
  double              a, d;
  double              closest, maxd;
  double              vxyz[3], cxyz[3];
  p4est_quadrant_t    corner;

  /* compute center location */
  p4est_ospray_quadrant_center (p4est, which_tree, quadrant, cxyz);

  /* loop over corner locations */
  maxd = 0.;
  closest = 1.;
  for (c = 0; c < P4EST_CHILDREN; ++c) {
    /* compute world coordinates of corner */
    p4est_quadrant_corner_node (quadrant, c, &corner);
    p4est_qcoord_to_vertex (p4est->connectivity, which_tree,
                            corner.x, corner.y,
#ifdef P4_TO_P8
                            corner.z,
#endif
                            vxyz);

    /* compute distance of corner to center */
    d = 0.;
    for (k = 0; k < 3; ++k) {
      a = vxyz[k] - cxyz[k];
      d += a * a;
    }
    maxd = SC_MAX (maxd, d);

    /* compute distance of corner to ellipsoid */
    d = ospex_distance_ellipsoid (g, vxyz);
    closest = SC_MIN (closest, d);
  }
  maxd = std::sqrt (maxd);

  /* refine if we're closer than half the element diameter */
  return closest * g->halfmax <= g->nearness * maxd;
}

static void
ospex_create_mesh (ospex_global_t * g)
{
  int                 lev;
  char                namebuf[BUFSIZ];
  p4est_gloidx_t      gpre;

  /* create initial forest */
#ifndef P4_TO_P8
  g->conn = !g->tilt ? p4est_connectivity_new_brick (2, 2, 0, 0) :
    p4est_connectivity_new_corner ();
#else
  g->conn = !g->tilt ? p4est_connectivity_new_brick (2, 2, 1, 0, 0, 0) :
    p8est_connectivity_new_rotcubes ();
#endif
  g->p4est = p4est_new_ext (g->mpicomm, g->conn, 0, g->minlevel, 1,
                            0, NULL, g);
  if (g->write_vtk) {
    snprintf (namebuf, BUFSIZ, "%s_%d_%d_%s_%d",
              g->prefix, g->minlevel, g->maxlevel, "new", g->minlevel);
    p4est_vtk_write_file (g->p4est, NULL, namebuf);
  }

  /* refine to reach maximum desired level */
  for (lev = g->minlevel; lev < g->maxlevel; ++lev) {
    P4EST_GLOBAL_PRODUCTIONF ("Refining forest from level %d\n", lev);
    gpre = g->p4est->global_num_quadrants;
    p4est_refine (g->p4est, 0, ospex_refine_fn, NULL);
    if (gpre == g->p4est->global_num_quadrants) {
      /* no refinement occurred */
      break;
    }
    p4est_balance (g->p4est, P4EST_CONNECT_FULL, NULL);
    p4est_partition (g->p4est, 0, NULL);

    /* output next mesh */
    if (g->write_vtk) {
      snprintf (namebuf, BUFSIZ, "%s_%d_%d_%s_%d",
                g->prefix, g->minlevel, g->maxlevel, "new", lev + 1);
      p4est_vtk_write_file (g->p4est, NULL, namebuf);
    }
  }
  P4EST_GLOBAL_PRODUCTIONF ("Refinement stopped with level %d\n", lev);
  g->curlevel = lev;
}

static void
ospex_create_data (ospex_global_t * g)
{
  p4est_t            *p4est = g->p4est;
  int                 retval;
  size_t              zz;
  double              d;
  double              cxyz[3];
  char                namebuf[BUFSIZ];
  p4est_topidx_t      jt;
  p4est_locidx_t      li, lend;
  p4est_tree_t       *tree;
  p4est_quadrant_t   *quadrant;
  p4est_vtk_context_t *vtkc;
  sc_array_t         *pc;

  /* create cell-centered constant data */
  pc = g->pcw_const = sc_array_new_count
    (sizeof (double), lend = p4est->local_num_quadrants);

  /* loop over local quadrants */
  li = 0;
  for (jt = p4est->first_local_tree; jt <= p4est->last_local_tree; ++jt) {
    tree = p4est_tree_array_index (p4est->trees, jt);
    for (zz = 0; zz < tree->quadrants.elem_count; ++zz, ++li) {
      quadrant = p4est_quadrant_array_index (&tree->quadrants, zz);

      /* compute value at quadrant center */
      p4est_ospray_quadrant_center (p4est, jt, quadrant, cxyz);
      d = ospex_distance_ellipsoid (g, cxyz);
      *(double *) sc_array_index_int (pc, li) = SC_MIN (d, 1.);
    }
  }

  /* write the data out to VTK */
  if (g->write_vtk) {
    snprintf (namebuf, BUFSIZ, "%s_%d_%d_%s_%d",
              g->prefix, g->minlevel, g->maxlevel, "con", g->curlevel);
    vtkc = p4est_vtk_context_new (p4est, namebuf);
    vtkc = p4est_vtk_write_header (vtkc);
    SC_CHECK_ABORT (vtkc != NULL, "Error VTK writing header");
    vtkc = p4est_vtk_write_cell_dataf
      (vtkc, 1, 1, 1, g->mpiwrap, 1, 0, "pcw_const", pc, vtkc);
    SC_CHECK_ABORT (vtkc != NULL, "Error VTK writing cell data");
    retval = p4est_vtk_write_footer (vtkc);
    SC_CHECK_ABORT (!retval, "Error VTK writing footer");
  }
}

static void
ospex_clear_all (ospex_global_t * g)
{
  sc_array_destroy (g->pcw_const);

  p4est_destroy (g->p4est);
  p4est_connectivity_destroy (g->conn);
}

static box3f worldBounds;

int main(int argc, char **argv) {
  int mpiThreadCapability = 0;
  MPI_Init_thread(&argc, &argv, MPI_THREAD_MULTIPLE, &mpiThreadCapability);
  if (mpiThreadCapability != MPI_THREAD_MULTIPLE &&
      mpiThreadCapability != MPI_THREAD_SERIALIZED) {
    std::cout <<  "OSPRay requires the MPI runtime to support thread "
      << "multiple or thread serialized.\n";
    return 1;
  }

  ospex_global_t      sglobal, *g = &sglobal;

  /*---------------- initialize MPI and p4est logging ----------------*/

  //mpiret = sc_MPI_Init (&argc, &argv);
  //SC_CHECK_MPI (mpiret);

  memset (g, 0, sizeof (ospex_global_t));
  g->mpicomm = MPI_COMM_WORLD;
  sc_init (g->mpicomm, 1, 1, NULL, SC_LP_DEFAULT);
  p4est_init (NULL, SC_LP_DEFAULT);
  g->options = sc_options_new (argv[0]);

  /*------------------ process command line and run ------------------*/

  ospex_parse_options (g, argc, argv);
  ospex_setup_params (g);
  ospex_create_mesh (g);
  ospex_create_data (g);

  int mpiRank      = 0;
  int mpiWorldSize = 0;
  MPI_Comm_rank(MPI_COMM_WORLD, &mpiRank);
  MPI_Comm_size(MPI_COMM_WORLD, &mpiWorldSize);

  std::cout << "OSPRay rank " << mpiRank << "/" << mpiWorldSize << "\n";

  // load the MPI module, and select the MPI distributed device. Here we
  // do not call ospInit, as we want to explicitly pick the distributed
  // device. This can also be done by passing --osp:mpi-distributed when
  // using ospInit, however if the user doesn't pass this argument your
  // application will likely not behave as expected
  ospLoadModule("mpi");
  ospLoadModule("mpi_distributed");
  ospLoadModule("p4est");

  OSPDevice mpiDevice = ospNewDevice("mpi_distributed");
  ospDeviceCommit(mpiDevice);
  ospSetCurrentDevice(mpiDevice);

  // set an error callback to catch any OSPRay errors and exit the application
  ospDeviceSetErrorFunc(ospGetCurrentDevice(),
    [](OSPError error, const char *errorDetails) {
      std::cerr << "OSPRay error: " << errorDetails << std::endl;
      exit(error);
    });

  p4est_topidx_t total_trees, first_local_tree, last_local_tree;
  p4est_ospray_tree_counts(g->p4est, &total_trees, &first_local_tree, &last_local_tree);
  std::cout << "Rank " << mpiRank << " has trees [" << first_local_tree
    << ", " << last_local_tree << "] of the total " << total_trees << " trees\n";

  // This outer rank serialization loop is just for debugging
  for (int r = 0; r < mpiWorldSize; ++r) {
    if (r == mpiRank) {
      for (auto i = first_local_tree; i <= last_local_tree; ++i) {
        std::cout << "Rank " << mpiRank << " tree " << i << "\n";
        double tree_aabb[6] = {0.f};
        p4est_ospray_tree_aabb(g->p4est, i, tree_aabb);
        std::cout << " AABB: {(" << tree_aabb[0] << ", " << tree_aabb[1] << ", " << tree_aabb[2]
          << "), (" << tree_aabb[3] << ", " << tree_aabb[4] << ", " << tree_aabb[5] << ")}\n";

        sc_array_t *coarse_quadrants = sc_array_new(sizeof(p4est_quadrant_t));
        p4est_ospray_local_coarsest(g->p4est, i, NULL, coarse_quadrants);
        std::cout << "Rank " << mpiRank << " # of coarse quadrants in " << i
          << ": " << coarse_quadrants->elem_count << "\n";

        sc_array_destroy(coarse_quadrants);
      }
      std::cout << std::flush;
    }
    MPI_Barrier(MPI_COMM_WORLD);
  }

  // TODO: aggregate together the world bounds
  worldBounds = box3f(vec3f(0.f), vec3f(2.f));

  // TODO: One volume per-tree, and one model per-convex region from Carsten's
  // convex region list.
  // create the "world" model which will contain all of our geometries
  std::vector<OSPModel> models{ospNewModel()};

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
  const std::vector<float> opacities = {0.1, 0.8};
  OSPData colorsData = ospNewData(colors.size(), OSP_FLOAT3, colors.data());
  ospCommit(colorsData);
  OSPData opacityData = ospNewData(opacities.size(), OSP_FLOAT, opacities.data());
  ospCommit(opacityData);

  const vec2f valueRange(0.f, 1.f);
  ospSetData(transferFcn, "colors", colorsData);
  ospSetData(transferFcn, "opacities", opacityData);
  ospSet2f(transferFcn, "valueRange", valueRange.x, valueRange.y);
  ospCommit(transferFcn);

  std::cout << "p4est ptr: " << g->p4est << "\n";
  for (int i = first_local_tree; i <= last_local_tree; ++i) {
    OSPVolume tree = ospNewVolume("p4est");
    ospSetVoidPtr(tree, "p4estTree", (void*)g->p4est);
    ospSetVoidPtr(tree, "p4estDataCallback", (void*)ospex_data_callback);
    ospSet1i(tree, "treeID", i);
    ospSetObject(tree, "transferFunction", transferFcn);
    ospCommit(tree);
    ospAddVolume(models[0], tree);
    ospRelease(tree);

    ospSet1i(models[0], "id", 0);
    // override the overall volume bounds to clip off the ghost voxels, so
    // they are just used for interpolation
    //ospSet3fv(models[0], "region.lower", &bricks[i].bounds.lower.x);
    //ospSet3fv(models[0], "region.upper", &bricks[i].bounds.upper.x);
    ospCommit(models[0]);
  }

  // create OSPRay renderer
  OSPRenderer renderer = ospNewRenderer("mpi_raycast");

  OSPLight ambientLight = ospNewLight("ambient");
  ospCommit(ambientLight);
  OSPData lightData = ospNewData(1, OSP_LIGHT, &ambientLight, 0);
  ospCommit(lightData);
  ospSetObject(renderer, "lights", lightData);
  ospRelease(lightData);

  // create a GLFW OSPRay window: this object will create and manage the OSPRay
  // frame buffer and camera directly
  auto glfwOSPRayWindow =
      std::unique_ptr<GLFWDistribP4estWindow>(new GLFWDistribP4estWindow(
          vec2i{1024, 768}, worldBounds, models, renderer));

  // UI Example code
  int spp        = 1;
  int currentSpp = 1;
  if (mpiRank == 0) {
    glfwOSPRayWindow->registerImGuiCallback(
        [&]() { ImGui::SliderInt("spp", &spp, 1, 64); });
  }

  glfwOSPRayWindow->registerDisplayCallback([&](GLFWDistribP4estWindow *win) {
    // Send the UI changes out to the other ranks so we can synchronize
    // how many samples per-pixel we're taking
    MPI_Bcast(&spp, 1, MPI_INT, 0, MPI_COMM_WORLD);
    if (spp != currentSpp) {
      currentSpp = spp;
      ospSet1i(renderer, "spp", spp);
      win->addObjectToCommit(renderer);
    }
  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  // cleanup remaining objects
  for (auto &m : models) {
    ospRelease(m);
  }
  ospRelease(renderer);

  // cleanly shut OSPRay down
  ospShutdown();

  ospex_clear_all (g);
  sc_options_destroy (g->options);
  sc_finalize ();

  MPI_Finalize();

  return 0;
}

bool computeDivisor(int x, int &divisor) {
  int upperBound = std::sqrt(x);
  for (int i = 2; i <= upperBound; ++i) {
    if (x % i == 0) {
      divisor = i;
      return true;
    }
  }
  return false;
}

// Compute an X x Y x Z grid to have 'num' grid cells,
// only gives a nice grid for numbers with even factors since
// we don't search for factors of the number, we just try dividing by two
vec3i computeGrid(int num) {
  vec3i grid(1);
  int axis    = 0;
  int divisor = 0;
  while (computeDivisor(num, divisor)) {
    grid[axis] *= divisor;
    num /= divisor;
    axis = (axis + 1) % 3;
  }
  if (num != 1) {
    grid[axis] *= num;
  }
  return grid;
}

VolumeBrick makeLocalVolume(const int mpiRank, const int mpiWorldSize) {
  const vec3i grid = computeGrid(mpiWorldSize);
  const vec3i brickId(mpiRank % grid.x,
                      (mpiRank / grid.x) % grid.y,
                      mpiRank / (grid.x * grid.y));
  // The bricks are 64^3 + 1 layer of ghost voxels on each axis
  const vec3i brickVolumeDims = vec3i(64);
  const vec3i brickGhostDims  = vec3i(brickVolumeDims + 2);

  // The grid is over the [0, grid * brickVolumeDims] box
  worldBounds            = box3f(vec3f(0.f), vec3f(grid * brickVolumeDims));
  const vec3f brickLower = brickId * brickVolumeDims;
  const vec3f brickUpper = brickId * brickVolumeDims + brickVolumeDims;

  VolumeBrick brick;
  brick.bounds = box3f(brickLower, brickUpper);
  // we just put ghost voxels on all sides here, but a real application
  // would change which faces of each brick have ghost voxels dependent
  // on the actual data
  brick.ghostBounds = box3f(brickLower - vec3f(1.f), brickUpper + vec3f(1.f));

  brick.brick = ospNewVolume("block_bricked_volume");

  // ospSet1f(brick.brick, "samplingRate", 0.25f);
  ospSetString(brick.brick, "voxelType", "uchar");
  ospSet3iv(brick.brick, "dimensions", &brickGhostDims.x);
  // we use the grid origin to place this brick in the right position inside
  // the global volume
  ospSet3fv(brick.brick, "gridOrigin", &brick.ghostBounds.lower.x);

  // generate the volume data to just be filled with this rank's id
  const size_t nVoxels = brickGhostDims.x * brickGhostDims.y * brickGhostDims.z;
  std::vector<char> volumeData(nVoxels, static_cast<char>(mpiRank ));
  ospSetRegion(brick.brick,
               volumeData.data(),
               osp_vec3i{0, 0, 0},
               (osp_vec3i &)brickGhostDims);

  return brick;
}
