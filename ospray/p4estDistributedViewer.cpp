#include <iterator>
#include <vector>
#include <memory>
#include <random>
#include <mpi.h>
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

static box3f worldBounds;

// Generate the rank's local volume brick
VolumeBrick makeLocalVolume(const int mpiRank, const int mpiWorldSize);

int main(int argc, char **argv) {
  int mpiThreadCapability = 0;
  MPI_Init_thread(&argc, &argv, MPI_THREAD_MULTIPLE, &mpiThreadCapability);
  if (mpiThreadCapability != MPI_THREAD_MULTIPLE &&
      mpiThreadCapability != MPI_THREAD_SERIALIZED) {
    std::cout <<  "OSPRay requires the MPI runtime to support thread "
      << "multiple or thread serialized.\n";
    return 1;
  }

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
  ospLoadModule("p4est");

  OSPDevice mpiDevice = ospNewDevice("mpi_distributed");
  ospDeviceCommit(mpiDevice);
  ospSetCurrentDevice(mpiDevice);

  // set an error callback to catch any OSPRay errors and exit the application
  ospDeviceSetErrorFunc(
      ospGetCurrentDevice(), [](OSPError error, const char *errorDetails) {
        std::cerr << "OSPRay error: " << errorDetails << std::endl;
        exit(error);
      });

  // create the "world" model which will contain all of our geometries
  std::vector<OSPModel> models{ ospNewModel(), ospNewModel() };

  // Each rank specifies two bricks and two models.
  // TODO WILL: In the future these wil be one volume per-tree, and one
  // model per-convex region from Carsten's convex region list.
  std::array<VolumeBrick, 2> bricks = {
    makeLocalVolume(mpiRank * 2, mpiWorldSize * 2),
    makeLocalVolume(mpiRank * 2 + 1, mpiWorldSize * 2),
  };

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
  const std::vector<float> opacities = {0.01, 0.1};
  OSPData colorsData = ospNewData(colors.size(), OSP_FLOAT3, colors.data());
  ospCommit(colorsData);
  OSPData opacityData = ospNewData(opacities.size(), OSP_FLOAT, opacities.data());
  ospCommit(opacityData);

  const vec2f valueRange(-0.5f, mpiWorldSize * 2);
  ospSetData(transferFcn, "colors", colorsData);
  ospSetData(transferFcn, "opacities", opacityData);
  ospSet2f(transferFcn, "valueRange", valueRange.x, valueRange.y);
  ospCommit(transferFcn);

  for (size_t i = 0; i < models.size(); ++i) {
    ospSetObject(bricks[i].brick, "transferFunction", transferFcn);
    ospCommit(bricks[i].brick);
    ospAddVolume(models[i], bricks[i].brick);
    ospRelease(bricks[i].brick);

    ospSet1i(models[i], "id", mpiRank * 2 + i);
    // override the overall volume bounds to clip off the ghost voxels, so
    // they are just used for interpolation
    ospSet3fv(models[i], "region.lower", &bricks[i].bounds.lower.x);
    ospSet3fv(models[i], "region.upper", &bricks[i].bounds.upper.x);
    ospCommit(models[i]);
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
