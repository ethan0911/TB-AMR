#include <stdint.h>
#include <stdio.h>
#ifdef _WIN32
#include <malloc.h>
#else
#include <alloca.h>
#endif

#include <imgui.h>
#include <iterator>
#include <memory>
#include <random>

#include "GLFWOSPRayWindow.h"
#include "ospray/common/OSPCommon.h"
#include "ospray/ospray.h"

#include <p4est_to_p8est.h>

/* p4est has two separate interfaces for 2D and 3D, p4est*.h and p8est*.h.
 * Most API functions are available for both dimensions.  The header file
 * p4est_to_p8est.h #define's the 2D names to the 3D names such that most code
 * only needs to be written once.  In this example, we rely on this. */
#ifndef P4_TO_P8
#include <p4est_extended.h>
#include <p4est_ospray.h>
#include <p4est_vtk.h>
#else
#include <p8est_extended.h>
#include <p8est_ospray.h>
#include <p8est_vtk.h>
#endif

#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "../ospray/DataQueryCallBack.h"
#include "../ospray/VoxelOctree.h"
#include "dataImporter.h"
#include "loader/meshloader.h"
#include "widgets/TransferFunctionWidget.h"

#include "Utils.h"

using namespace ospcommon;

// From http://www.martinbroadhurst.com/how-to-split-a-string-in-c.html
template <class Container>
void split_string(const std::string &str, Container &cont, char delim = ' ')
{
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delim)) {
    cont.push_back(token);
  }
}

std::string intputDataType;
FileName inputFile;
std::string inputField;
std::vector<std::string> inputMesh;
bool showMesh = false;

void parseCommandLind(int &ac, const char **&av)
{
  for (int i = 1; i < ac; ++i) {
    const std::string arg = av[i];
    if (arg == "-t" || arg == "--type") {
      intputDataType = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-d" || arg == "--data") {
      inputFile = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-f" || arg == "--field") {
      inputField = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    }else{
      showMesh = true;
      inputMesh.push_back(std::string(av[i + 1]));
      removeArgs(ac, av, i, 1);
      --i;
    }
  }

  // PRINT(inputMesh[0]);
  // PRINT(inputMesh.size());
  // PRINT(inputFile.str());
}

int main(int argc, const char **argv)
{
  // if( argc != 2 ){
  //   std::cout << "Usage: p4estViewer <path_to_p4est_file>" << std::endl;
  //   exit(1);
  // }

  //! initialize OSPRay; e.g. "--osp:debug"***********************
  OSPError initError = ospInit(&argc, (const char **)argv);

  if (initError != OSP_NO_ERROR)
    return initError;

  // set an error callback to catch any OSPRay errors and exit the application
  ospDeviceSetErrorFunc(
      ospGetCurrentDevice(), [](OSPError error, const char *errorDetails) {
        std::cerr << "OSPRay error: " << errorDetails << std::endl;
        exit(error);
      });

  // Load our custom OSPRay volume types from the module
  ospLoadModule("p4est");

  parseCommandLind(argc, argv);

  //! Set up us the transfer function*******************************************
  OSPTransferFunction transferFcn = ospNewTransferFunction("piecewise_linear");
  const std::vector<vec3f> colors = {vec3f(0, 0, 0.563),
                                     vec3f(0, 0, 1),
                                     vec3f(0, 1, 1),
                                     vec3f(0.5, 1, 0.5),
                                     vec3f(1, 1, 0),
                                     vec3f(1, 0, 0),
                                     vec3f(0.5, 0, 0)};
  const std::vector<float> opacities = {1.f, 1.f};
  OSPData colorsData = ospNewData(colors.size(), OSP_FLOAT3, colors.data());
  ospCommit(colorsData);
  OSPData opacityData =
      ospNewData(opacities.size(), OSP_FLOAT, opacities.data());
  ospCommit(opacityData);

  // The value range here will be different from Will's code. It will need to
  // match Timo's data.
  vec2f valueRange(0.0f, 1.f);

  ospSetData(transferFcn, "colors", colorsData);
  ospSetData(transferFcn, "opacities", opacityData);
  ospSet2f(transferFcn, "valueRange", valueRange.x, valueRange.y);
  ospCommit(transferFcn);

  OSPWorld world = ospNewWorld();

  // TODO: compute world bounds or read it from p4est
  box3f universeBounds(vec3f(0.f), vec3f(1.f));

  std::vector<VoxelOctree*> voxelOctrees;

  std::shared_ptr<DataSource> pData = NULL;

  //! load P4est data*******************************************
  int mpiret;
  // int                 recursive, partforcoarsen, balance;
  sc_MPI_Comm mpicomm;
  p4est_t *p4est;
  p4est_connectivity_t *conn;

  if (intputDataType == "p4est") {
    /* Initialize MPI; see sc_mpi.h.
     * If configure --enable-mpi is given these are true MPI calls.
     * Else these are dummy functions that simulate a single-processor run. */
    char **av = (char **)argv;
    mpiret    = sc_MPI_Init(&argc, &av);
    SC_CHECK_MPI(mpiret);
    mpicomm = sc_MPI_COMM_WORLD;

    /* These functions are optional.  If called they store the MPI rank as a
     * static variable so subsequent global p4est log messages are only issued
     * from processor zero.  Here we turn off most of the logging; see sc.h. */
    sc_init(mpicomm, 1, 1, NULL, SC_LP_ESSENTIAL);
    p4est_init(NULL, SC_LP_PRODUCTION);

    // See p4est_extended.h
    int autopartition       = 1;
    int broadcasthead       = 1;
    int *user_ptr           = NULL;
    std::string input_fname = std::string(argv[4]);

    // Read info file. Use this file to decide if we want to load data, and if
    // so, how much.
    std::string info_fname = input_fname + ".info";
    std::ifstream info_fstream(info_fname, std::ios::in);

    // For now, ASSUME that if there is no .info file, there is no data to load.
    // This assumption is probalby okay for Timo's data, but it does not
    // necessarily hold in the general case.
    int num_bytes = -1;
    if (info_fstream.is_open()) {
      std::string currLine;
      // Below is hack that assumes that info files are two lines long, and in
      // the format of Dr. Heister's sample info files located in
      // ../data/cube_*/
      std::getline(info_fstream,
                   currLine);  // first line is a human-readable header
      std::getline(info_fstream,
                   currLine);  // second line contains the info that we want
      info_fstream.close();
      std::vector<std::string> str_tokens;
      split_string<std::vector<std::string>>(currLine, str_tokens);
      num_bytes = std::stoi(str_tokens[2]);
    } else {
      num_bytes = 0;
    }

    int load_data = (num_bytes > 0);

    if (load_data) {
      std::cout << "Loading data..." << std::endl;
    } else {
      std::cout << "No data to load." << std::endl;
    }

    // NATHAN: Read p4est from file.
    p4est = p4est_load_ext(input_fname.c_str(), mpicomm, num_bytes, load_data,
                           autopartition, broadcasthead, user_ptr, &conn);

#if 0
    // p4est traversal

    p4est_iterate(p4est,NULL,NULL,volume_callback, NULL,
#ifdef P4_TO_P8
                  NULL,
#endif
                  NULL);

    OSPData vtxData = ospNewData(verts.size(), OSP_FLOAT3, verts.data());
    OSPData idxData = ospNewData(indices.size(), OSP_INT4, indices.data());
    OSPData cellFieldData =
        ospNewData(cellField.size(), OSP_FLOAT, cellField.data());

    ospCommit(vtxData);
    ospCommit(idxData);
    ospCommit(cellFieldData);

    OSPVolume volume = ospNewVolume("unstructured_volume");
    ospSetData(volume, "vertices", vtxData);
    ospSetData(volume, "cellField", cellFieldData);
    ospSetData(volume, "indices", idxData);

    ospSet1f(volume, "samplingRate", 1.f);
    ospSetObject(volume, "transferFunction", transferFcn);
    // ospSet3f(volume, "volumeClippingBoxLower", 0.0f, 0.0f, 0.0f);
    // ospSet3f(volume, "volumeClippingBoxUpper", 0.5f, 0.5f, 0.5f);
    ospCommit(volume);

    // create the "world" model which will contain all of our geometries
    ospAddVolume(world, volume);
    ospCommit(world);
#else
    // p4est data, voxeloctree traversal

    pData = std::make_shared<p4estSource>(p4est,conn);
    pData->parseData();


    VoxelOctree* voxelAccel = new VoxelOctree(pData->voxels.data(),pData->voxels.size(),
                                box3f(pData->gridOrigin, vec3f(pData->dimensions)),
                                pData->gridWorldSpace);

    voxelOctrees.push_back(voxelAccel);

    p4est_topidx_t total_trees, first_local_tree, last_local_tree;
    p4est_ospray_tree_counts(
        p4est, &total_trees, &first_local_tree, &last_local_tree);
    std::cout << "Have trees [" << first_local_tree << ", " << last_local_tree
              << "] of the total " << total_trees << " trees\n";

    for (int i = first_local_tree; i <= last_local_tree; ++i) {
      OSPVolume tree = ospNewVolume("p4est");
      ospSetVoidPtr(tree, "p4estTree", (void *)p4est);
      ospSet1f(tree, "samplingRate", 1.f);
      // ospSetVoidPtr(tree, "p4estDataCallback", (void *)load_data_callback);
      // ospSet1i(tree, "treeID", i);


      ospSet3f(tree, "gridWorldSpace", pData->gridWorldSpace.x, pData->gridWorldSpace.y, pData->gridWorldSpace.z);
      ospSet3i(tree, "dimensions", pData->dimensions.x, pData->dimensions.y, pData->dimensions.z);
      ospSetVoidPtr(tree, "voxelOctree", (void *)voxelOctrees[voxelOctrees.size() - 1]);

      ospSetObject(tree, "transferFunction", transferFcn);
      ospCommit(tree);
      ospAddVolume(world, tree);
      ospRelease(tree);
    }

    universeBounds = box3f(vec3f(-0.3f), vec3f(1.3f));
    valueRange     = vec2f(0.f, 1.f);

#endif
  }



  if (intputDataType == "synthetic") {
    pData = std::make_shared<syntheticSource>();
    pData->parseData();
    universeBounds = box3f(vec3f(0.f), vec3f(4.f));
    valueRange     = vec2f(0.f, 12.f);
  }

  // NASA exajet data

  if (intputDataType == "exajet") {
    pData         = std::make_shared<exajetSource>(inputFile, inputField);
    time_point t1 = Time();
    pData->parseData();
    double loadTime = Time(t1);
    std::cout << yellow << "Loading time: " << loadTime << " s" << reset << "\n";

    universeBounds = box3f(pData->gridOrigin, pData->gridWorldSpace * pData->dimensions) + pData->worldOrigin;
    valueRange = vec2f(-10.0f, 20.0f);  // y_vorticity.bin
  }


  Mesh mesh;
  affine3f transform =
      affine3f::translate(vec3f(0.f)) * affine3f::scale(vec3f(1.f));

  if (showMesh) {
    OSPMaterial objMaterial = ospNewMaterial("scivis", "OBJMaterial");
    ospSet3f(objMaterial, "Kd", 3 / 255.f, 10 / 255.f, 25 / 255.f);
    ospSet3f(objMaterial, "Ks", 77 / 255.f, 77 / 255.f, 77 / 255.f);
    ospSet1f(objMaterial, "Ns", 10.f);
    ospCommit(objMaterial);
    mesh.LoadMesh(inputMesh);
    mesh.SetTransform(transform);
    mesh.AddToModel(world, objMaterial);
  }

  if (intputDataType == "synthetic" || intputDataType == "exajet") {
    VoxelOctree* voxelAccel = new VoxelOctree(pData->voxels.data(),pData->voxels.size(),
                                box3f(pData->gridOrigin, vec3f(pData->dimensions)),
                                pData->gridWorldSpace);

    voxelOctrees.push_back(voxelAccel);
                              
    OSPVolume tree = ospNewVolume("p4est");
    ospSet1f(tree, "samplingRate", 1.f);
    // pass exajet data and metadata, only for one tree right now
    ospSet3f(tree, "worldOrigin", pData->worldOrigin.x, pData->worldOrigin.y, pData->worldOrigin.z);
    ospSet3f(tree,"gridOrigin", pData->gridOrigin.x, pData->gridOrigin.y, pData->gridOrigin.z);
    ospSet3f(tree, "gridWorldSpace", pData->gridWorldSpace.x, pData->gridWorldSpace.y, pData->gridWorldSpace.z);
    ospSet3i(tree, "dimensions", pData->dimensions.x, pData->dimensions.y, pData->dimensions.z);
    ospSetVoidPtr(tree, "voxelOctree", (void *)voxelOctrees[voxelOctrees.size() - 1]);
    ospSet1i(tree, "gradientShadingEnabled", 0);

    ospSetObject(tree, "transferFunction", transferFcn);
    ospCommit(tree);
    ospAddVolume(world, tree);
    ospRelease(tree);
  }

  ospCommit(world);

  // create and setup an ambient light
  std::array<OSPLight, 2> lights = {ospNewLight("ambient"),
                                    ospNewLight("distant")};
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

  // TransferFunctionWidget
  std::shared_ptr<tfn::tfn_widget::TransferFunctionWidget> widget;
  std::vector<float> colors_tfn;
  std::vector<float> opacities_tfn;
  vec2f valueRange_tfn;
  std::mutex lock;
  if (transferFcn != nullptr) {
    using tfn::tfn_widget::TransferFunctionWidget;
    widget = std::make_shared<TransferFunctionWidget>(
        [&](const std::vector<float> &c,
            const std::vector<float> &a,
            const std::array<float, 2> &r) {
          lock.lock();
          colors_tfn     = std::vector<float>(c);
          opacities_tfn  = std::vector<float>(a);
          valueRange_tfn = vec2f(r[0], r[1]);
          lock.unlock();
        });
    widget->setDefaultRange(valueRange[0], valueRange[1]);
  }

  bool isTFNWidgetShow = false;

  // create a GLFW OSPRay window: this object will create and manage the OSPRay
  // frame buffer and camera directly
  auto glfwOSPRayWindow = std::unique_ptr<GLFWOSPRayWindow>(
      new GLFWOSPRayWindow(vec2i{1024, 768}, universeBounds, world, renderer));

  glfwOSPRayWindow->registerImGuiCallback([&]() {
    static int spp = 1;
    if (ImGui::SliderInt("spp", &spp, 1, 64)) {
      ospSet1i(renderer, "spp", spp);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }

    if (transferFcn != nullptr) {
      if (widget->drawUI(&isTFNWidgetShow)) {
        widget->render(128);
      };

      OSPData colorsData =
          ospNewData(colors_tfn.size() / 3, OSP_FLOAT3, colors_tfn.data());
      ospCommit(colorsData);
      std::vector<float> o(opacities_tfn.size() / 2);
      for (int i = 0; i < opacities_tfn.size() / 2; ++i) {
        o[i] = opacities_tfn[2 * i + 1];
      }
      OSPData opacitiesData = ospNewData(o.size(), OSP_FLOAT, o.data());
      ospCommit(opacitiesData);
      ospSetData(transferFcn, "colors", colorsData);
      ospSetData(transferFcn, "opacities", opacitiesData);
      ospSet2f(transferFcn, "valueRange", valueRange_tfn.x, valueRange_tfn.y);
      glfwOSPRayWindow->addObjectToCommit(transferFcn);
      ospRelease(colorsData);
      ospRelease(opacitiesData);
    }
  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  ospRelease(renderer);
  // cleanly shut OSPRay down
  ospShutdown();

  if (intputDataType == "p4est") {
    /* Destroy the p4est and the connectivity structure. */
    p4est_destroy(p4est);
    p4est_connectivity_destroy(conn);

    /* Verify that allocations internal to p4est and sc do not leak memory.
     * This should be called if sc_init () has been called earlier. */
    sc_finalize();

    /* This is standard MPI programs.  Without --enable-mpi, this is a dummy. */
    mpiret = sc_MPI_Finalize();
    SC_CHECK_MPI(mpiret);
  }

  return 0;
}
