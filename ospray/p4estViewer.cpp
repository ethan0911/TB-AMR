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
#include <p4est_ospray.h>
#else
#include <p8est_vtk.h>
#include <p8est_extended.h>
#include <p8est_ospray.h>
#endif

#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>

#include "VoxelOctree.h"

using namespace ospcommon;

#include "DataQueryCallBack.h"



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
  //P4EST_GLOBAL_PRODUCTIONF
    //("This is the p4est %dD demo example/steps/%s_step1\n",
     //P4EST_DIM, P4EST_STRING);

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

  // Load our custom OSPRay volume types from the module
  ospLoadModule("p4est");


  //See p4est_extended.h
	//int data_size = 0;
	//int load_data = 0;
	int autopartition = 1;
	int broadcasthead = 1;
	int* user_ptr = NULL;
	//char* input_fname = argv[1];
  std::string input_fname = std::string(argv[1]);

  //Read info file. Use this file to decide if we want to load data, and if so, how much.
  std::string info_fname = input_fname + ".info";
  std::ifstream info_fstream(info_fname, std::ios::in);
  
  //For now, ASSUME that if there is no .info file, there is no data to load.
  //This assumption is probalby okay for Timo's data, but it does not necessarily hold in the general case.
  int num_bytes = -1;
  if(info_fstream.is_open()){
    std::string currLine;
    // Below is hack that assumes that info files are two lines long, and in the
    // format of Dr. Heister's sample info files located in ../data/cube_*/
    std::getline(info_fstream, currLine); //first line is a human-readable header
    std::getline(info_fstream, currLine); //second line contains the info that we want
    info_fstream.close();
    std::vector<std::string> str_tokens;
    split_string<std::vector<std::string>>(currLine, str_tokens);
    num_bytes = std::stoi(str_tokens[2]);
  } else {
    num_bytes = 0;
  }

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
  box3f universeBounds(vec3f(0.f), vec3f(1.f));

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
	const std::vector<float> opacities = {1.f, 1.f};
	OSPData colorsData = ospNewData(colors.size(), OSP_FLOAT3, colors.data());
	ospCommit(colorsData);
	OSPData opacityData = ospNewData(opacities.size(), OSP_FLOAT, opacities.data());
	ospCommit(opacityData);

  //The value range here will be different from Will's code. It will need to match Timo's data.
  const vec2f valueRange(0.0f, 1.0f);
  ospSetData(transferFcn, "colors", colorsData);
	ospSetData(transferFcn, "opacities", opacityData);
  ospSet2f(transferFcn, "valueRange", valueRange.x, valueRange.y);
	ospCommit(transferFcn);
  //End transfer function setup
  //*********************************************************

#if 0

	p4est_iterate (p4est,NULL,NULL,volume_callback,NULL,    	
#ifdef P4_TO_P8
					 NULL,  
#endif
					 NULL); 
        
  OSPData vtxData = ospNewData(verts.size(), OSP_FLOAT3, verts.data());
  OSPData idxData = ospNewData(indices.size(), OSP_INT4, indices.data());
  OSPData cellFieldData = ospNewData(cellField.size(), OSP_FLOAT, cellField.data());


  ospCommit(vtxData);
  ospCommit(idxData);
  ospCommit(cellFieldData);

  OSPVolume volume = ospNewVolume("unstructured_volume");
  ospSetData(volume, "vertices", vtxData);
  ospSetData(volume, "cellField", cellFieldData);
  ospSetData(volume, "indices", idxData);

  ospSet1f(volume, "samplingRate", 1.f);
  ospSetObject(volume, "transferFunction", transferFcn);
  //ospSet3f(volume, "volumeClippingBoxLower", 0.0f, 0.0f, 0.0f);
  //ospSet3f(volume, "volumeClippingBoxUpper", 0.5f, 0.5f, 0.5f);
  ospCommit(volume);

  // create the "world" model which will contain all of our geometries
  OSPWorld world = ospNewWorld();
  ospAddVolume(world, volume);
  ospCommit(world);
#else

  // TODO: aggregate together the world bounds
  // universeBounds = box3f(vec3f(0.f), vec3f(4.f));
  universeBounds = box3f(vec3f(-0.3f), vec3f(1.3f));

  // TODO: One volume per-tree, and one model per-convex region from Carsten's
  // convex region list.
  // create the "universe" world  which will contain all of our geometries
  //std::vector<OSPWorld> worlds{ospNewWorld()};

  OSPWorld world = ospNewWorld();

  p4est_topidx_t total_trees, first_local_tree, last_local_tree;
  p4est_ospray_tree_counts(p4est, &total_trees, &first_local_tree, &last_local_tree);
  std::cout << "Have trees [" << first_local_tree
    << ", " << last_local_tree << "] of the total " << total_trees << " trees\n";

  std::cout << "p4est ptr: " << p4est << "\n";
  for (int i = first_local_tree; i <= last_local_tree; ++i) {
    OSPVolume tree = ospNewVolume("p4est");
    ospSetVoidPtr(tree, "p4estTree", (void*)p4est);
    ospSetVoidPtr(tree, "p4estDataCallback", (void*)load_data_callback);
    ospSet1f(tree, "samplingRate", 1.f);
    ospSet1i(tree, "treeID", i);
    ospSetObject(tree, "transferFunction", transferFcn);
    ospCommit(tree);
    ospAddVolume(world, tree);
    ospRelease(tree);

    //ospSet1i(worlds[0], "id", 0);

    // NATHAN: below commented-out block is from Will. May only be needed for
    // the distributed viewer.

    // override the overall volume bounds to clip off the ghost voxels, so
    // they are just used for interpolation
    //ospSet3fv(models[0], "region.lower", &bricks[i].bounds.lower.x);
    //ospSet3fv(models[0], "region.upper", &bricks[i].bounds.upper.x);
    
    //ospCommit(worlds[0]);
  }

  ospCommit(world);

#endif

  // create and setup an ambient light
  std::array<OSPLight, 2> lights = {
    ospNewLight("ambient"),
    ospNewLight("distant")
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
          vec2i{1024, 768}, universeBounds, world, renderer));

  glfwOSPRayWindow->registerImGuiCallback([&]() {
    static int spp = 1;
    if (ImGui::SliderInt("spp", &spp, 1, 64)) {
      ospSet1i(renderer, "spp", spp);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  //the below causes a double free error?
  //for (auto &w : worlds) {
    //ospRelease(w);
  //}

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
