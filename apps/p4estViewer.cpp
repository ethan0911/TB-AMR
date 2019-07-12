#include <stdint.h>
#include <stdio.h>
#ifdef _WIN32
#include <malloc.h>
#else
#include <alloca.h>
#endif
#include <imgui.h>
#include <memory>
#include <random>

#include "GLFWOSPRayWindow.h"
#include "ospray/common/OSPCommon.h"
#include "ospray/ospray.h"

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


std::string intputDataType;
FileName inputFile;
std::string inputField;
std::vector<std::string> inputMesh;
bool showMesh = false;

enum class DataRep { unstructured, octree };

struct BenchmarkInfo {
  bool benchmarkMode = false;
  int cellBytes = -1;
  std::string camParamPath;
  int numTrials = -1;
  int numWarmupFrames = -1;
  std::string subdirName;
  DataRep currDataRep;
};

// Overload << operator to represent BenchmarkInfo and DataRep as strings.
// This makes debug printing easier.
std::ostream& operator<<(std::ostream &strm, const DataRep &dr) {
  if(dr == DataRep::octree){
    return strm << "Octree";
  } else if(dr == DataRep::unstructured) {
    return strm << "Unstructured";
  } else {
    throw std::logic_error("Unrecognized DataRep!");
  }
}

std::ostream& operator<<(std::ostream &strm, const BenchmarkInfo &bi) {
  return strm << "Bytes per cell: " << bi.cellBytes << std::endl
              << "Cam param file: " << bi.camParamPath << std::endl
              << "# trials: " << bi.numTrials << std::endl
              << "# warmup frames: " << bi.numWarmupFrames << std::endl
              << "Subdirectory to create: " << bi.subdirName << std::endl
              << "Data representation: " << bi.currDataRep << std::endl;
}

void parseCommandLine(int &ac, const char **&av, BenchmarkInfo& benchInfo, bool& enableTFwidget)
{
  for (int i = 1; i < ac; ++i) {
    const std::string arg = av[i];
    if (arg == "-t" || arg == "--type") {
      intputDataType = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-i" || arg == "--input") {
      inputFile = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "--use-tf-widget") {
      enableTFwidget = true;
      removeArgs(ac, av, i, 1);
      --i;
    } else if (arg == "-b" || arg == "--benchmark") {
      benchInfo.benchmarkMode = true;

      std::string bench_config_str = av[i + 1];
      //std::cout << "Benchmark config string: " << bench_config_str << std::endl;

      // Code snippet from: https://www.geeksforgeeks.org/tokenizing-a-string-cpp/
      // Vector of string to save tokens
      vector <string> tokens;

      // stringstream class check1
      stringstream bench_config_sstream(bench_config_str);

      // Tokenizing w.r.t. space ' '
      string intermediate;
      while(getline(bench_config_sstream, intermediate, ' '))
      {
        tokens.push_back(intermediate);
      }

      std::cout << "Benchmark config string tokens: ";
      for(std::string t : tokens){
        std::cout << t << " " << std::endl;
      }
      std::cout << std::endl;

      // TODO: Come up with a more robust solution than relying on order in sequence.
      // Maybe use key/value pairs
      benchInfo.cellBytes = std::atoi(tokens[0].c_str());
      benchInfo.camParamPath = tokens[1];
      benchInfo.numTrials = std::atoi(tokens[2].c_str());
      benchInfo.numWarmupFrames = std::atoi(tokens[3].c_str());
      benchInfo.subdirName = tokens[4];
      std::string dataRepName = tokens[5];

      if (dataRepName.compare("unstructured") == 0) {
        benchInfo.currDataRep = DataRep::unstructured;
      } else if (dataRepName.compare("octree") == 0){
        benchInfo.currDataRep = DataRep::octree;
      } else {
        throw std::domain_error("Invalid data representation!");
      }

      std::cout << benchInfo;

      removeArgs(ac, av, i, 2);
      --i;
    } else{
      showMesh = true;
      inputMesh.push_back(std::string(av[i + 1]));
      removeArgs(ac, av, i, 1);
      --i;
    }
  }
}

int main(int argc, const char **argv)
{

  bool useTFwidget = false;

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

  BenchmarkInfo bInfo;
  parseCommandLine(argc, argv, bInfo, useTFwidget);

  //! Set up us the transfer function*******************************************
  OSPTransferFunction transferFcn = ospNewTransferFunction("piecewise_linear");
  /*
   *const std::vector<vec3f> colors = {vec3f(0, 0, 0.563),
   *                                   vec3f(0, 0, 1),
   *                                   vec3f(0, 1, 1),
   *                                   vec3f(0.5, 1, 0.5),
   *                                   vec3f(1, 1, 0),
   *                                   vec3f(1, 0, 0),
   *                                   vec3f(0.5, 0, 0)};
   */
  //const std::vector<float> opacities = {1.f, 1.f};

  // The below set of colors/opacities should in theory match ParaView's default
  // transfer function
  const std::vector<vec3f> colors = {
    vec3f(0.23137254902000001f, 0.298039215686f, 0.75294117647100001f),
    vec3f(0.86499999999999999, 0.86499999999999999, 0.86499999999999999),
    vec3f(0.70588235294099999, 0.015686274509800001, 0.149019607843)
  };
  const std::vector<float> opacities = {0.f, 1.f};
  OSPData colorsData = ospNewData(colors.size(), OSP_FLOAT3, colors.data());
  ospCommit(colorsData);
  OSPData opacityData =
      ospNewData(opacities.size(), OSP_FLOAT, opacities.data());
  ospCommit(opacityData);

  // The value range here will be different from Will's code. It will need to
  // match Timo's data.
  vec2f valueRange(0.0f, 1.f);


  OSPWorld world = ospNewWorld();

  // TODO: compute world bounds or read it from p4est
  box3f universeBounds(vec3f(0.f), vec3f(1.f));

  std::vector<std::shared_ptr<VoxelOctree>> voxelOctrees;

  std::shared_ptr<DataSource> pData = NULL;

  // mmap the binary file
  char octreeFileName[10000];

  if (intputDataType == "p4est") {
    pData = std::make_shared<p4estSource>();
    pData->mapMetaData(inputFile.str());
    universeBounds = box3f(vec3f(-0.3f), vec3f(1.3f));
    valueRange     = vec2f(0.f, 1.0f); //HACK! Currently hardcoded for Mandelbrot set.
  }

  if (intputDataType == "synthetic") {
    pData = std::make_shared<syntheticSource>();
    pData->mapMetaData(inputFile.str());
    universeBounds = box3f(vec3f(0.f), vec3f(4.f));
    valueRange     = vec2f(0.f, 12.f);
  }

  // NASA exajet data
  if (intputDataType == "exajet") {
    pData         = std::make_shared<exajetSource>(inputFile, inputField);

    pData->mapMetaData(inputFile.str());
    universeBounds = box3f(pData->gridOrigin, pData->gridWorldSpace * pData->dimensions) + pData->worldOrigin;
    valueRange = vec2f(-10.0f, 20.0f);  // y_vorticity.bin
  }

  ospSetData(transferFcn, "colors", colorsData);
  ospSetData(transferFcn, "opacities", opacityData);
  ospSet2f(transferFcn, "valueRange", valueRange.x, valueRange.y);
  ospCommit(transferFcn);

  Mesh mesh;
  affine3f transform =
      affine3f::translate(vec3f(0.f)) * affine3f::scale(vec3f(1.f));

  time_point t1;
  if (showMesh) {
    t1 = Time();
    OSPMaterial objMaterial = ospNewMaterial("scivis", "OBJMaterial");
    ospSet3f(objMaterial, "Kd", 3 / 255.f, 10 / 255.f, 25 / 255.f);
    ospSet3f(objMaterial, "Ks", 77 / 255.f, 77 / 255.f, 77 / 255.f);
    ospSet1f(objMaterial, "Ns", 10.f);
    ospCommit(objMaterial);
    mesh.LoadMesh(inputMesh);
    mesh.SetTransform(transform);
    mesh.AddToModel(world, objMaterial);
    double loadMeshTime = Time(t1);
    std::cout<<green<<"Loading Mesh takes " << loadMeshTime << " s" << reset <<"\n";
  }

  std::shared_ptr<VoxelOctree> voxelAccel = std::make_shared<VoxelOctree>();

  t1 = Time();
  sprintf(octreeFileName, "%s%06i.oct", inputFile.str().c_str(), 0);
  std::string octFile(octreeFileName);
  voxelAccel->mapOctreeFromFile(octFile);
  double loadTime = Time(t1);
  std::cout << yellow << "Loading " <<voxelAccel->_octreeNodes.size() << " octree nodes from file takes " << loadTime << " s" << reset << "\n";
  voxelOctrees.push_back(voxelAccel);

  //voxelAccel->printOctree();

  OSPVolume tree = ospNewVolume("p4est");
  if (intputDataType == "synthetic")
    ospSet1f(tree, "samplingRate", 16.f);
  if (intputDataType == "p4est")
    ospSet1f(tree, "samplingRate", 16.f);

  if (intputDataType == "exajet")
    ospSet1f(tree, "samplingRate", 0.125f);

  // pass exajet data and metadata, only for one tree right now
  ospSet3f(tree, "worldOrigin", pData->worldOrigin.x, pData->worldOrigin.y, pData->worldOrigin.z);
  ospSet3f(tree,"gridOrigin", pData->gridOrigin.x, pData->gridOrigin.y, pData->gridOrigin.z);
  ospSet3f(tree, "gridWorldSpace", pData->gridWorldSpace.x, pData->gridWorldSpace.y, pData->gridWorldSpace.z);
  ospSet3i(tree, "dimensions", pData->dimensions.x, pData->dimensions.y, pData->dimensions.z);

  ospSetVoidPtr(tree, "voxelOctree", (void *)voxelOctrees[voxelOctrees.size() - 1].get());
  ospSet1i(tree, "gradientShadingEnabled", 0);
  // ospSet1i(tree, "adaptiveSampling", 0);

  ospSetObject(tree, "transferFunction", transferFcn);
  ospCommit(tree);
  ospAddVolume(world, tree);
  ospRelease(tree);

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


  //**************************************
  //TODO: Make the below block contingent on the transfer function editor being enabled.
  std::vector<float> colors_tfn;
  std::vector<float> opacities_tfn;
  vec2f valueRange_tfn;

  std::mutex lock;
  if(useTFwidget){
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
  }

  bool isTFNWidgetShow = false;
  //End of block
  //**************************************


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


  //*************************************
  //TODO: Make the below block contingent on the transfer function editor being enabled.
   if (useTFwidget && transferFcn != nullptr) {
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
  //end of block
  //**************************************

  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  ospRelease(renderer);
  // cleanly shut OSPRay down
  ospShutdown();

  return 0;
}
