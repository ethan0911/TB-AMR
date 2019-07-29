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
FileName inputData;
FileName inputOctFile;
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
    } else if (arg == "-d" || arg == "--data") {
      inputData = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-i" || arg == "--input-oct") {
      inputOctFile = FileName(av[i + 1]);
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
  if (ospLoadModule("tamr") != OSP_NO_ERROR) {
    throw std::runtime_error("failed to initialize TAMR module");
  }
  if (ospLoadModule("impi") != OSP_NO_ERROR) {
    throw std::runtime_error("failed to initialize IMPI module");
  }

  OSPWorld world = ospNewWorld();
  // create OSPRay renderer
  OSPRenderer renderer = ospNewRenderer("scivis");

  if (!renderer) {
    throw std::runtime_error("invalid renderer name: scivis ");
  }

  BenchmarkInfo bInfo;
  parseCommandLine(argc, argv, bInfo, useTFwidget);

    //! Set up us the transfer function*******************************************
  OSPTransferFunction transferFcn = ospNewTransferFunction("piecewise_linear");

  // The below set of colors/opacities should in theory match ParaView's default
  // transfer function
  vec2f valueRange(0.0f, 1.f);

  const std::vector<vec3f> colorArray = {
    vec3f(0.23137254902000001f, 0.298039215686f, 0.75294117647100001f),
    vec3f(0.86499999999999999, 0.86499999999999999, 0.86499999999999999),
    vec3f(0.70588235294099999, 0.015686274509800001, 0.149019607843)
  };
  const std::vector<float> opacityArray = {0.f, 1.f};
  OSPData colors = ospNewData(colorArray.size(), OSP_FLOAT3, colorArray.data());
  ospCommit(colors);
  OSPData opacities =ospNewData(opacityArray.size(), OSP_FLOAT, opacityArray.data());
  ospCommit(opacities);
  ospSetData(transferFcn, "colors", colors);
  ospSetData(transferFcn, "opacities", opacities);
  ospSet2f(transferFcn, "valueRange", valueRange.x, valueRange.y);
  ospCommit(transferFcn);
  ospRelease(colors);
  ospRelease(opacities);

  OSPMaterial objMaterial = ospNewMaterial("scivis", "OBJMaterial");
  ospSet3f(objMaterial, "Kd", 150 / 255.f, 10 / 255.f, 25 / 255.f);
  ospSet3f(objMaterial, "Ks", 77 / 255.f, 77 / 255.f, 77 / 255.f);
  ospSet1f(objMaterial, "Ns", 10.f);
  ospCommit(objMaterial);

  float isoValue = 20.f;
  // TODO: compute world bounds or read it from p4est
  box3f universeBounds(vec3f(0.f), vec3f(1.f));

  std::shared_ptr<DataSource> pData = NULL;
  std::vector<std::shared_ptr<VoxelOctree>> voxelOctrees;

  pData = std::make_shared<syntheticSource>();
  pData->parseData();

  std::shared_ptr<VoxelOctree> voxelAccel = NULL;

  if (inputOctFile == "") {
    voxelAccel = std::make_shared<VoxelOctree>(
        pData->voxels.data(),
        pData->voxels.size(),
        box3f(pData->gridOrigin, vec3f(pData->dimensions)),
        pData->gridWorldSpace);
  } else {
    // mmap the binary file
    char octreeFileName[10000];
    sprintf(octreeFileName, "%s%06i.oct", inputOctFile.str().c_str(), 0);
    std::string octFile(octreeFileName);
    voxelAccel = std::make_shared<VoxelOctree>();
    voxelAccel->mapOctreeFromFile(octFile);
  }

  assert(voxelAccel);
  voxelOctrees.push_back(voxelAccel);

  OSPGeometry geometry = ospNewGeometry("impi");
  ospSet1f(geometry, "isoValue", isoValue);
  size_t numVoxels = pData->voxels.size();
  ospSetVoidPtr(geometry, "inputVoxels", (void *)pData->voxels.data());
  ospSetVoidPtr(geometry,"numInputVoxels", (void *)&numVoxels);
  ospSetMaterial(geometry, objMaterial);
  ospCommit(geometry);
  ospAddGeometry(world, geometry);
  ospCommit(world);


  // create and setup an ambient light
  std::array<OSPLight, 2> lights = {ospNewLight("ambient"),
                                    ospNewLight("distant")};
  ospCommit(lights[0]);

  ospSet3f(lights[1], "direction", -1.f, -1.f, 0.5f);
  ospSet1f(lights[1], "intensity", 0.003f);
  ospSet1f(lights[1], "angularDiameter", 0.53f);
  ospSet3f(lights[1], "color", 1.f, 131 / 255.f, 131 / 255.f);
  ospCommit(lights[1]);

  OSPData lightData = ospNewData(lights.size(), OSP_LIGHT, lights.data(), 0);
  ospCommit(lightData);


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

  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  ospRelease(renderer);
  // cleanly shut OSPRay down
  ospShutdown();

  return 0;
}
