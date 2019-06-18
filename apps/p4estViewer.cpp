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

void parseCommandLind(int &ac, const char **&av)
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

  std::vector<std::shared_ptr<VoxelOctree>> voxelOctrees;

  std::shared_ptr<DataSource> pData = NULL;

  // mmap the binary file
  char octreeFileName[10000];

  if (intputDataType == "p4est") {
    pData = std::make_shared<p4estSource>();
    pData->mapMetaData(inputFile.str());
    universeBounds = box3f(vec3f(-0.3f), vec3f(1.3f));
    valueRange     = vec2f(0.f, 1.f);
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

  std::shared_ptr<VoxelOctree> voxelAccel = std::make_shared<VoxelOctree>();

  time_point t1 = Time();
  sprintf(octreeFileName, "%s%06i.oct", inputFile.str().c_str(), 0);
  std::string octFile(octreeFileName);
  voxelAccel->mapOctreeFromFile(octFile);
  double loadTime = Time(t1);
  std::cout << yellow << "Loading " <<voxelAccel->_octreeNodes.size() << " octree nodes from file takes " << loadTime << " s" << reset << "\n";
  voxelOctrees.push_back(voxelAccel);
  
                            
  OSPVolume tree = ospNewVolume("p4est");
  ospSet1f(tree, "samplingRate", 1.f);
  // pass exajet data and metadata, only for one tree right now
  ospSet3f(tree, "worldOrigin", pData->worldOrigin.x, pData->worldOrigin.y, pData->worldOrigin.z);
  ospSet3f(tree,"gridOrigin", pData->gridOrigin.x, pData->gridOrigin.y, pData->gridOrigin.z);
  ospSet3f(tree, "gridWorldSpace", pData->gridWorldSpace.x, pData->gridWorldSpace.y, pData->gridWorldSpace.z);
  ospSet3i(tree, "dimensions", pData->dimensions.x, pData->dimensions.y, pData->dimensions.z);

  ospSetVoidPtr(tree, "voxelOctree", (void *)voxelOctrees[voxelOctrees.size() - 1].get());
  ospSet1i(tree, "gradientShadingEnabled", 0);

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

  return 0;
}