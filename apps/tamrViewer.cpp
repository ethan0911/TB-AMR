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
#include "widgets/transfer_function_widget.h"

#include "Utils.h"

using namespace ospcommon;
using namespace ospcommon::math;

std::string intputDataType;
FileName inputOctFile;
FileName inputIsosurfaceOctFile;
std::string inputField = "default";
std::string isosurfaceField = "default";
std::vector<std::string> inputMesh;
bool showMesh = false;
vec2f valueRange(0.0f, 1.f);
bool showIso = false;
float isoValue;

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

void parseCommandLine(int &ac, const char **&av, BenchmarkInfo& benchInfo)
{
  for (int i = 1; i < ac; ++i) {
    const std::string arg = av[i];
    if (arg == "-t" || arg == "--type") {
      intputDataType = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-d" || arg == "--data") {
      inputOctFile = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    }else if (arg == "-i" || arg == "--input-oct") {
      inputOctFile = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-f" || arg == "--field") {
      inputField = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    } else if(arg == "-vr" || arg == "--valueRange"){
      valueRange.x = std::stof(av[i + 1]);
      valueRange.y = std::stof(av[i + 2]);
      removeArgs(ac, av, i, 3);
      --i;
    } else if (arg == "-iso") {
      showIso    = true;
      isoValue = std::stof(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-iso-oct") {
      inputIsosurfaceOctFile = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-iso-field") {
      isosurfaceField = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "--use-tf-widget") {
      std::cout << "note: tfwidget is now enabled by default\n";
      removeArgs(ac, av, i, 1);
      --i;
    } else if (arg == "-b" || arg == "--benchmark") {
      benchInfo.benchmarkMode = true;

      std::string bench_config_str = av[i + 1];
      // std::cout << "Benchmark config string: " << bench_config_str <<
      // std::endl;

      // Code snippet from:
      // https://www.geeksforgeeks.org/tokenizing-a-string-cpp/ Vector of string
      // to save tokens
      vector<string> tokens;

      // stringstream class check1
      stringstream bench_config_sstream(bench_config_str);

      // Tokenizing w.r.t. space ' '
      string intermediate;
      while (getline(bench_config_sstream, intermediate, ' ')) {
        tokens.push_back(intermediate);
      }

      std::cout << "Benchmark config string tokens: ";
      for (std::string t : tokens) {
        std::cout << t << " " << std::endl;
      }
      std::cout << std::endl;

      // TODO: Come up with a more robust solution than relying on order in
      // sequence. Maybe use key/value pairs
      benchInfo.cellBytes       = std::atoi(tokens[0].c_str());
      benchInfo.camParamPath    = tokens[1];
      benchInfo.numTrials       = std::atoi(tokens[2].c_str());
      benchInfo.numWarmupFrames = std::atoi(tokens[3].c_str());
      benchInfo.subdirName      = tokens[4];
      std::string dataRepName   = tokens[5];

      if (dataRepName.compare("unstructured") == 0) {
        benchInfo.currDataRep = DataRep::unstructured;
      } else if (dataRepName.compare("octree") == 0) {
        benchInfo.currDataRep = DataRep::octree;
      } else {
        throw std::domain_error("Invalid data representation!");
      }

      std::cout << benchInfo;

      removeArgs(ac, av, i, 2);
      --i;
    } else {
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
  if (ospLoadModule("tamr") != OSP_NO_ERROR) {
    throw std::runtime_error("failed to initialize TAMR module");
  }
  if (ospLoadModule("impi") != OSP_NO_ERROR) {
    throw std::runtime_error("failed to initialize IMPI module");
  }

  OSPWorld world = ospNewWorld();
  OSPGroup group = ospNewGroup();
  // create OSPRay renderer
  OSPRenderer renderer = ospNewRenderer("scivis");

  if (!renderer) {
    throw std::runtime_error("invalid renderer name: scivis ");
  }

  BenchmarkInfo bInfo;
  parseCommandLine(argc, argv, bInfo);


  // vec2f valueRange(0.0f, 1.f);

  OSPMaterial objMaterial = ospNewMaterial("scivis", "default");
  ospSetVec3f(objMaterial, "Kd", 3 / 255.f, 10 / 255.f, 25 / 255.f);
  ospCommit(objMaterial);

  // TODO: compute world bounds or read it from p4est
  box3f universeBounds(vec3f(0.f), vec3f(1.f));

  std::vector<std::shared_ptr<DataSource>> dataSources;
  // The other volume to colormap the isosurface by, if desired
  std::shared_ptr<DataSource> isosurfaceData = NULL;
  std::vector<std::shared_ptr<VoxelOctree>> voxelOctrees;

  if (intputDataType == "p4est") {
    auto pData = std::make_shared<p4estSource>();
    pData->mapMetaData(inputOctFile.str());
    universeBounds = box3f(vec3f(-0.3f), vec3f(1.3f));
    dataSources.push_back(pData);

    if (!inputIsosurfaceOctFile.str().empty()) {
        pData = std::make_shared<p4estSource>();
        pData->mapMetaData(inputIsosurfaceOctFile.str());
        dataSources.push_back(pData);
    }
    // valueRange     = vec2f(0.f, 1.0f); //HACK! Currently hardcoded for Mandelbrot set.
  }

  if (intputDataType == "synthetic") {
    auto pData = std::make_shared<syntheticSource>();
    pData->mapMetaData(inputOctFile.str());
    dataSources.push_back(pData);
    universeBounds = box3f(vec3f(0.f), vec3f(4.f));
    // valueRange     = vec2f(1.f, 9.f);
    // valueRange     = vec2f(4.f, 8.f);
    // valueRange     = vec2f(0.f, 64.f);
    if (!inputIsosurfaceOctFile.str().empty()) {
        pData = std::make_shared<syntheticSource>();
        pData->mapMetaData(inputIsosurfaceOctFile.str());
        dataSources.push_back(pData);
    }
  }

  // NASA exajet data
  if (intputDataType == "exajet" || intputDataType == "landing") {
    auto pData = std::make_shared<exajetSource>(inputOctFile, inputField);
    pData->mapMetaData(inputOctFile.str());
    universeBounds = box3f(pData->gridOrigin, pData->gridWorldSpace * pData->dimensions) + pData->worldOrigin;
    dataSources.push_back(pData);

    // valueRange = vec2f(-10.0f, 20.0f);  // y_vorticity.bin
    // valueRange = vec2f(1.2f, 1.205f);  // density.bin [0.59,1.95]

    if (!inputIsosurfaceOctFile.str().empty()) {
        pData = std::make_shared<exajetSource>(inputIsosurfaceOctFile, isosurfaceField);
        pData->mapMetaData(inputIsosurfaceOctFile.str());
        dataSources.push_back(pData);
    }
  }

    // NASA landinggear data
  // if (intputDataType == "landing") {
  //   pData         = std::make_shared<landingSource>(inputData, inputField.str());

  //   pData->mapMetaData(inputOctFile.str());
  //   universeBounds = box3f(pData->gridOrigin, pData->gridWorldSpace * pData->dimensions) + pData->worldOrigin;
  // }

  Mesh mesh;
  affine3f transform =
      affine3f::translate(vec3f(0.f)) * affine3f::scale(vec3f(1.f));

  std::vector<OSPGeometricModel> geometries;
  time_point t1;
  if (showMesh) {
    t1                      = Time();
    mesh.LoadMesh(inputMesh);
    mesh.SetTransform(transform);
    mesh.AddToModel(geometries, objMaterial);
    double loadMeshTime = Time(t1);
    std::cout << green << "Loading NASA Exajet Airplane Mesh Takes " << loadMeshTime << " s" << reset
              << "\n";
  }

  //! Set up us the transfer function*******************************************
  std::vector<TransferFunctionWidget> tfnWidgets;
  std::vector<OSPTransferFunction> transferFcns;
  for (size_t i = 0; i < dataSources.size(); ++i) {
      tfnWidgets.emplace_back(valueRange.x, valueRange.y);

      auto &tfcn = tfnWidgets[i];
      std::vector<float> colorArray;
      std::vector<float> opacityArray;
      tfcn.get_colormapf(colorArray, opacityArray);

      OSPData colors = ospNewData(colorArray.size() / 3, OSP_VEC3F, colorArray.data());
      ospCommit(colors);

      OSPData opacities =
          ospNewData(opacityArray.size(), OSP_FLOAT, opacityArray.data());
      ospCommit(opacities);

      OSPTransferFunction fcn = ospNewTransferFunction("piecewise_linear");
      ospSetData(fcn, "color", colors);
      ospSetData(fcn, "opacity", opacities);
      ospSetVec2f(fcn, "valueRange", valueRange.x, valueRange.y);
      ospCommit(fcn);
      ospRelease(colors);
      ospRelease(opacities);
      transferFcns.push_back(fcn);
  }

  for (size_t i = 0; i < dataSources.size(); ++i) {
      auto src = dataSources[i];
      bool bGeneOctree = false;
      std::shared_ptr<VoxelOctree> voxelAccel = NULL;

      if(bGeneOctree){
          // TODO: run this function here will cause bugs for p4est data,
          // TODO: since the p4est handle  is not initialized
          src->parseData();
          voxelAccel = std::make_shared<VoxelOctree>(
                  src->voxels.data(),
                  src->voxels.size(),
                  src->voxelRange,
                  box3f(src->gridOrigin, vec3f(src->dimensions)),
                  src->gridWorldSpace,
                  src->worldOrigin);
      }else{
          voxelAccel = std::make_shared<VoxelOctree>();
          t1         = Time();
          char octreeFileName[10000] = {0};
          sprintf(octreeFileName, "%s-%s%06i.oct",
                  i == 0 ? inputOctFile.c_str() : inputIsosurfaceOctFile.c_str(),
                  i == 0 ? inputField.c_str() : isosurfaceField.c_str(),
                  0);
          std::string octFile(octreeFileName);
          voxelAccel->mapOctreeFromFile(octFile);
          double loadTime = Time(t1);
          std::cout << yellow << "Loading " << voxelAccel->_octreeNodes.size()
              << " octree nodes from '" << octFile << "' takes " << loadTime << " s" << reset
              << "\n";
      }
      voxelOctrees.push_back(voxelAccel);
  }

  std::vector<OSPVolume> volumes;
  std::vector<OSPVolumetricModel> volumetricModels;

  for (size_t i = 0; i < dataSources.size(); ++i) {
      OSPVolume tree = ospNewVolume("tamr");
      auto src = dataSources[i];

      // pass exajet data and metadata, only for one tree right now
      ospSetVec3f(tree, "worldOrigin", src->worldOrigin.x, src->worldOrigin.y, src->worldOrigin.z);
      ospSetVec3f(tree,"gridOrigin", src->gridOrigin.x, src->gridOrigin.y, src->gridOrigin.z);
      ospSetVec3f(tree, "gridWorldSpace", src->gridWorldSpace.x, src->gridWorldSpace.y, src->gridWorldSpace.z);
      ospSetVec3i(tree, "dimensions", src->dimensions.x, src->dimensions.y, src->dimensions.z);

      ospSetVoidPtr(tree, "voxelOctree", (void *)voxelOctrees[i].get());
      ospSetInt(tree, "gradientShadingEnabled", 0);

      ospCommit(tree);

      OSPVolumetricModel volumeModel = ospNewVolumetricModel(tree);
      ospSetObject(volumeModel, "transferFunction", transferFcns[i]);
      if (intputDataType == "synthetic") {
          ospSetFloat(volumeModel, "samplingRate", 1.f);
      } else if (intputDataType == "p4est") {
          ospSetFloat(volumeModel, "samplingRate", 1.f);
      } else if (intputDataType == "exajet") {
          ospSetFloat(volumeModel, "samplingRate", 0.125f);
      }

      ospCommit(volumeModel);
      volumes.push_back(tree);
      volumetricModels.push_back(volumeModel);
  }

  // TODO: This will now take from some separate field we load up
  OSPTexture isoColormap = ospNewTexture("volume");
  ospSetObject(isoColormap, "volume", volumetricModels.back());
  ospCommit(isoColormap);

  OSPData volumeList = ospNewData(1, OSP_VOLUMETRIC_MODEL, &volumetricModels[0]);
  ospCommit(volumeList);

  ospSetObject(group, "volume", volumeList);

  if (showIso) {
    auto pData = dataSources[0];
    t1 = Time();
    char voxelFileName[10000] = {0};
    sprintf(voxelFileName,
            "%s-%s",
            inputOctFile.str().c_str(),
            inputField.c_str());
    std::string vFile(voxelFileName);
    pData->mapVoxelsArrayData(vFile);
    double loadPointTime = Time(t1);
    std::cout << yellow << "Loading input cell data takes: " << loadPointTime
              << " s" << reset << "\n";

    // float isoValue       = 1.201f;  // exajet density
    // float isoValue       = 6.5;   //synthetic data
    // float isoValue       = 0.5f;  //p4est
    // float isoValue       = 200.f;  //exajet y_vorticity

    OSPGeometry geometry = ospNewGeometry("impi");
    ospSetFloat(geometry, "isoValue", isoValue);
    size_t numVoxels = pData->voxels.size();
    ospSetVoidPtr(geometry, "TAMRVolume", (void *)volumes[0]);
    ospSetVoidPtr(geometry, "inputVoxels", (void *)pData->voxels.data());
    ospSetVoidPtr(geometry, "numInputVoxels", (void *)&numVoxels);
    ospSetInt(volumes[0], "gradientShadingEnabled", 1);
    ospCommit(volumes[0]);
    ospCommit(geometry);

    OSPMaterial dataMat = ospNewMaterial("scivis", "default");
    ospSetVec3f(dataMat, "Kd", 1.f, 1.f, 1.f); 
    ospSetObject(dataMat, "map_Kd", isoColormap);
    ospCommit(dataMat);

    OSPGeometricModel geomModel = ospNewGeometricModel(geometry);
    ospSetObject(geomModel, "material", dataMat);
    ospCommit(geomModel);
    ospRelease(dataMat);

    geometries.push_back(geomModel);

    OSPData geomList = ospNewData(geometries.size(), OSP_GEOMETRIC_MODEL, geometries.data());
    ospCommit(geomList);
    ospSetObject(group, "geometry", geomList);
  }

  ospCommit(group);
  OSPInstance instance = ospNewInstance(group);
  ospCommit(instance);

  OSPData instances = ospNewData(1, OSP_INSTANCE, &instance);
  ospCommit(instances);

  ospSetObject(world, "instance", instances);
  ospCommit(world);

  // create and setup an ambient light
  std::array<OSPLight, 2> lights = {ospNewLight("ambient"),
                                    ospNewLight("distant")};

  ospSetVec3f(lights[0], "color", 134.f/255.f,134.f/255.f,134.f/255.f);
  ospSetFloat(lights[0], "intensity", 0.5f);
  ospCommit(lights[0]);

  ospSetVec3f(lights[1], "direction",-1.f, 1.f, -1.f);
  ospSetFloat(lights[1], "intensity", 2.5f);
  ospSetFloat(lights[1], "angularDiameter", 0.53f);
  ospSetVec3f(lights[1], "color", 55.f/255.f,100.f/255.f,145.f/255.f);
  ospCommit(lights[1]);

  OSPData lightData = ospNewData(lights.size(), OSP_LIGHT, lights.data(), 0);
  ospCommit(lightData);


  ospSetObject(renderer, "lights", lightData);
  ospSetVec3f(renderer, "bgColor", 1.0, 1.0, 1.0);
  ospCommit(renderer);
  ospRelease(lightData);

  // create a GLFW OSPRay window: this object will create and manage the OSPRay
  // frame buffer and camera directly
  auto glfwOSPRayWindow = std::unique_ptr<GLFWOSPRayWindow>(
      new GLFWOSPRayWindow(vec2i{1024, 768}, universeBounds, world, renderer));

  vec3f eyePos(32.185230, 31.767683, 0.592874);
  vec3f lookDir(0.642963, 0.754452, -0.131926);
  vec3f upDir(0.015485,-0.185020,-0.982615);

  // glfwOSPRayWindow->setCamera(eyePos, lookDir, upDir);

  glfwOSPRayWindow->registerImGuiCallback([&]() {
    static int spp = 1;
    static int samplesPerCell = 1;
    if (ImGui::SliderInt("spp", &spp, 1, 64)) {
      ospSetInt(renderer, "spp", spp);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
    if (ImGui::SliderInt("samples per cell", &samplesPerCell, 1, 16)) {
      ospSetInt(volumes[0], "samplesPerCell", samplesPerCell);
      glfwOSPRayWindow->addObjectToCommit(volumes[0]);
    }

    static ImVec4 ambColor = ImColor(134.f/255.f, 134.f/255.f, 134./255.f, 1.f);
    if (ImGui::ColorEdit4("color_ambient",(float *)&ambColor,
            ImGuiColorEditFlags_NoAlpha | ImGuiColorEditFlags_NoInputs |
            ImGuiColorEditFlags_NoLabel | ImGuiColorEditFlags_AlphaPreview |
            ImGuiColorEditFlags_NoOptions | ImGuiColorEditFlags_NoTooltip)) {
      ospSetVec3f(lights[0], "color", ambColor.x, ambColor.y, ambColor.z);
      ospCommit(lights[0]);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
    ImGui::SameLine();
    ImGui::Text("%s - %s", "ambient", "light");
    static float ambIntensity(0.5f);
    if (ImGui::SliderFloat("intensity", &ambIntensity, 0.f, 10.f, "%.3f", 5.0f)) {
      ospSetFloat(lights[0], "intensity", ambIntensity);
      ospCommit(lights[0]);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }

    static ImVec4 dirLightColor1 = ImColor(1.f, 1.f, 1.f, 1.f);
    if (ImGui::ColorEdit4("color_dirlight1",(float *)&dirLightColor1,
            ImGuiColorEditFlags_NoAlpha | ImGuiColorEditFlags_NoInputs |
            ImGuiColorEditFlags_NoLabel | ImGuiColorEditFlags_AlphaPreview |
            ImGuiColorEditFlags_NoOptions | ImGuiColorEditFlags_NoTooltip)) {
      ospSetVec3f(lights[1], "color", dirLightColor1.x, dirLightColor1.y, dirLightColor1.z);
      ospCommit(lights[1]);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
    ImGui::SameLine();
    ImGui::Text("%s - %s", "direction", "1");

    static vec3f dL1_dir = vec3f(1.f, 1.f, 1.f);
    if (ImGui::SliderFloat3("direction", &dL1_dir.x, -1.f, 1.f)) {
      ospSetVec3f(lights[1], "direction", dL1_dir.x, dL1_dir.y, dL1_dir.z);
      ospCommit(lights[1]);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }

    for (int i = 0; i < tfnWidgets.size(); ++i) {
        auto &tfnWidget = tfnWidgets[i];

        const std::string panelName = "Transfer Function " + std::to_string(i);
        if (ImGui::Begin(panelName.c_str())) {
            tfnWidget.draw_ui();
        }
        ImGui::End();

        if (tfnWidget.changed()) {
            std::vector<float> colorArray;
            std::vector<float> opacityArray;
            tfnWidget.get_colormapf(colorArray, opacityArray);

            OSPData colors = ospNewData(colorArray.size() / 3, OSP_VEC3F, colorArray.data());
            ospCommit(colors);

            OSPData opacities =
                ospNewData(opacityArray.size(), OSP_FLOAT, opacityArray.data());
            ospCommit(opacities);

            ospSetData(transferFcns[i], "color", colors);
            ospSetData(transferFcns[i], "opacity", opacities);

            auto range = tfnWidgets[i].get_current_range();
            ospSetVec2f(transferFcns[i], "valueRange", range[0], range[1]);
            glfwOSPRayWindow->addObjectToCommit(transferFcns[i]);
            ospRelease(colors);
            ospRelease(opacities);
        }
      }
  });

  // start the GLFW main loop, which will continuously render
  glfwOSPRayWindow->mainLoop();

  ospRelease(renderer);
  // cleanly shut OSPRay down
  ospShutdown();

  return 0;
}
