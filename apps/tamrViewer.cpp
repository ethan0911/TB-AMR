#include <stdint.h>
#include <stdio.h>
#include <cstdlib>
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
#include <stdexcept>
#include <memory>

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
std::array<std::string, 2> colormapNames = { "", "" };
std::vector<std::string> inputMesh;
bool showMesh = false;
bool showVolume = true;
std::vector<vec2f> valueRanges = {vec2f (0.0f, 1.f), vec2f(0.0f, 1.f)};
bool showIso = false;
float isoValue;
std::string rendererName = "scivis";
int aoSamples = 0;
bool exajetInstancing = false;

vec2i windowDims = vec2i(1024, 768);
bool cameraOnCmdline = false;
float fov = 40.f;
vec3f camEye = vec3f(0, 0, -5);
vec3f camDir = vec3f(0, 0, 1);
vec3f camUp = vec3f(0, 1, 0);

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
std::ostream& operator<<(std::ostream &strm, const vec3f &v) {
  return strm << v.x << " " << v.y << " " << v.z;
}
std::ostream& operator<<(std::ostream &strm, const vec4i &v) {
  return strm << v[0] << " " << v[1] << " " << v[2] << " " << v[3];
}

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

void parseCommandLine(int &ac, const char **&av, BenchmarkInfo& benchInfo)
{
  benchInfo.currDataRep = DataRep::octree; // HACK: Set to octree by default
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
    } else if (arg == "-u" || arg == "--unstructured") {
      benchInfo.currDataRep = DataRep::unstructured;
      removeArgs(ac, av, i, 1);
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
      valueRanges[0].x = std::stof(av[i + 1]);
      valueRanges[0].y = std::stof(av[i + 2]);
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
    } else if(arg == "-iso-vr") {
      valueRanges[1].x = std::stof(av[i + 1]);
      valueRanges[1].y = std::stof(av[i + 2]);
      removeArgs(ac, av, i, 3);
      --i;
    } else if (arg == "--use-tf-widget") {
      std::cout << "note: tfwidget is now enabled by default\n";
      removeArgs(ac, av, i, 1);
      --i;
    } else if (arg == "--vol-cmap") {
        colormapNames[0] = av[i + 1];
        removeArgs(ac, av, i, 2);
        --i;
    } else if (arg == "--iso-cmap") {
        colormapNames[1] = av[i + 1];
        removeArgs(ac, av, i, 2);
        --i;
    } else if (arg == "--opacity-scale") {
        std::cout << "--opacity-scale is no longer required/supported\n";
        removeArgs(ac, av, i, 1);
        --i;
    } else if (arg == "--ao") {
        aoSamples = std::atoi(av[i + 1]);
        removeArgs(ac, av, i, 2);
        --i;
    } else if (arg == "--hide-volume") {
        showVolume = false;
        removeArgs(ac, av, i, 1);
        --i;
    } else if (arg == "--renderer") {
        rendererName = av[i + 1];
        removeArgs(ac, av, i, 2);
        --i;
    } else if (arg == "--exa-instance") {
        exajetInstancing = true;
        removeArgs(ac, av, i, 1);
        --i;
    } else if (arg == "--size") {
        windowDims.x = std::atoi(av[i + 1]);
        windowDims.y = std::atoi(av[i + 2]);
        removeArgs(ac, av, i, 3);
        --i;
    } else if (arg == "--fov") {
        cameraOnCmdline = true;
        fov = std::atof(av[i + 1]);
        removeArgs(ac, av, i, 2);
        --i;
    } else if (arg == "--eye") {
        cameraOnCmdline = true;
        camEye.x = std::atof(av[i + 1]);
        camEye.y = std::atof(av[i + 2]);
        camEye.z = std::atof(av[i + 3]);
        removeArgs(ac, av, i, 4);
        --i;
    } else if (arg == "--dir") {
        cameraOnCmdline = true;
        camDir.x = std::atof(av[i + 1]);
        camDir.y = std::atof(av[i + 2]);
        camDir.z = std::atof(av[i + 3]);
        removeArgs(ac, av, i, 4);
        --i;
    } else if (arg == "--up") {
        cameraOnCmdline = true;
        camUp.x = std::atof(av[i + 1]);
        camUp.y = std::atof(av[i + 2]);
        camUp.z = std::atof(av[i + 3]);
        removeArgs(ac, av, i, 4);
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

      // I should come up with a more robust solution than relying on order in
      // sequence. Maybe use key/value pairs
      benchInfo.cellBytes       = std::atoi(tokens[0].c_str());
      benchInfo.camParamPath    = tokens[1];
      benchInfo.numTrials       = std::atoi(tokens[2].c_str());
      benchInfo.numWarmupFrames = std::atoi(tokens[3].c_str());
      benchInfo.subdirName      = tokens[4];
      std::string dataRepName   = tokens[5];

      // FIXME: This will override the "-u" flag, if it is set.
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
      inputMesh.push_back(av[i]);
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

  BenchmarkInfo bInfo;
  parseCommandLine(argc, argv, bInfo);

  OSPWorld world = ospNewWorld();
  OSPGroup group = ospNewGroup();
  // create OSPRay renderer
  OSPRenderer renderer = ospNewRenderer(rendererName.c_str());
  if (!renderer) {
    throw std::runtime_error("invalid renderer name: scivis ");
  }

  // vec2f valueRange(0.0f, 1.f);

  OSPMaterial objMaterial = ospNewMaterial(rendererName.c_str(), "default");
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
      tfnWidgets.emplace_back(valueRanges[i].x, valueRanges[i].y);

      auto &tfcn = tfnWidgets[i];
      if (!colormapNames[i].empty()) {
          tfcn.add_colormap(Colormap(colormapNames[i], true));
      }
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
      ospSetVec2f(fcn, "valueRange", valueRanges[i].x, valueRanges[i].y);
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
    OSPVolume curr_vol = 0;
    if(bInfo.currDataRep == DataRep::unstructured){
      // HACK: Ignoring InputIsosurfaceOctFile for now

      std::string vert_fname = inputOctFile.base() + ".v.unstruct";
      std::unique_ptr<vec3f[]> vertvals;
      size_t vertvals_numbytes = -1;
      // Below, I am using a code snippet from:
      // http://www.cplusplus.com/doc/tutorial/files/
      //
      // Open the file "with the ios::ate flag, which means that the get
      // pointer will be positioned at the end of the file. This way, when we
      // call to member tellg(), we will directly obtain the size of the file."
      ifstream vertvalfile (vert_fname, ios::in|ios::binary|ios::ate);
      if (vertvalfile.is_open()) {
        vertvals_numbytes = vertvalfile.tellg();
        vertvals = std::unique_ptr<vec3f[]>(new vec3f[vertvals_numbytes]);
        vertvalfile.seekg (0, ios::beg);
        vertvalfile.read (reinterpret_cast<char*>(vertvals.get()), vertvals_numbytes);
        std::cout << "Vertex values: read " << vertvals_numbytes << " bytes!" << std::endl;
        vertvalfile.close();
      } else {
        throw new std::runtime_error("Could not open vert value file: " + vert_fname);
      }
      size_t num_vertvals = vertvals_numbytes / sizeof(vec3f); // HACK

      // Read vertex array
      /*
       *std::vector<vec3f> verts;
       *std::string curr_line;
       *std::ifstream vtxfile(inputOctFile.base() + ".v.unstruct");
       *if (vtxfile.is_open()) {
       *  while (getline(vtxfile, curr_line)) {
       *    std::vector<std::string> str_tokens;
       *    split_string<std::vector<std::string>>(curr_line, str_tokens);
       *    if (str_tokens.size() != 3) {
       *      std::cout << "# tokens: " << str_tokens.size() << std::endl;
       *      throw std::runtime_error("Invalid .unstruct file format!");
       *    }
       *    vec3f curr_vert = vec3f(atof(str_tokens[0].c_str()),
       *                            atof(str_tokens[1].c_str()),
       *                            atof(str_tokens[2].c_str()));
       *    verts.push_back(curr_vert);
       *  }
       *  vtxfile.close();
       *}
       */
      std::string idx_fname = inputOctFile.base() + ".i.unstruct";
      std::unique_ptr<vec4i[]> idxvals;
      size_t idxvals_numbytes = -1;
      ifstream indexfile (idx_fname, ios::in|ios::binary|ios::ate);
      if (indexfile.is_open()) {
        idxvals_numbytes = indexfile.tellg();
        idxvals = std::unique_ptr<vec4i[]>(new vec4i[idxvals_numbytes]);
        indexfile.seekg (0, ios::beg);
        indexfile.read (reinterpret_cast<char*>(idxvals.get()), idxvals_numbytes);
        std::cout << "Index values: read " << idxvals_numbytes << " bytes!" << std::endl;
        indexfile.close();
      } else {
        throw new std::runtime_error("Could not open index value file: " + idx_fname);
      }
      size_t num_idxvals = idxvals_numbytes / sizeof(vec4i); // HACK

      // Read index array
      /*
       *std::vector<vec4i> idxs;
       *std::string curr_line = "";
       *std::ifstream idxfile(inputOctFile.base() + ".i.unstruct");
       *if (idxfile.is_open()) {
       *  while (getline(idxfile, curr_line)) {
       *    std::vector<std::string> str_tokens;
       *    split_string<std::vector<std::string>>(curr_line, str_tokens);
       *    if (str_tokens.size() != 4) {
       *      std::cout << "# tokens: " << str_tokens.size() << std::endl;
       *      throw std::runtime_error("Invalid .unstruct file format!");
       *    }
       *    vec4i curr_idxs = vec4i(atoi(str_tokens[0].c_str()),
       *                            atoi(str_tokens[1].c_str()),
       *                            atoi(str_tokens[2].c_str()),
       *                            atoi(str_tokens[3].c_str()));
       *    idxs.push_back(curr_idxs);
       *  }
       *  idxfile.close();
       *}
       */

      std::string field_fname = inputOctFile.base() + ".f.unstruct";
      std::unique_ptr<float[]> fieldvals;
      size_t fieldvals_numbytes = -1;
      ifstream fieldvalfile (field_fname, ios::in|ios::binary|ios::ate);
      if (fieldvalfile.is_open()) {
        fieldvals_numbytes = fieldvalfile.tellg();
        fieldvals = std::unique_ptr<float[]>(new float[fieldvals_numbytes]);
        fieldvalfile.seekg (0, ios::beg);
        fieldvalfile.read (reinterpret_cast<char*>(fieldvals.get()), fieldvals_numbytes);
        std::cout << "Field values: read " << fieldvals_numbytes << " bytes!" << std::endl;
        fieldvalfile.close();
      } else {
        throw new std::runtime_error("Could not open field value file: " + field_fname);
      }
      size_t num_fieldvals = fieldvals_numbytes / sizeof(float); // HACK

      // Create OSPData arrays for verts, indices, field values
      //OSPData vtxData = ospNewData(verts.size(), OSP_VEC3F, verts.data());
      OSPData vtxData = ospNewData(num_vertvals, OSP_VEC3F, vertvals.get());
      //OSPData idxData = ospNewData(idxs.size(), OSP_VEC4I, idxs.data());
      OSPData idxData = ospNewData(num_idxvals, OSP_VEC4I, idxvals.get());
      OSPData cellFieldData = ospNewData(num_fieldvals, OSP_FLOAT, fieldvals.get());

      // Create cell type array, and create a matching OSPData object
      std::vector<char> cell_types;
      cell_types.resize(num_fieldvals); // size is the number of cells
      for (int i = 0; i < cell_types.size(); ++i) {
        cell_types[i] = OSP_HEXAHEDRON;
      }
      OSPData typeData = ospNewData(cell_types.size(), OSP_UCHAR, cell_types.data());

      // Call OSPCommit on the above OSPData arrays
      ospCommit(vtxData);
      ospCommit(idxData);
      ospCommit(cellFieldData);
      ospCommit(typeData);

      // Create a new OSPVolume
      curr_vol = ospNewVolume("unstructured_volume");

      // Don't set the transfer function here... in the new API, the transfer
      // function appears to be bound to an OSPVolumetricModel.

      // Bind data arrays to the OSPVolume
      ospSetData(curr_vol, "vertex.position", vtxData);
      ospSetData(curr_vol, "cell.value", cellFieldData);
      ospSetData(curr_vol, "index", idxData);
      ospSetData(curr_vol, "cell.type", typeData);
      // The following optimization assumes hexes have planar sides.
      // In the case of our TAMR data, this should always be true.
      ospSetString(curr_vol, "hexMethod", "planar");
      //TODO: Test with and without precomputed normals.
    } else {
      curr_vol = ospNewVolume("tamr");
      auto src = dataSources[i];

      // pass exajet data and metadata, only for one tree right now
      ospSetVec3f(curr_vol,
                  "worldOrigin",
                  src->worldOrigin.x,
                  src->worldOrigin.y,
                  src->worldOrigin.z);
      ospSetVec3f(curr_vol,
                  "gridOrigin",
                  src->gridOrigin.x,
                  src->gridOrigin.y,
                  src->gridOrigin.z);
      ospSetVec3f(curr_vol,
                  "gridWorldSpace",
                  src->gridWorldSpace.x,
                  src->gridWorldSpace.y,
                  src->gridWorldSpace.z);
      ospSetVec3i(curr_vol,
                  "dimensions",
                  src->dimensions.x,
                  src->dimensions.y,
                  src->dimensions.z);

      ospSetVoidPtr(curr_vol, "voxelOctree", (void *)voxelOctrees[i].get());
      ospSetInt(curr_vol, "gradientShadingEnabled", 0);
    }
    if (curr_vol == 0)
      throw std::runtime_error("Null pointer to OSPVolume!");

    ospCommit(curr_vol);

    OSPVolumetricModel volumeModel = ospNewVolumetricModel(curr_vol);
    ospSetObject(volumeModel, "transferFunction", transferFcns[i]);
    if (intputDataType == "synthetic") {
        ospSetFloat(volumeModel, "samplingRate", 1.f);
    } else if (intputDataType == "p4est") {
        ospSetFloat(volumeModel, "samplingRate", 1.f);
    } else if (intputDataType == "exajet") {
        ospSetFloat(volumeModel, "samplingRate", 0.125f);
    }

    ospCommit(volumeModel);
    volumes.push_back(curr_vol);
    volumetricModels.push_back(volumeModel);
  }

  // TODO: This will now take from some separate field we load up
  OSPTexture isoColormap = ospNewTexture("volume");
  ospSetObject(isoColormap, "volume", volumetricModels.back());
  ospCommit(isoColormap);

  if (showVolume) {
      OSPData volumeList = ospNewData(1, OSP_VOLUMETRIC_MODEL, &volumetricModels[0]);
      ospCommit(volumeList);

      ospSetObject(group, "volume", volumeList);
  }

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

    OSPMaterial dataMat = ospNewMaterial(rendererName.c_str(), "default");
    ospSetVec3f(dataMat, "Kd", 1.f, 1.f, 1.f);
    ospSetObject(dataMat, "map_Kd", isoColormap);
    ospCommit(dataMat);

    OSPGeometricModel geomModel = ospNewGeometricModel(geometry);
    ospSetObject(geomModel, "material", dataMat);
    ospCommit(geomModel);
    ospRelease(dataMat);

    geometries.push_back(geomModel);

  }

  if (!geometries.empty()) {
    OSPData geomList = ospNewData(geometries.size(), OSP_GEOMETRIC_MODEL, geometries.data());
    ospCommit(geomList);
    ospSetObject(group, "geometry", geomList);
  }

  ospCommit(group);

  std::vector<OSPInstance> instances = {
      ospNewInstance(group)
  };
  ospCommit(instances[0]);

  if (exajetInstancing) {
      instances.push_back(ospNewInstance(group));
      affine3f flip_matrix(one);
      flip_matrix.l.vy.y = -1;
      ospSetAffine3fv(instances[1], "xfm", reinterpret_cast<float*>(&flip_matrix));
      ospCommit(instances[1]);
  }

  OSPData instancesData = ospNewData(instances.size(), OSP_INSTANCE, instances.data());
  ospCommit(instancesData);

  ospSetObject(world, "instance", instancesData);
  ospCommit(world);

  vec3f lightDir(1.f);
  vec3f directionalColor(55.f/255.f, 100.f/255.f, 145.f/255.f);
  float directionalIntensity = 2.5f;
  vec3f ambientcolor(1.f);
  float ambientIntensity = 0.5f;
  // create and setup an ambient light
  std::array<OSPLight, 2> lights = {ospNewLight("ambient"),
      ospNewLight("distant")};

  ospSetVec3fv(lights[0], "color", &ambientcolor.x);
  ospSetFloat(lights[0], "intensity", ambientIntensity);
  ospCommit(lights[0]);

  ospSetVec3fv(lights[1], "direction", &lightDir.x);
  ospSetFloat(lights[1], "intensity", directionalIntensity);
  ospSetFloat(lights[1], "angularDiameter", 0.53f);
  ospSetVec3fv(lights[1], "color", &directionalColor.x);
  ospCommit(lights[1]);

  if (rendererName != "scivis") {
      OSPData lightData = ospNewData(lights.size(), OSP_LIGHT, lights.data());
      ospCommit(lightData);
      ospSetObject(renderer, "light", lightData);
  }

  ospSetVec3f(renderer, "bgColor", 1.0, 1.0, 1.0);
  ospSetInt(renderer, "aoSamples", aoSamples);
  ospCommit(renderer);

  // create a GLFW OSPRay window: this object will create and manage the OSPRay
  // frame buffer and camera directly
  auto glfwOSPRayWindow = std::unique_ptr<GLFWOSPRayWindow>(
      new GLFWOSPRayWindow(windowDims, universeBounds, world, renderer));

  if (cameraOnCmdline) {
      glfwOSPRayWindow->setCamera(camEye, camDir, camUp, fov);
  }

  glfwOSPRayWindow->registerImGuiCallback([&]() {
    static int spp = 1;
    static int samplesPerCell = 1;
    if (ImGui::SliderInt("spp", &spp, 1, 64)) {
      ospSetInt(renderer, "spp", spp);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
    if (ImGui::SliderInt("ao samples", &aoSamples, 0, 4)) {
      ospSetInt(renderer, "aoSamples", aoSamples);
      glfwOSPRayWindow->addObjectToCommit(renderer);
    }
    if (ImGui::SliderInt("samples per cell", &samplesPerCell, 1, 16)) {
      ospSetInt(volumes[0], "samplesPerCell", samplesPerCell);
      glfwOSPRayWindow->addObjectToCommit(volumes[0]);
    }

    if (rendererName != "scivis") {
        ImGui::PushID(1);
        ImGui::Text("ambient light");
        if (ImGui::SliderFloat("intensity", &ambientIntensity, 0.f, 10.f)) {
            ospSetFloat(lights[0], "intensity", ambientIntensity);
            glfwOSPRayWindow->addObjectToCommit(lights[0]);
            glfwOSPRayWindow->addObjectToCommit(renderer);
        }
        if (ImGui::ColorEdit3("color", &ambientcolor.x)) {
            ospSetVec3fv(lights[0], "color", &ambientcolor.x);
            glfwOSPRayWindow->addObjectToCommit(lights[0]);
            glfwOSPRayWindow->addObjectToCommit(renderer);
        }
        ImGui::PopID();

        ImGui::PushID(2);
        ImGui::Text("directional light");
        if (ImGui::SliderFloat3("direction", &lightDir.x, -1.f, 1.f)) {
            ospSetVec3fv(lights[1], "direction", &lightDir.x);
            glfwOSPRayWindow->addObjectToCommit(lights[1]);
            glfwOSPRayWindow->addObjectToCommit(renderer);
        }
        if (ImGui::SliderFloat("intensity", &directionalIntensity, 0.f, 10.f)) {
            ospSetFloat(lights[1], "intensity", directionalIntensity);
            glfwOSPRayWindow->addObjectToCommit(lights[1]);
            glfwOSPRayWindow->addObjectToCommit(renderer);
        }
        if (ImGui::ColorEdit3("color", &directionalColor.x)) {
            ospSetVec3fv(lights[1], "color", &directionalColor.x);
            glfwOSPRayWindow->addObjectToCommit(lights[1]);
            glfwOSPRayWindow->addObjectToCommit(renderer);
        }
        ImGui::PopID();
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
