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

#include "ospray/common/OSPCommon.h"
#include "ospray/ospray.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "../ospray/VoxelOctree.h"
#include "dataImporter.h"
#include "loader/meshloader.h"

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

std::string inputDataType;
FileName inputData;
FileName inputField("default");
std::string outputFile;
bool unstructured = false;

void parseCommandLine(int &ac, const char **&av)
{
  for (int i = 1; i < ac; ++i) {
    const std::string arg = av[i];
    if (arg == "-t" || arg == "--type") {
      inputDataType = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-d" || arg == "--data") {
      inputData = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    } else if (arg == "-f" || arg == "--field") {
      inputField = FileName(av[i + 1]);
      removeArgs(ac, av, i, 2);
      --i;
    }else if (arg == "-o" || arg == "--output"){
      outputFile = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    }else if (arg == "-u" || arg == "--unstructured"){
      unstructured = true;
      removeArgs(ac, av, i, 2);
      --i;
    } else {
      throw "Invalid argument!";
    }
  }

  PRINT(inputDataType);

  if (inputDataType == "")
    throw runtime_error("Input data type must be set!!");

  if (inputData == "" && inputDataType != "synthetic")
    throw runtime_error("Input file must be set!!");

  if (inputDataType == "exajet" && inputField == "")
    throw runtime_error("Data field must be set for the exajet data!");

  if (outputFile == "")
    throw runtime_error("Output data type must be set!!");
}


//only support for one tree currently, need to extend to multiple tree
int main(int argc, const char **argv)
{

  parseCommandLine(argc, argv);

  std::vector<std::shared_ptr<VoxelOctree>> voxelOctrees;
  std::shared_ptr<DataSource> pData = NULL;

  if (inputDataType == "synthetic") {
    pData = std::make_shared<syntheticSource>();
  }

  // NASA exajet data

  if (inputDataType == "exajet") {
    const vec3i gridMin     = vec3i(1232128, 1259072, 1238336);
    const float voxelScale  = 0.0005;
    const vec3f worldOrigin = vec3f(-1.73575, -9.44, -3.73281);
    pData = std::make_shared<exajetSource>(inputData, inputField.str(),gridMin,voxelScale,worldOrigin);
  }

  if (inputDataType == "landing") {
#if 0
    //configuration for LNAL Metero data
    const vec3i gridMin     = vec3i(-3680, -800, -1920);
    const float voxelScale  = 0.0005f;
    const vec3f worldOrigin = vec3f(0.f);
#endif

#if 1
    // configuration for NASA LandingGear
    const vec3i gridMin     = vec3i(0);
    const float voxelScale  = 2.44e-04;
    const vec3f worldOrigin = vec3f(15.995, 16, 0.1);
#endif

    pData = std::make_shared<exajetSource>(inputData, inputField.str(),gridMin,voxelScale,worldOrigin);
  }

  if (inputDataType == "synthetic" || inputDataType == "exajet" ||
      inputDataType == "landing") {
    time_point t1 = Time();
    pData->parseData();
    double loadTime = Time(t1);
    std::cout << yellow << "Loading time: " << loadTime << " s" << reset
              << "\n";
    pData->saveMetaData(outputFile);

    char voxelFileName[10000];
    sprintf(voxelFileName, "%s-%s", outputFile.c_str(), inputField.name().c_str());
    std::string vFile(voxelFileName);
    pData->saveVoxelsArrayData(vFile);

    std::shared_ptr<VoxelOctree> voxelAccel = std::make_shared<VoxelOctree>(
        pData->voxels.data(),
        pData->voxels.size(),
        pData->voxelRange,
        box3f(pData->gridOrigin, vec3f(pData->dimensions)),
        pData->gridWorldSpace,
        pData->worldOrigin);


    // voxelAccel->printOctree();
    voxelOctrees.push_back(voxelAccel);
  }

  // We could make this check earlier in the code for performance, i.e. don't
  // build the octree if we know that our output is unstructured. I am making
  // this check here only for simplicity.
  if (unstructured) {
    pData->dumpUnstructured(outputFile);

    char octreeFileName[10000];
    for(size_t i = 0 ; i < voxelOctrees.size();i++){
      sprintf(octreeFileName, "%s-%s%06i", outputFile.c_str(),inputField.name().c_str(), (int)i);
      std::string oFile(octreeFileName);
      voxelOctrees[i]->saveOctree(oFile);
    }
  }

  return 0;
}
