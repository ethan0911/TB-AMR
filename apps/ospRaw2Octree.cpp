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
std::string outputFile;

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
    }else if (arg == "-o" || arg == "--output"){
      outputFile = av[i + 1];
      removeArgs(ac, av, i, 2);
      --i;
    }
  }

  PRINT(intputDataType);

  if (intputDataType == "")
    throw runtime_error("Input data type must be set!!");

  if (inputFile == "" && intputDataType != "synthetic")
    throw runtime_error("Input file must be set!!");

  if (intputDataType == "exajet" && inputField == "")
    throw runtime_error("Data field must be set for the exajet data!");

  if (outputFile == "")
    throw runtime_error("Output data type must be set!!");
}

int main(int argc, const char **argv)
{

  parseCommandLind(argc, argv);

  std::vector<std::shared_ptr<VoxelOctree>> voxelOctrees;
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
    std::string input_fname = inputFile.str();//std::string(argv[4]);

    // Read info file. Use this file to decide if we want to load data, and if
    // so, how much.
    std::string info_fname = input_fname + ".info";
    std::ifstream info_fstream(info_fname, std::ios::in);
    PRINT(info_fname);

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

    // p4est data, voxeloctree traversal

    pData = std::make_shared<p4estSource>(p4est,conn);
    pData->parseData();
    pData->saveMetaData(outputFile);

    std::shared_ptr<VoxelOctree> voxelAccel = std::make_shared<VoxelOctree>(
        pData->voxels.data(),
        pData->voxels.size(),
        box3f(pData->gridOrigin, vec3f(pData->dimensions)),
        pData->gridWorldSpace);

    voxelOctrees.push_back(voxelAccel);

    p4est_topidx_t total_trees, first_local_tree, last_local_tree;
    p4est_ospray_tree_counts(
        p4est, &total_trees, &first_local_tree, &last_local_tree);
    std::cout << "Have trees [" << first_local_tree << ", " << last_local_tree
              << "] of the total " << total_trees << " trees\n";

    // for (int i = first_local_tree; i <= last_local_tree; ++i) {
    //   OSPVolume tree = ospNewVolume("p4est");
    //   ospSetVoidPtr(tree, "p4estTree", (void *)p4est);
    //   ospSet1f(tree, "samplingRate", 1.f);
    //   // ospSetVoidPtr(tree, "p4estDataCallback", (void *)load_data_callback);
    //   // ospSet1i(tree, "treeID", i);


    //   ospSet3f(tree, "gridWorldSpace", pData->gridWorldSpace.x, pData->gridWorldSpace.y, pData->gridWorldSpace.z);
    //   ospSet3i(tree, "dimensions", pData->dimensions.x, pData->dimensions.y, pData->dimensions.z);
    //   ospSetVoidPtr(tree, "voxelOctree", (void *)voxelOctrees[voxelOctrees.size() - 1]);

    //   ospSetObject(tree, "transferFunction", transferFcn);
    //   ospCommit(tree);
    //   ospAddVolume(world, tree);
    //   ospRelease(tree);
    // }
  }

  if (intputDataType == "synthetic") {
    pData = std::make_shared<syntheticSource>();
  }

  // NASA exajet data

  if (intputDataType == "exajet") {
    pData         = std::make_shared<exajetSource>(inputFile, inputField);
  }

  if (intputDataType == "synthetic" || intputDataType == "exajet") {
    time_point t1 = Time();
    pData->parseData();
    double loadTime = Time(t1);
    std::cout << yellow << "Loading time: " << loadTime << " s" << reset << "\n";
    pData->saveMetaData(outputFile);
    std::shared_ptr<VoxelOctree> voxelAccel = std::make_shared<VoxelOctree>(
        pData->voxels.data(),
        pData->voxels.size(),
        box3f(pData->gridOrigin, vec3f(pData->dimensions)),
        pData->gridWorldSpace);

    voxelOctrees.push_back(voxelAccel);
  }

  // mmap the binary file
  char octreeFileName[10000];
  for(size_t i = 0 ; i < voxelOctrees.size();i++){
    sprintf(octreeFileName, "%s%06i", outputFile.c_str(), (int)i);
    std::string oFile(octreeFileName);
    voxelOctrees[i]->saveOctree(oFile);
  }

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
