#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include "../ospray/VoxelOctree.h"
#include "ospcommon/vec.h"
#include "ospcommon/FileName.h"
#include "ospcommon/containers/AlignedVector.h"

using namespace ospcommon;
using namespace std;

const vec3i exaJetGridMin    = vec3i(1232128, 1259072, 1238336);
const float exaJetVoxelScale = 0.0005;
const vec3f exaJetWorldMin   = vec3f(-1.73575, -9.44, -3.73281);
const int  baseLevel = 6;

struct Hexahedron
{
  vec3i lower;
  int level;
};


struct DataSource{

public:
  std::vector<voxel> voxels;
  
  //! Volume size in voxels per dimension. e.g. (4 x 4 x 2) 
  vec3i dimensions; 
  //! Grid origin.
  vec3f gridOrigin;
  //! Grid spacing in each dimension in world coordinate.
  vec3f gridWorldSpace;

  vec3f worldOrigin; 

  virtual void parseDataFromFile(const FileName filePath, const string fieldName) =0; 
  
};

struct exajetSource: public DataSource{
  void parseDataFromFile(const FileName filePath, const string fieldName) override;
};



// void importExajet(const FileName fileName, const string fieldName);