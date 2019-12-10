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
#include "ospcommon/math/vec.h"
#include "ospcommon/math/range.h"
#include "ospcommon/os/FileName.h"
#include "ospcommon/containers/AlignedVector.h"
#include "ospcommon/tasking/parallel_for.h"

using namespace ospcommon;
using namespace ospcommon::math;
using namespace std;

struct Hexahedron
{
  vec3i lower;
  int level;
};

struct DataSource{
public:
  std::vector<voxel> voxels;

  size_t voxelNum;

  range1f voxelRange;

  //! Volume size in voxels per dimension. e.g. (4 x 4 x 2)
  vec3i dimensions;
  //! Grid origin.
  vec3f gridOrigin;
  //! Grid spacing in each dimension in world coordinate.
  vec3f gridWorldSpace;
  vec3f worldOrigin;

  virtual void parseData() =0;

  void saveMetaData(const std::string &fileName);
  void saveVoxelsArrayData(const std::string &fileName);
  void mapMetaData(const std::string &fileName);
  void mapVoxelsArrayData(const std::string &fileName);
  void dumpUnstructured(const std::string &fileName);
};


/**************************************************************
// NASA Exajet Dataset
**************************************************************/
struct exajetSource: public DataSource{

  exajetSource(){};
  exajetSource(const FileName filePath, const string fieldName);
  exajetSource(const FileName filePath,
               const string fieldName,
               vec3i gridMin,
               float voxelScale,
               vec3f worldOrigin);
  void parseData() override;

 private:
  FileName filePath;
  string fieldName;

  vec3i exaJetGridMin;
  float exaJetVoxelScale;
  vec3f exaJetWorldOrigin;
};


/**************************************************************
// Synthetic dataset
**************************************************************/
struct syntheticSource : public DataSource{
  void parseData() override;
};
