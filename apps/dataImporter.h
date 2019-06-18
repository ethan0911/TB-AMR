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

  virtual void parseData() =0; 

  virtual void saveMetaData(const std::string &fileName) = 0;
  virtual void mapMetaData(const std::string &fileName) = 0;
  
};

inline void saveMeta(const std::string &fileName,
                     vec3i &dim,
                     vec3f &gridOrigin,
                     vec3f &gridWorldSpace,
                     vec3f &worldOrigin)
{
  FILE *meta = fopen(fileName.c_str(), "w");
  fprintf(meta, "<?xml?>\n");
  fprintf(meta, "<ospray>\n");
  {
    fprintf(meta, "  <Metadata\n");
    {
      fprintf(meta,"    dimensions=\"%i %i %i\"\n", dim.x, dim.y, dim.z);
      fprintf(meta,"    gridOrigin=\"%f %f %f\"\n", gridOrigin.x, gridOrigin.y, gridOrigin.z);
      fprintf(meta,"    gridWorldSpace=\"%f %f %f\"\n", gridWorldSpace.x, gridWorldSpace.y, gridWorldSpace.z);
      fprintf(meta,"    worldOrigin=\"%f %f %f\"\n", worldOrigin.x, worldOrigin.y, worldOrigin.z);
      fprintf(meta,"    >\n");
    }
    fprintf(meta, "  </Metadata>\n");
  }
  fprintf(meta, "</ospray>\n");
  fclose(meta);
}

inline void mapMeta(const std::string &fileName,
                    vec3i &dim,
                    vec3f &gridOrigin,
                    vec3f &gridWorldSpace,
                    vec3f &worldOrigin)
{
  std::shared_ptr<xml::XMLDoc> doc = xml::readXML(fileName.c_str());
  if (!doc)
    throw std::runtime_error("could not read metadata file:" + fileName);

  std::shared_ptr<xml::Node> osprayNode = std::make_shared<xml::Node>(doc->child[0]);
  assert(osprayNode->name == "ospray");

  std::shared_ptr<xml::Node> metaDataNode = std::make_shared<xml::Node>(osprayNode->child[0]);
  assert(metaDataNode->name == "Metadata");

  sscanf(metaDataNode->getProp("dimensions").c_str(),"%i %i %i",&dim.x, &dim.y, &dim.z);

  sscanf(metaDataNode->getProp("gridOrigin").c_str(),"%f %f %f",
        &gridOrigin.x, &gridOrigin.y, &gridOrigin.z);

  sscanf(metaDataNode->getProp("gridWorldSpace").c_str(),"%f %f %f",
        &gridWorldSpace.x, &gridWorldSpace.y, &gridWorldSpace.z);

  sscanf(metaDataNode->getProp("worldOrigin").c_str(),"%f %f %f",
         &worldOrigin.x, &worldOrigin.y, &worldOrigin.z);
}


struct exajetSource: public DataSource{

  exajetSource(){};
  exajetSource(const FileName filePath, const string fieldName);
  void parseData() override;
  void saveMetaData(const std::string &fileName) override
  {
    saveMeta(fileName, dimensions, gridOrigin, gridWorldSpace, worldOrigin);
  }

  void mapMetaData(const std::string &fileName)
  {
    mapMeta(fileName, dimensions, gridOrigin, gridWorldSpace, worldOrigin);
  }

  private:
  FileName filePath;
  string fieldName;
};


struct syntheticSource : public DataSource{
  void parseData() override;
  void saveMetaData(const std::string &fileName) override
  {
    saveMeta(fileName, dimensions, gridOrigin, gridWorldSpace, worldOrigin);
  }

  void mapMetaData(const std::string &fileName)
  {
    mapMeta(fileName, dimensions, gridOrigin, gridWorldSpace, worldOrigin);
  }
};

///////////////////////////////////////////////////////////////
// Generate a list of voxel from p4est data
///////////////////////////////////////////////////////////////
struct P4estDumpInfo
{
  int maxLevel;
  float maxLevelWidth;
  std::vector<voxel> *voxel_vec;
};

static double get_data_from_quadrant_copy(const p4est_quadrant_t* o, const p4est_t* p4est, p4est_topidx_t which_tree){
  size_t data_size = p4est->data_size;
  if(data_size > 0){
    //TODO: Check if static_cast is the right type of cast
    char* curr_data = static_cast<char*>(o->p.user_data);

    // Loop through the buffer, and print the contents at a hex
    // string (one hex character per nibble, 68 bytes = 136 nibbles)
    // Concatenate the hex characters to a stringstream

    //Interpret the most significant 8 bytes as floats. Ignore the least significant 4 bytes.

    double *double1 = reinterpret_cast<double*>(curr_data + 60); 
    double *double2 = reinterpret_cast<double*>(curr_data + 52); 
    double *double3 = reinterpret_cast<double*>(curr_data + 44); 
    double *double4 = reinterpret_cast<double*>(curr_data + 36); 
    double *double5 = reinterpret_cast<double*>(curr_data + 28); 
    double *double6 = reinterpret_cast<double*>(curr_data + 20); 
    double *double7 = reinterpret_cast<double*>(curr_data + 12); 
    double *double8 = reinterpret_cast<double*>(curr_data + 4); 

    double avg = (*double1 + *double2 + *double3 + *double4 + *double5 + *double6 + *double7 + *double8)/8;
    return avg;
  } else{ //No data
    // HACK: return the treeid divided by 3 for testing purposes (assuming we
    // have tree indices 0 thru 4 in some simle test data)
    return which_tree / 3.0f;
  }
}

static void dump_p4est_callback(p4est_iter_volume_info_t *info, void *user_data)
{
  P4estDumpInfo *userInfo       = reinterpret_cast<P4estDumpInfo *>(user_data);
  std::vector<voxel> *voxel_vec = userInfo->voxel_vec;
  p4est_quadrant_t *o = info->quad;  // o is the current octant

  double aabb[6];
  double *lower_corner = &aabb[0];
  double *upper_corner = &aabb[3];
  p4est_ospray_quadrant_aabb(info->p4est, info->treeid, o, aabb);

  vec3f lowerCorner(lower_corner[0], lower_corner[1], lower_corner[2]);

  // this model only really makes sense for cell-centered data...
  double cellValue = get_data_from_quadrant_copy(o, info->p4est, info->treeid);
  double cellWidth = upper_corner[0] - lower_corner[0];

  // Bold Assumption: cells are square (SAME length, width, height)
  if (o->level > userInfo->maxLevel) {
    userInfo->maxLevel      = o->level;
    userInfo->maxLevelWidth = cellWidth;
  }

  voxel_vec->push_back(voxel(lowerCorner, cellWidth, cellValue));
}

struct p4estSource: public DataSource{

  p4estSource(){};
  p4estSource(p4est_t *p4est, p4est_connectivity_t *conn);
  ~p4estSource(){};
  void parseData() override;
  void saveMetaData(const std::string &fileName) override
  {
    saveMeta(fileName, dimensions, gridOrigin, gridWorldSpace, worldOrigin);
  }

  void mapMetaData(const std::string &fileName)
  {
    mapMeta(fileName, dimensions, gridOrigin, gridWorldSpace, worldOrigin);
  }

 private:
  p4est_t *p4est;
  p4est_connectivity_t *conn;
};

