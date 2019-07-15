#include "dataImporter.h"
#include "Utils.h"
#include "ospcommon/xml/XML.h"



exajetSource::exajetSource(const FileName filePath, const string fieldName){
  this->filePath = filePath;
  this->fieldName = fieldName;
}

void exajetSource::parseData()
{
   // Open the hexahedron data file
  int hexFd           = open(filePath.c_str(), O_RDONLY);
  struct stat statBuf = {0};
  fstat(hexFd, &statBuf);
  size_t numHexes = statBuf.st_size / sizeof(Hexahedron);
  std::cout << yellow << "Loading File: " << filePath.base() << "\t"
            << "Field: " << fieldName << "\t"
            << "#Voxels: " << numHexes << reset <<"\n";
  void *hexMapping =
      mmap(NULL, statBuf.st_size, PROT_READ, MAP_PRIVATE, hexFd, 0);
  if (hexMapping == MAP_FAILED) {
    std::cout << "Failed to map hexes file\n";
    perror("hex_mapping file");
    return;
  }

  // Open the field data file
  //   const std::string cellFieldName = "y_vorticity.bin";
  const FileName fieldFile = filePath.path() + fieldName;
  // std::cout << "Loading field file: " << fieldFile << "\n";
  int fieldFd              = open(fieldFile.c_str(), O_RDONLY);
  struct stat fieldStatBuf = {0};
  fstat(fieldFd, &fieldStatBuf);
  // std::cout "\033["<< "File " << fieldFile.c_str() << "\n"
  //           << "size: " << fieldStatBuf.st_size <<"m" <<"\n";
  void *fieldMapping =
      mmap(NULL, fieldStatBuf.st_size, PROT_READ, MAP_PRIVATE, fieldFd, 0);
  if (fieldMapping == MAP_FAILED) {
    std::cout << "Failed to map field file\n";
    perror("field_mapping file");
    return;
  }

  size_t sIdx = 0;//numHexes * 0.01;

  size_t eIdx = numHexes; 

  size_t showHexsNum = eIdx - sIdx;

  const int desiredLevel = -1;

  const Hexahedron *hexes = static_cast<const Hexahedron *>(hexMapping);
  const float *cellField     = static_cast<const float *>(fieldMapping);

  // std::vector<voxel> voxels;
  this->voxels.resize(showHexsNum);

  float minWidth = std::numeric_limits<float>::max();
  box3f bounds(vec3f(0.0f)); 

  range_t<float> vRange;  

  for (size_t i = sIdx; i < eIdx; ++i) {
    const Hexahedron &h = hexes[i];
    if (desiredLevel == -1 || h.level == desiredLevel) {
      vec3f lower = vec3f(h.lower - exaJetGridMin) * exaJetVoxelScale;
      float width = (1 << h.level) * exaJetVoxelScale;
      if(width < minWidth)
        minWidth = width;
      bounds.extend(lower + vec3f(width));
      vRange.extend(cellField[i]);
      this->voxels[i-sIdx] = voxel(lower,width,cellField[i]);
    }
  }
  // PRINT(bounds);
  PRINT(vRange);

  vec3f res =vec3f(bounds.size()/minWidth);
  this->dimensions = vec3i(round(res.x),round(res.y),round(res.z));
  this->gridOrigin = vec3f(0.f);
  this->gridWorldSpace = vec3f(minWidth);
  this->worldOrigin = exaJetWorldMin;
}



void syntheticSource::parseData()
{
  float width = 1.0;
  vec3f ll = vec3f(0.f);

  voxels.push_back(voxel(ll,2 * width,4.0));
  //voxels.push_back(voxel(vec3f(0.5,0.0,0.0),0.5,6.0));
  voxels.push_back(voxel(ll + vec3f(0.0,2 * width,0.0),2 * width,8.0));
  voxels.push_back(voxel(ll + vec3f(2 * width,2 * width,0.0),2 * width,10.0));

  // voxels.push_back(voxel(ll + vec3f(0.0, 0.0, 2 * width), 2 * width, 4.0));
  // voxels.push_back(voxel(ll + vec3f(2 * width, 0.0, 2 * width), 2 * width, 6.0));
  // voxels.push_back(voxel(ll + vec3f(0.0, 2 * width, 2 * width), 2 * width, 8.0));
  // voxels.push_back(voxel(ll + vec3f(2 * width, 2 * width, 2 * width), 2 * width, 10.0));

  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0), 2 * width, 4.0));
  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0) + vec3f(2 * width, 0.0, 0.0), 2 * width, 6.0));
  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0) + vec3f(0.0, 2 * width, 0.0), 2 * width, 8.0));
  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0) + vec3f(2 * width, 2 * width, 0.0), 2 * width, 10.0));

  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0) + vec3f(0.0, 0.0, 2 * width), 2 * width, 4.0));
  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0) + vec3f(2 * width, 0.0, 2 * width), 2 * width, 6.0));
  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0) + vec3f(0.0, 2 * width, 2 * width), 2 * width, 8.0));
  // voxels.push_back(voxel(ll + vec3f(4 * width, 0.0, 0.0) + vec3f(2 * width), 2 * width, 10.0));

  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0),width,9.0));
  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0) + vec3f(width,0.0,0.0),width,9.5));
  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0) + vec3f(0.0,width,0.0),width,9.8));
  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0) + vec3f(width,width,0.0),width,10.2));
  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0) + vec3f(0.0,0.0,width),width,11.0));
  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0) + vec3f(width,0.0,width),width,11.2));
  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0) + vec3f(0.0,width,width),width,11.4));
  voxels.push_back(voxel(ll + vec3f(2 * width,0.0,0.0) + vec3f(width),width,11.6));

  this->dimensions = vec3i(4,4,2);
  this->gridWorldSpace = vec3f(width);
  this->gridOrigin = vec3f(0.f);
  this->worldOrigin = vec3f(0.f);

}



p4estSource::p4estSource(p4est_t *p4est, p4est_connectivity_t *conn){
  this->p4est = p4est;
  this->conn = conn;
}


void p4estSource::parseData()
{
  P4estDumpInfo currInfo;
  currInfo.maxLevel      = -1;
  currInfo.maxLevelWidth = -1;
  currInfo.voxel_vec     = &this->voxels;

  // call p4est_iterate here. Give the callback function a way to push to the
  // vector defined above.
  if (p4est) {
    p4est_iterate(p4est,
                  NULL,
                  &currInfo,
                  dump_p4est_callback,
                  NULL,
#ifdef P4_TO_P8
                  NULL,
#endif
                  NULL);
  } else {  // TODO: properly throw an exception here?
    std::cout << "p4est null, cannot build sparse octree from the p4est!!!"
              << std::endl;
  }

  // Number of cells of size "maxLevel" required to span the length of one side
  // of the cube.
  int numFinestCells = 1 << currInfo.maxLevel;

  this->dimensions       = vec3i(numFinestCells);
  this->gridWorldSpace = currInfo.maxLevelWidth;
  this->gridOrigin = vec3f(0.f);
  this->worldOrigin = vec3f(0.f);  // vec3f(-3.f) // for mandel data

  for (int i = 0; i < voxels.size(); i++) {
    this->voxels[i].lower -= this->worldOrigin; 
  }

  std::cout << "Num finest cells: " << numFinestCells << std::endl;
  std::cout << "max level: " << currInfo.maxLevel << std::endl;
}
