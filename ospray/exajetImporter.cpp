#include "exajetImporter.h"

void exajetSource::parseDataFromFile(const FileName filePath, const string fieldName)
{
   // Open the hexahedron data file
  int hexFd           = open(filePath.c_str(), O_RDONLY);
  struct stat statBuf = {0};
  fstat(hexFd, &statBuf);
  size_t numHexes = statBuf.st_size / sizeof(Hexahedron);
  std::cout << "File " << filePath.c_str() << "\n"
            << "size: " << statBuf.st_size << "\n"
            << "#hexes: " << numHexes << "\n";
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
  std::cout << "Loading field file: " << fieldFile << "\n";
  int fieldFd              = open(fieldFile.c_str(), O_RDONLY);
  struct stat fieldStatBuf = {0};
  fstat(fieldFd, &fieldStatBuf);
  std::cout << "File " << fieldFile.c_str() << "\n"
            << "size: " << fieldStatBuf.st_size << "\n";
  void *fieldMapping =
      mmap(NULL, fieldStatBuf.st_size, PROT_READ, MAP_PRIVATE, fieldFd, 0);
  if (fieldMapping == MAP_FAILED) {
    std::cout << "Failed to map field file\n";
    perror("field_mapping file");
    return;
  }

  size_t sIdx = 0;//numHexes * 0.01;

  size_t eIdx = 20000;//numHexes; 

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
  PRINT(bounds);
  PRINT(vRange);

  vec3f res =vec3f(bounds.size()/minWidth);
  this->dimensions = vec3i(round(res.x),round(res.y),round(res.z));
  this->gridOrigin = vec3f(0.f);
  this->gridWorldSpace = vec3f(minWidth);
  this->worldOrigin = exaJetWorldMin;
  // PRINT(res);
  // PRINT(this->dimensions);

  // for (size_t i = 0; i < this->voxels.size(); i++) {
  //   printf("coord:[%f,%f,%f], width:%f, value: %f\n",
  //         this->voxels[i].lower.x,
  //         this->voxels[i].lower.y,
  //         this->voxels[i].lower.z,
  //         this->voxels[i].width,
  //         this->voxels[i].value);
  // }
}



// void importExajet(const FileName fileName, const string fieldName)
// {
//   // Open the hexahedron data file
//   int hexFd           = open(fileName.c_str(), O_RDONLY);
//   struct stat statBuf = {0};
//   fstat(hexFd, &statBuf);
//   size_t numHexes = statBuf.st_size / sizeof(Hexahedron);
//   std::cout << "File " << fileName.c_str() << "\n"
//             << "size: " << statBuf.st_size << "\n"
//             << "#hexes: " << numHexes << "\n";
//   void *hexMapping =
//       mmap(NULL, statBuf.st_size, PROT_READ, MAP_PRIVATE, hexFd, 0);
//   if (hexMapping == MAP_FAILED) {
//     std::cout << "Failed to map hexes file\n";
//     perror("hex_mapping file");
//     return;
//   }

//   // Open the field data file
//   //   const std::string cellFieldName = "y_vorticity.bin";
//   const FileName fieldFile = fileName.path() + fieldName;
//   std::cout << "Loading field file: " << fieldFile << "\n";
//   int fieldFd              = open(fieldFile.c_str(), O_RDONLY);
//   struct stat fieldStatBuf = {0};
//   fstat(fieldFd, &fieldStatBuf);
//   std::cout << "File " << fieldFile.c_str() << "\n"
//             << "size: " << fieldStatBuf.st_size << "\n";
//   void *fieldMapping =
//       mmap(NULL, fieldStatBuf.st_size, PROT_READ, MAP_PRIVATE, fieldFd, 0);
//   if (fieldMapping == MAP_FAILED) {
//     std::cout << "Failed to map field file\n";
//     perror("field_mapping file");
//     return;
//   }

//   numHexes = 10; 

//   const int desiredLevel = -1;

//   const Hexahedron *hexes = static_cast<const Hexahedron *>(hexMapping);
//   const float *cellField     = static_cast<const float *>(fieldMapping);

//   std::vector<voxel> voxels;
//   voxels.resize(numHexes);

//   for (size_t i = 0; i < numHexes; ++i) {
//     const Hexahedron &h = hexes[i];
//     if (desiredLevel == -1 || h.level == desiredLevel) {
//       vec3f lower = vec3f(h.lower - exaJetGridMin) * exaJetVoxelScale;
//       float width = (1 << h.level) * exaJetVoxelScale;
//       voxels[i] = voxel(lower,width,cellField[i]);
//     }
//   }

//   for (size_t i = 0; i < numHexes; i++) {
//     printf("coord:[%f,%f,%f], width:%f, value: %f\n",
//           voxels[i].lower.x,
//           voxels[i].lower.y,
//           voxels[i].lower.z,
//           voxels[i].width,
//           voxels[i].value);
//   }
// }
