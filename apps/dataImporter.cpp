#include "dataImporter.h"
#include "Utils.h"
#include "ospcommon/xml/XML.h"



void DataSource::saveMetaData(const std::string &fileName)
{
  FILE *meta = fopen(fileName.c_str(), "w");
  fprintf(meta, "<?xml?>\n");
  fprintf(meta, "<ospray>\n");
  {
    fprintf(meta, "  <Metadata\n");
    {
      fprintf(meta,"    dimensions=\"%i %i %i\"\n", dimensions.x, dimensions.y, dimensions.z);
      fprintf(meta,"    gridOrigin=\"%f %f %f\"\n", gridOrigin.x, gridOrigin.y, gridOrigin.z);
      fprintf(meta,"    gridWorldSpace=\"%f %f %f\"\n", gridWorldSpace.x, gridWorldSpace.y, gridWorldSpace.z);
      fprintf(meta,"    worldOrigin=\"%f %f %f\"\n", worldOrigin.x, worldOrigin.y, worldOrigin.z);
      fprintf(meta,"    voxelNum=\"%zu\"\n", this->voxels.size());
      fprintf(meta,"    >\n");
    }
    fprintf(meta, "  </Metadata>\n");
  }
  fprintf(meta, "</ospray>\n");
  fclose(meta);
}

void DataSource::mapMetaData(const std::string &fileName)
{
  std::shared_ptr<xml::XMLDoc> doc = xml::readXML(fileName.c_str());
  if (!doc)
    throw std::runtime_error("could not read metadata file:" + fileName);

  std::shared_ptr<xml::Node> osprayNode = std::make_shared<xml::Node>(doc->child[0]);
  assert(osprayNode->name == "ospray");

  std::shared_ptr<xml::Node> metaDataNode = std::make_shared<xml::Node>(osprayNode->child[0]);
  assert(metaDataNode->name == "Metadata");

  sscanf(metaDataNode->getProp("dimensions").c_str(),"%i %i %i",&dimensions.x, &dimensions.y, &dimensions.z);

  sscanf(metaDataNode->getProp("gridOrigin").c_str(),"%f %f %f",
        &gridOrigin.x, &gridOrigin.y, &gridOrigin.z);

  sscanf(metaDataNode->getProp("gridWorldSpace").c_str(),"%f %f %f",
        &gridWorldSpace.x, &gridWorldSpace.y, &gridWorldSpace.z);

  sscanf(metaDataNode->getProp("worldOrigin").c_str(),"%f %f %f",
        &worldOrigin.x, &worldOrigin.y, &worldOrigin.z);

  sscanf(metaDataNode->getProp("voxelNum").c_str(), "%zu", &voxelNum);
}

void DataSource::saveVoxelsArrayData(const std::string &fileName)
{
  const std::string voxlBinFile = fileName + ".vxl";
  FILE *voxelsFile              = fopen(voxlBinFile.c_str(), "wb");

  if (!fwrite(voxels.data(), sizeof(voxel), voxels.size(), voxelsFile))
    throw std::runtime_error("Could not write" + voxlBinFile);
  fclose(voxelsFile);
}

void DataSource::mapVoxelsArrayData(const std::string &fileName)
{
  std::string voxelFileName = fileName + ".vxl";
  FILE *file                = fopen(voxelFileName.c_str(), "rb");
  if (!file)
    throw std::runtime_error("could not open voxel bin file " + voxelFileName);

  this->voxels.resize(voxelNum);
  fread(this->voxels.data(), sizeof(voxel), voxelNum, file);
  fclose(file);
}

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
            << "#Voxels: " << numHexes << reset << "\n";

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

  size_t sIdx = 0;
  size_t eIdx =  numHexes; 
  size_t showHexsNum = eIdx - sIdx;

  const int desiredLevel = -1;

  const Hexahedron *hexes = static_cast<const Hexahedron *>(hexMapping);
  const float *cellField     = static_cast<const float *>(fieldMapping);

  this->voxels.resize(showHexsNum);

  float minWidth = std::numeric_limits<float>::max();
  box3f bounds(vec3f(0.0f)); 

  range_t<float> vRange;  

  for (size_t i = sIdx; i < eIdx; ++i) {
    const Hexahedron &h = hexes[i];
    if (desiredLevel == -1 || h.level == desiredLevel) {
      vec3f lower = vec3f(h.lower - exaJetGridMin) * exaJetVoxelScale + exaJetWorldOrigin;
      float width = (1 << h.level) * exaJetVoxelScale;
      if(width < minWidth)
        minWidth = width;
      bounds.extend(lower + vec3f(width));
      vRange.extend(cellField[i]);
      this->voxels[i-sIdx] = voxel(lower,width,cellField[i]);
    }
  }

  PRINT(vRange);

  vec3f res =vec3f(bounds.size()/minWidth);
  this->dimensions = vec3i(round(res.x),round(res.y),round(res.z));
  this->gridOrigin = vec3f(0.f);
  this->gridWorldSpace = vec3f(minWidth);
  this->worldOrigin = exaJetWorldOrigin;

}



void syntheticSource::parseData()
{
  float width = 1.0;
  vec3f worldOrigin = vec3f(0.f);

#if 1
  float minGridWidth = width;
  vec3i gridDim(2,2,2);

  for (int z = 0; z < gridDim.z; z++)
    for (int y = 0; y < gridDim.y; y++)
      for (int x = 0; x < gridDim.x; x++) {
        bool isFinerCell = x < gridDim.x * 0.5;
        int level; 
        float cellWidth;

        vec3f lower = worldOrigin + width * vec3f(x, y, z);
        if(!isFinerCell)
        {
          cellWidth = width;
          vec3f center = lower - worldOrigin + vec3f(0.5 * cellWidth);
          voxel v(lower, cellWidth, center.x * center.y * center.z);
          minGridWidth = min(minGridWidth, cellWidth);
          voxels.push_back(v);
        } else {
          cellWidth = 0.5 * width;
          minGridWidth = min(minGridWidth, cellWidth);
          for (int i = 0; i < 8; i++) {
            vec3f childLower = lower + cellWidth * vec3f(i & 1 ? 1 : 0, i & 2 ? 1 : 0, i & 4 ? 1 : 0);
            vec3f childCenter = childLower - worldOrigin + vec3f(0.5 * cellWidth);
            voxel v(childLower, cellWidth , childCenter.x * childCenter.y * childCenter.z);
            voxels.push_back(v);
          }
        }
      }

  this->dimensions     = 1.f / minGridWidth * gridDim ;
  this->gridWorldSpace = vec3f(minGridWidth);
  this->gridOrigin     = vec3f(0.f);
  this->worldOrigin    = worldOrigin;

#elif 1

  float cellWidth = 2 * width;
  float halfCellWidth = width;

  vec3f lo0, lo1, lo2, lo3, lo4, lo5, lo6, lo7;
  lo0 = worldOrigin;
  lo1 = worldOrigin + vec3f(cellWidth, 0.f, 0.f);
  lo2 = worldOrigin + vec3f(0.f, cellWidth, 0.f);
  lo3 = worldOrigin + vec3f(cellWidth, cellWidth, 0.f);
  lo4 = worldOrigin + vec3f(0.f, 0.f, cellWidth);
  lo5 = worldOrigin + vec3f(cellWidth, 0.f, cellWidth);
  lo6 = worldOrigin + vec3f(0.f, cellWidth, cellWidth);
  lo7 = worldOrigin + vec3f(cellWidth);

  voxels.push_back(voxel(lo0, cellWidth, 1.0));
  voxels.push_back(voxel(lo1, cellWidth, 2.0));
  voxels.push_back(voxel(lo2, cellWidth, 3.0));
  voxels.push_back(voxel(lo3, cellWidth, 4.0));

  voxels.push_back(voxel(lo4, cellWidth, 5.0));
  // voxels.push_back(voxel(lo5, cellWidth, 6.0));
  // voxels.push_back(voxel(lo6, cellWidth, 7.0));
  // voxels.push_back(voxel(lo7, cellWidth, 8.0));

  voxels.push_back(voxel(lo5, halfCellWidth, 6.0));
  voxels.push_back(voxel(lo5 + vec3f(halfCellWidth,0.0,0.0),halfCellWidth,6.1)); 
  voxels.push_back(voxel(lo5 + vec3f(0.0,halfCellWidth,0.0),halfCellWidth,6.2));
  voxels.push_back(voxel(lo5 + vec3f(halfCellWidth,halfCellWidth,0.0),halfCellWidth,6.3)); 
  voxels.push_back(voxel(lo5 + vec3f(0.0,0.0,halfCellWidth),halfCellWidth,6.4));
  voxels.push_back(voxel(lo5 + vec3f(halfCellWidth,0.0,halfCellWidth),halfCellWidth,6.5));
  voxels.push_back(voxel(lo5 + vec3f(0.0,halfCellWidth,halfCellWidth),halfCellWidth,6.6));
  voxels.push_back(voxel(lo5 + vec3f(halfCellWidth),halfCellWidth,6.7));

  voxels.push_back(voxel(lo6, halfCellWidth, 7.0));
  voxels.push_back(voxel(lo6 + vec3f(halfCellWidth,0.0,0.0),halfCellWidth,7.1)); 
  voxels.push_back(voxel(lo6 + vec3f(0.0,halfCellWidth,0.0),halfCellWidth,7.2));
  voxels.push_back(voxel(lo6 + vec3f(halfCellWidth,halfCellWidth,0.0),halfCellWidth,7.3)); 
  voxels.push_back(voxel(lo6 + vec3f(0.0,0.0,halfCellWidth),halfCellWidth,7.4));
  voxels.push_back(voxel(lo6 + vec3f(halfCellWidth,0.0,halfCellWidth),halfCellWidth,7.5));
  voxels.push_back(voxel(lo6 + vec3f(0.0,halfCellWidth,halfCellWidth),halfCellWidth,7.6));
  voxels.push_back(voxel(lo6 + vec3f(halfCellWidth),halfCellWidth,7.7));

  voxels.push_back(voxel(lo7, halfCellWidth, 8.0));
  voxels.push_back(voxel(lo7 + vec3f(halfCellWidth,0.0,0.0),halfCellWidth,8.1)); 
  voxels.push_back(voxel(lo7 + vec3f(0.0,halfCellWidth,0.0),halfCellWidth,8.2));
  voxels.push_back(voxel(lo7 + vec3f(halfCellWidth,halfCellWidth,0.0),halfCellWidth,8.3)); 
  voxels.push_back(voxel(lo7 + vec3f(0.0,0.0,halfCellWidth),halfCellWidth,8.4));
  voxels.push_back(voxel(lo7 + vec3f(halfCellWidth,0.0,halfCellWidth),halfCellWidth,8.5));
  voxels.push_back(voxel(lo7 + vec3f(0.0,halfCellWidth,halfCellWidth),halfCellWidth,8.6));
  voxels.push_back(voxel(lo7 + vec3f(halfCellWidth),halfCellWidth,8.7));

  this->dimensions = vec3i(4,4,4);
  this->gridWorldSpace = vec3f(halfCellWidth);
  this->gridOrigin = vec3f(0.f);
  this->worldOrigin = worldOrigin;

#else

  voxels.push_back(voxel(worldOrigin, 2 * width, 4.0));
  // voxels.push_back(voxel(ll + vec3f(2 * width, 0.0, 0.0), 2 * width, 4.0));
  voxels.push_back(voxel(worldOrigin + vec3f(0.0, 2 * width, 0.0), 2 * width, 6.0));
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width, 2 * width, 0.0), 2 * width, 8.0));

  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0),width,5.0));
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0) + vec3f(width,0.0,0.0),width,7.0)); 
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0) + vec3f(0.0,width,0.0),width,5.0));
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0) + vec3f(width,width,0.0),width,7.0)); 
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0) + vec3f(0.0,0.0,width),width,5.0));
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0) + vec3f(width,0.0,width),width,7.0));
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0) + vec3f(0.0,width,width),width,5.0));
  voxels.push_back(voxel(worldOrigin + vec3f(2 * width,0.0,0.0) + vec3f(width),width,7.0));

  this->dimensions = vec3i(4,4,2);
  this->gridWorldSpace = vec3f(width);
  this->gridOrigin = vec3f(0.f);
  this->worldOrigin = worldOrigin;

#endif

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
  this->gridWorldSpace   = currInfo.maxLevelWidth;
  this->gridOrigin       = vec3f(0.f);
  this->worldOrigin      = vec3f(0.f);  // vec3f(-3.f) // for mandel data

  for (int i = 0; i < voxels.size(); i++) {
    this->voxels[i].lower -= this->worldOrigin;
  }

  std::cout << "Num finest cells: " << numFinestCells << std::endl;
  std::cout << "max level: " << currInfo.maxLevel << std::endl;
}
