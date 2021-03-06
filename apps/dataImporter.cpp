#include "dataImporter.h"
#include <fstream>
#include <vector>
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

void DataSource::dumpUnstructured(const std::string &fileName){
  std::vector<vec3f> verts;
  std::vector<vec4i> indices;
  std::vector<float> fieldData;

  // FIXME: Using naive indexing for now... for a fair comparison we would need
  // to implement non-naive indexing, possibly using hash tables - this would
  // work much like an OBJ writer.

  // Iterate through cells (Feng calls these "voxels")
  // Each cell has a lower corner, width, and value
  for (voxel& cell : voxels) {
    float oct_len = cell.width;
    float x = cell.lower.x;
    float y = cell.lower.y;
    float z = cell.lower.z;

    std::vector<uint32_t> lower_idxs; //indices 0 thru 3 for the current hex
    std::vector<uint32_t> upper_idxs; //indices 4 thru 7 for the current hex

    // Following the "winding order" found in Hexahedron.cxx from:
    // https://vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/Hexahedron
    // Because the OSPRay docs say that "for hexahedral cells... vertex ordering
    // is the same as VTK_HEXAHEDRON: four bottom vertices counterclockwise, then
    // top four counterclockwise."

    //Vertex 0
    lower_idxs.push_back(verts.size());
    verts.push_back(vec3f(x, y, z));

    //Vertex 1
    lower_idxs.push_back(verts.size());
    verts.push_back(vec3f(x + oct_len, y, z));

    //Vertex 2
    lower_idxs.push_back(verts.size());
    verts.push_back(vec3f(x + oct_len, y + oct_len, z));

    //Vertex 3
    lower_idxs.push_back(verts.size());
    verts.push_back(vec3f(x, y + oct_len, z));

    //Vertex 4
    upper_idxs.push_back(verts.size());
    verts.push_back(vec3f(x, y, z + oct_len));

    //Vertex 5
    upper_idxs.push_back(verts.size());
    verts.push_back(vec3f(x + oct_len, y, z + oct_len));

    //Vertex 6
    upper_idxs.push_back(verts.size());
    verts.push_back(vec3f(x + oct_len, y + oct_len, z + oct_len));

    //Vertex 7
    upper_idxs.push_back(verts.size());
    verts.push_back(vec3f(x, y + oct_len, z + oct_len));

    //Push indices to the index buffer
    indices.push_back(vec4i(lower_idxs[0],lower_idxs[1],lower_idxs[2],lower_idxs[3]));
    indices.push_back(vec4i(upper_idxs[0],upper_idxs[1],upper_idxs[2],upper_idxs[3]));

    fieldData.push_back(cell.value); //Assuming data is cell-centered
  }

  std::cout << "Seralizing: " << fileName << std::endl;

  /*
   *std::ofstream vertfile(fileName + ".v.unstruct");
   *for (vec3f& v : verts) {
   *  vertfile << v.x << " " << v.y << " " << v.z << std::endl;
   *}
   *vertfile.close();
   */
  std::ofstream vertfile(fileName + ".v.unstruct", ios::out | ios::trunc | ios::binary);
  size_t num_vert_bytes = sizeof(vec3f)*verts.size();
  std::cout << "Writing " << num_vert_bytes << " bytes of vert data!" << std::endl;
  vertfile.write(reinterpret_cast<const char*>(verts.data()), num_vert_bytes);
  vertfile.close();

  /*
   *std::ofstream indfile(fileName + ".i.unstruct");
   *for (vec4i& i : indices) {
   *  indfile << i[0] << " " << i[1] << " " << i[2] << " " << i[3] << std::endl;
   *}
   *indfile.close();
   */
  std::ofstream indfile(fileName + ".i.unstruct", ios::out | ios::trunc | ios::binary);
  size_t num_ind_bytes = sizeof(vec4i)*indices.size();
  std::cout << "Writing " << num_ind_bytes << " bytes of index data!" << std::endl;
  indfile.write(reinterpret_cast<const char*>(indices.data()), num_ind_bytes);
  indfile.close();

  std::ofstream fieldfile(fileName + ".f.unstruct", ios::out | ios::trunc | ios::binary);
  size_t num_field_bytes = sizeof(float)*fieldData.size();
  std::cout << "Writing " << num_field_bytes << " bytes of field data!" << std::endl;
  fieldfile.write(reinterpret_cast<const char*>(fieldData.data()), num_field_bytes);
  fieldfile.close();
}

exajetSource::exajetSource(const FileName filePath, const string fieldName){
  this->filePath = filePath;
  this->fieldName = fieldName;
}

exajetSource::exajetSource(const FileName filePath,
                           const string fieldName,
                           vec3i gridMin,
                           float voxelScale,
                           vec3f worldOrigin)
{
  this->filePath    = filePath;
  this->fieldName   = fieldName;
  this->exaJetGridMin     = gridMin;
  this->exaJetVoxelScale  = voxelScale;
  this->exaJetWorldOrigin = worldOrigin;
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

  range1f vRange;

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
  this->voxelRange     = vRange;
}


void syntheticSource::parseData()
{
  float width = 1.0;
  vec3f worldOrigin = vec3f(0.f);

#if 1
  float minGridWidth = width;
  vec3i gridDim(4,4,4);

  range1f rg;

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
          rg.extend(v.value);
          voxels.push_back(v);
        } else {
          cellWidth = 0.5 * width;
          minGridWidth = min(minGridWidth, cellWidth);
          for (int i = 0; i < 8; i++) {
            vec3f childLower = lower + cellWidth * vec3f(i & 1 ? 1 : 0, i & 2 ? 1 : 0, i & 4 ? 1 : 0);
            vec3f childCenter = childLower - worldOrigin + vec3f(0.5 * cellWidth);
            voxel v(childLower, cellWidth , childCenter.x * childCenter.y * childCenter.z);
            rg.extend(v.value);
            voxels.push_back(v);
          }
        }
      }

  this->dimensions     = 1.f / minGridWidth * gridDim ;
  this->gridWorldSpace = vec3f(minGridWidth);
  this->gridOrigin     = vec3f(0.f);
  this->worldOrigin    = worldOrigin;
  this->voxelRange     = rg;

#elif 0

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
  this->voxelRange = range1f(4.f,8.f);

#endif

}
