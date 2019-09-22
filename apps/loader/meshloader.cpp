#define TINYOBJLOADER_IMPLEMENTATION  // define this in only *one* .cc

#include "meshloader.h"
#include <limits>
#include <set>

void WarnAlways(std::string str)
{
  std::cerr << std::endl
            << "\033[1;33m"
            << "[Warning] " << str << "\033[0m" << std::endl
            << std::endl;
}
void WarnOnce(std::string str)
{
  static std::set<std::string> warned;
  if (warned.find(str) == warned.end()) {
    WarnAlways(str);
    warned.insert(str);
  }
}
void ErrorNoExit(std::string str)
{
  std::cerr << std::endl
            << "\033[1;31m"
            << "[Error] " << str << "\033[0m" << std::endl
            << std::endl;
}
void ErrorFatal(std::string str)
{
  ErrorNoExit(str);
  exit(EXIT_FAILURE);
}

std::string ParsePath(const std::string &str)
{
  std::string cstr = str;
#if defined(WIN32) || defined(_WIN32) || \
    defined(__WIN32) && !defined(__CYGWIN__)
  std::replace(cstr.begin(), cstr.end(), '/', '\\');
#else
  std::replace(cstr.begin(), cstr.end(), '\\', '/');
#endif
  return cstr;
}

void Mesh::ComputePath(const std::string &str)
{
  fpath    = ParsePath(str);
  size_t p = fpath.find_last_of("/\\");
  if (p != std::string::npos) {
    dpath = fpath.substr(0, p + 1);
    fname = fpath.substr(p + 1, fpath.size() - dpath.size());
  } else {
    dpath = "";
    fname = fpath;
  }
}

void Mesh::TinyObjLoader::Clear()
{
  err.clear();
  attributes.vertices.clear();
  attributes.normals.clear();
  attributes.texcoords.clear();
  shapes.clear();
}

void Mesh::SetTransform(const affine3f &xfm)
{
  transform = affine3f(xfm);
}

bool Mesh::LoadFromFileObj(const char *filename, bool loadMtl)
{
  // initialize
  tiny.Clear();
  ComputePath(filename);

  // load mesh from file using tiny obj loader
  bool succeed = tinyobj::LoadObj(&(tiny.attributes),
                                  &(tiny.shapes),
                                  &(tiny.materials),
                                  &(tiny.err),
                                  fpath.c_str(),
                                  dpath == "" ? nullptr : dpath.c_str(),
                                  loadMtl);
  if (!tiny.err.empty()) {
    ErrorNoExit(tiny.err);
  }
  if (!succeed) {
    return false;
  }

  // initialize bounding box
  bbox.upper = vec3f(std::numeric_limits<float>::min());
  bbox.lower = vec3f(std::numeric_limits<float>::max());

  // initialize geometries array
  geometries.resize(tiny.shapes.size());

  // process geometry
  for (size_t s = 0; s < tiny.shapes.size(); s++)  // Loop over shapes
  {
    Geometry &geo = geometries[s];
    // note: it seems tiny obj loader uses per-face material
    //       but we need only one material per geometry
    //       so we use the first face for material index
    geo.num_faces = tiny.shapes[s].mesh.num_face_vertices.size();
    if (geo.num_faces <= 0) {
      WarnAlways("shape #" + std::to_string(s) +
                 "found one shape with no faces");
    }

    size_t vidx_offset = 0;
    for (size_t f = 0; f < geo.num_faces; f++)  // Loop over faces (polygon)
    {
      // number of vertices of this face
      int fv = tiny.shapes[s].mesh.num_face_vertices[f];
      if (fv != 3) {
        ErrorNoExit("this mesh is not a pure trianglar mesh");
        return false;
      }

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv /*fv=3*/; v++) {
        tinyobj::index_t idx = tiny.shapes[s].mesh.indices[vidx_offset + v];
// WILL NOTE: HACK TO TRANSFORM LANDING GEAR: -translate 15.995 16 0.1
#if 1
        float vx = tiny.attributes.vertices[3 * idx.vertex_index + 0] + 15.995;
        float vy = tiny.attributes.vertices[3 * idx.vertex_index + 1] + 16;
        float vz = tiny.attributes.vertices[3 * idx.vertex_index + 2] + 0.1;
#else
        float vx = tiny.attributes.vertices[3 * idx.vertex_index + 0];
        float vy = tiny.attributes.vertices[3 * idx.vertex_index + 1];
        float vz = tiny.attributes.vertices[3 * idx.vertex_index + 2];
#endif
        geo.index.push_back(geo.index.size());
        geo.vertex.push_back(vx);
        geo.vertex.push_back(vy);
        geo.vertex.push_back(vz);
        bbox.upper = max(bbox.upper, vec3f(vx, vy, vz));
        bbox.lower = min(bbox.lower, vec3f(vx, vy, vz));
        // check normal
        if (idx.normal_index >= 0) {
          geo.has_normal = true;
          float nx       = tiny.attributes.normals[3 * idx.normal_index + 0];
          float ny       = tiny.attributes.normals[3 * idx.normal_index + 1];
          float nz       = tiny.attributes.normals[3 * idx.normal_index + 2];
          geo.normal.push_back(nx);
          geo.normal.push_back(ny);
          geo.normal.push_back(nz);
        } else {
          WarnOnce("normal not found");
        }
        // check texture coordinate
        if (idx.texcoord_index >= 0) {
          geo.has_texcoord = true;
          float tx = tiny.attributes.texcoords[2 * idx.texcoord_index + 0];
          float ty = tiny.attributes.texcoords[2 * idx.texcoord_index + 1];
          geo.texcoord.push_back(tx);
          geo.texcoord.push_back(ty);
        } else {
          WarnOnce("texture coordinate not found");
        }
      }
      vidx_offset += fv;
    }
  }

  center = 0.5f * (bbox.upper + bbox.lower);
  return true;
}

void Mesh::LoadFromVTKFile(const FileName &fileName, Geometry &geo)
{
  vtkDataSet *dataSet = readVTKFile<vtkXMLPolyDataReader>(fileName.c_str());

  int numberOfCells  = dataSet->GetNumberOfCells();
  int numberOfPoints = dataSet->GetNumberOfPoints();

  geo.num_faces = numberOfCells;

  double point[3];
  for (int i = 0; i < numberOfPoints; i++) {
    dataSet->GetPoint(i, point);
    geo.vertex.push_back(point[0]);
    geo.vertex.push_back(point[1]);
    geo.vertex.push_back(point[2]);
  }

  for (int i = 0; i < numberOfCells; i++) {
    vtkCell *cell = dataSet->GetCell(i);

    if (cell->GetCellType() == VTK_TRIANGLE) {
      geo.index.push_back(cell->GetPointId(0));
      geo.index.push_back(cell->GetPointId(1));
      geo.index.push_back(cell->GetPointId(2));
    }
  }
}

void Mesh::LoadMesh(std::vector<std::string> inputMesh)
{
  geometries.resize(inputMesh.size());

  int geoSize = (int)inputMesh.size();

  ospcommon::tasking::parallel_for(geoSize, [&](int geoID) {
    LoadFromVTKFile(inputMesh[geoID], geometries[geoID]);
  });
}

void Mesh::AddToModel(OSPWorld world, OSPMaterial mtl)
{
#if 0
  for (auto &geo : geometries) {
    if (geo.num_faces != 0) {
      OSPGeometry gdata = ospNewGeometry("triangles");

      // index
      OSPData idata = ospNewData(geo.index.size() / 3,
                                 OSP_VEC3I,
                                 geo.index.data(),
                                 OSP_DATA_SHARED_BUFFER);
      ospCommit(idata);
      ospSetObject(gdata, "index", idata);
      ospRelease(idata);

      // vertex
      OSPData vdata = ospNewData(geo.vertex.size() / 3,
                                 OSP_VEC3F,
                                 geo.vertex.data(),
                                 OSP_DATA_SHARED_BUFFER);
      ospCommit(vdata);
      ospSetObject(gdata, "vertex", vdata);
      ospRelease(vdata);

      // normal
      if (geo.has_normal) {
        OSPData ndata = ospNewData(geo.normal.size() / 3,
                                   OSP_VEC3F,
                                   geo.normal.data(),
                                   OSP_DATA_SHARED_BUFFER);
        ospCommit(ndata);
        ospSetObject(gdata, "vertex.normal", ndata);
        ospRelease(ndata);
      }

      // texture coordinate
      if (geo.has_texcoord) {
        OSPData tdata = ospNewData(geo.texcoord.size() / 2,
                                   OSP_VEC2F,
                                   geo.texcoord.data(),
                                   OSP_DATA_SHARED_BUFFER);
        ospCommit(tdata);
        ospSetObject(gdata, "vertex.texcoord", tdata);
        ospRelease(tdata);
      }

      // add material
      if (mtl != nullptr) {
        ospSetMaterial(gdata, mtl);
      }

      // commit geometry
      ospCommit(gdata);
      ospAddGeometry(world, gdata);
    }
  }
#endif
}
