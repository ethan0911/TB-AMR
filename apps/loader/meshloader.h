#pragma once
#ifndef _MESH_LOADER_H_
#define _MESH_LOADER_H_

#include "ospcommon/AffineSpace.h"
#include "ospcommon/FileName.h"
#include "ospcommon/LinearSpace.h"
#include "ospcommon/box.h"
#include "ospcommon/range.h"
#include "ospcommon/tasking/parallel_for.h"
#include "ospcommon/vec.h"
#include "ospray/ospray.h"

// trying this obj loader https://github.com/syoyo/tinyobjloader
#include "tiny_obj_loader.h"

#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>

using namespace ospcommon;

template <class TReader>
vtkDataSet *readVTKFile(const FileName &fileName)
{
  vtkSmartPointer<TReader> reader = vtkSmartPointer<TReader>::New();

  reader->SetFileName(fileName.c_str());
  reader->Update();

  // return reader->GetOutput();

  reader->GetOutput()->Register(reader);
  return vtkDataSet::SafeDownCast(reader->GetOutput());
}

/** \brief structure for a triangular mesh */
class Mesh
{
 public:
  //! one geometry contains a continious mesh plus one material index
  struct Geometry
  {
    std::vector<float> vertex;
    std::vector<float> normal;
    std::vector<float> texcoord;
    std::vector<unsigned int> index;
    int num_faces     = 0;
    bool has_normal   = false;
    bool has_texcoord = false;
  };
  struct TinyObjLoader
  {
    std::string err;
    tinyobj::attrib_t attributes;                // attributes
    std::vector<tinyobj::shape_t> shapes;        // shapes
    std::vector<tinyobj::material_t> materials;  // materials
    void Clear();
  };
  TinyObjLoader tiny;
  /* geometric data */
  vec3f center;  // mesh center coordinate in world
  box3f bbox;    // mesh bounding box in world
  affine3f transform;
  /* meta data */
  std::string dpath;  // directory path to the mesh folder
  std::string fpath;  // directory path to the mesh folder
  std::string fname;  // filename of the mesh
  std::vector<Geometry> geometries;

 private:
  void ComputePath(const std::string &str);
  void LoadFromVTKFile(const FileName &fileName, Geometry &geo);

 public:
  /** \brief Accessors */
  std::string GetFullPath()
  {
    return fpath;
  }
  vec3f GetBBoxMax()
  {
    return bbox.upper - center;
  }
  vec3f GetBBoxMin()
  {
    return bbox.lower - center;
  }
  vec3f GetCenter()
  {
    return center;
  }
  float GetDiagonalLength()
  {
    return length(GetBBoxMax() - GetBBoxMin());
  }
  affine3f GetTransform()
  {
    return transform;
  }
  void SetTransform(const affine3f &);
  /**
   * \brief Overriding LoadFromFileObj function for TriMesh,
   *  force to triangulate
   */
  bool LoadFromFileObj(const char *filename, bool loadMtl = false);

  /**
   * \brief load .vtp file
   */
  void LoadMesh(std::vector<std::string> inputMesh);
  /**
   * \brief OSPRay helper
   */
  void AddToModel(OSPWorld world, OSPMaterial mtl = nullptr);
};

#endif  //_MESH_LOADER_H_
