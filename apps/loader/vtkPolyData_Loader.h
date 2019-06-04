#include <vtkSmartPointer.h>
#include <vtkCell.h>
#include <vtkCellData.h>
#include <vtkCellTypes.h>
#include <vtkDataSet.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtksys/SystemTools.hxx>


template <class TReader>
vtkDataSet *readVTKFile(const FileName &fileName)
{
  vtkSmartPointer<TReader> reader = vtkSmartPointer<TReader>::New();

  reader->SetFileName(fileName.c_str());
  reader->Update();

  reader->GetOutput()->Register(reader);
  return vtkDataSet::SafeDownCast(reader->GetOutput());
}

template <class TReader>
void loadVTKFile(const FileName &fileName)
{
  vtkDataSet *dataSet = readVTKFile<TReader>(fileName);

  int numberOfCells  = dataSet->GetNumberOfCells();
  int numberOfPoints = dataSet->GetNumberOfPoints();

  double point[3];
  for (int i = 0; i < numberOfPoints; i++) {
    dataSet->GetPoint(i, point);
    vertices->push_back(vec3f(point[0], point[1], point[2]));
  }

  for (int i = 0; i < numberOfCells; i++) {
    vtkCell *cell = dataSet->GetCell(i);

    if (cell->GetCellType() == VTK_TRIANGLE) {
      indices->push_back(
          vec3i(cell->GetPointId(0), cell->GetPointId(1), cell->GetPointId(2)));
    }
  }
}

// void loadFile(const FileName &fileName)
// {
//   std::string extension = fileName.ext();

//   if (extension == "vtp")
//     loadVTKFile<vtkXMLPolyDataReader>(fileName.c_str());
// }