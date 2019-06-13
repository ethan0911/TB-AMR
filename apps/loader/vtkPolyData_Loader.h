#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkGenericDataObjectReader.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataReader.h>

template <class TReader>
vtkDataSet *readVTKFile(const char* fileName)
{
  vtkSmartPointer<TReader> reader = vtkSmartPointer<TReader>::New();

  reader->SetFileName(fileName);
  reader->Update();

  reader->GetOutput()->Register(reader);
  return vtkDataSet::SafeDownCast(reader->GetOutput());
}