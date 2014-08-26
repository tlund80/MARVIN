
#include <one_shot_learning/ModelData.hpp>

namespace dti{
namespace one_shot_learning {

ModelData::ModelData(int index) : _index(index)
{
  _has_grasp_table = false;
  _has_grasp_table_GT = false;
  _has_GTMeshModel = false;
  _has_GTCloudModel = false;

}

ModelData::~ModelData()
{

}

}
}