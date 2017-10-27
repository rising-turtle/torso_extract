/*
 *  David Z
 *  global variables, functions
 *
 * */

#include "global_def.h"

int g_color[][3] = {255, 0, 0,  //RED
                            0, 255, 0, // GREEN
                            0, 0, 255,  // BLUE
                            255, 0, 255, // PURPLE
                            255, 255,255, // WHITE
                            255, 255, 0 // YELLOW
                        };
/*
bool markColor(color_point_cloud& cloud, int index)
{
    int C = sizeof(g_color)/(3*sizeof(int));
    index = index%C; 
    if(cloud.points.size() <= 0)
    {
      // cout<<"global_def.cpp: cloud has no points!"<<endl;
      return false;
    }
    int N = cloud.points.size();
    for(int i=0; i<N; i++)
    {
      cloud.points[i].r = g_color[index][0];
      cloud.points[i].g = g_color[index][1];
      cloud.points[i].b = g_color[index][2];
    }
    return true;
}

bool markColor(color_point_cloud& cloud, COLOR c)
{
  if(cloud.points.size() <= 0)
  {
    // cout<<"global_def.cpp: cloud has no points!"<<endl;
    return false;
  }
  int N = cloud.points.size();
  for(int i=0; i<N; i++)
  {
    cloud.points[i].r = g_color[c][0];
    cloud.points[i].g = g_color[c][1];
    cloud.points[i].b = g_color[c][2];
  }
  return true;
}

bool markColor(color_point_cloud& cloud, pcl::PointIndices::Ptr& index, COLOR c)
{
  if(index->indices.size() <=0 )
  {
    cout<<"global_def.cpp: index is empty!"<<endl;
    return false;
  }
  if(cloud.points.size() <= 0)
  {
    // cout<<"global_def.cpp: cloud has no points!"<<endl;
    return false;
  }
  int N = cloud.points.size();
  for(int i=0; i<index->indices.size(); i++)
  {
    int j = index->indices[i];

    if(j >= N)
    {
      cerr<<"global_def.cpp: index "<<j<<" exceeds cloud size: "<<N<<endl;
      return false;
    }
    cloud.points[j].r = g_color[c][0];
    cloud.points[j].g = g_color[c][1];
    cloud.points[j].b = g_color[c][2];
  }
  return true;
}
*/
const unsigned char* translate(uint8_t* p8, uint16_t* p16, int N)
{
  const unsigned char* pret = p8;
  for(int i=0; i<N; i++)
  {
    *p8++ = *p16++;
  }
  return pret;
}

const unsigned char* translate(vector<uint8_t>& v8, vector<uint16_t>& v16)
{
  assert(v8.size() == v16.size());
  int N = v8.size();
  const unsigned char* pret = v8.data();
  for(int i=0; i<N; i++)
  {
    v8[i] = v16[i];
  }
  return pret;
}

const unsigned char* translateScale(vector<uint8_t>& v8, vector<uint16_t>& v16)
{
  assert(v8.size() == v16.size());
  int N = v8.size();
  static const uint8_t MAX8 = 255;
  const unsigned char* pret = v8.data();
  
  uint16_t maxV = 0; //std::numeric_limits<uint16_t>::min();
  // note shadow_value_ and no_sample_value_ are all zero in OpenNI 2.x, different from OpenNI 1.x
  for(int i=0; i<N; i++)
  {
    if(v16[i] > maxV) maxV = v16[i];
  }
  // cout<<"record_video.cpp: maxV = "<<maxV<<endl;

  for(int i=0; i<N; i++)
  {
    if(maxV == 0) v8[i] = 0;
    else v8[i] = v16[i]*MAX8/maxV;
  }
  return pret;
}

bool computeEigenVector2D(Eigen::Matrix2f& covariance_matrix, Eigen::Vector2f& major_axis, Eigen::Vector2f& minor_axis, 
                            float& major_value, float& minor_value)
{
  Eigen::EigenSolver <Eigen::Matrix <float, 2, 2> > eigen_solver; 
  eigen_solver.compute(covariance_matrix); 
  
  Eigen::EigenSolver <Eigen::Matrix <float, 2, 2> >::EigenvectorsType eigen_vectors;
  Eigen::EigenSolver <Eigen::Matrix <float, 2, 2> >::EigenvalueType eigen_values;
  eigen_vectors = eigen_solver.eigenvectors ();
  eigen_values = eigen_solver.eigenvalues ();

  unsigned int temp = 0;
  unsigned int major_index = 0;
  unsigned int minor_index = 1;

  if (eigen_values.real () (major_index) < eigen_values.real () (minor_index))
  {
    temp = major_index;
    major_index = minor_index;
    minor_index = temp;
  }
 
  major_value = eigen_values.real () (major_index);
  minor_value = eigen_values.real () (minor_index);

  major_axis = eigen_vectors.col (major_index).real ();
  minor_axis = eigen_vectors.col (minor_index).real ();

  major_axis.normalize ();
  minor_axis.normalize ();

  // float det = major_axis.cross (minor_axis);
  float det = major_axis(0)*minor_axis(1) - major_axis(1)*minor_axis(0);
  if (det <= 0.0f)
  {
    major_axis (0) = -major_axis (0);
    major_axis (1) = -major_axis (1);
  }
  return true;
}

bool computeEigenVector(Eigen::Matrix3f& covariance_matrix, Eigen::Vector3f& major_axis, Eigen::Vector3f& middle_axis,
                              Eigen::Vector3f& minor_axis, float& major_value, float& middle_value, float& minor_value)
{
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> > eigen_solver;
  eigen_solver.compute (covariance_matrix);

  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvectorsType eigen_vectors;
  Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvalueType eigen_values;
  eigen_vectors = eigen_solver.eigenvectors ();
  eigen_values = eigen_solver.eigenvalues ();

  unsigned int temp = 0;
  unsigned int major_index = 0;
  unsigned int middle_index = 1;
  unsigned int minor_index = 2;

  if (eigen_values.real () (major_index) < eigen_values.real () (middle_index))
  {
    temp = major_index;
    major_index = middle_index;
    middle_index = temp;
  }

  if (eigen_values.real () (major_index) < eigen_values.real () (minor_index))
  {
    temp = major_index;
    major_index = minor_index;
    minor_index = temp;
  }

  if (eigen_values.real () (middle_index) < eigen_values.real () (minor_index))
  {
    temp = minor_index;
    minor_index = middle_index;
    middle_index = temp;
  }

  major_value = eigen_values.real () (major_index);
  middle_value = eigen_values.real () (middle_index);
  minor_value = eigen_values.real () (minor_index);

  major_axis = eigen_vectors.col (major_index).real ();
  middle_axis = eigen_vectors.col (middle_index).real ();
  minor_axis = eigen_vectors.col (minor_index).real ();

  major_axis.normalize ();
  middle_axis.normalize ();
  minor_axis.normalize ();

  float det = major_axis.dot (middle_axis.cross (minor_axis));
  if (det <= 0.0f)
  {
    major_axis (0) = -major_axis (0);
    major_axis (1) = -major_axis (1);
    major_axis (2) = -major_axis (2);
  }
  return true;
} 
