/*
 *  David Z 26. Oct. 2014
 *  
 *  global variables, functions
 *
 * */
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <algorithm>
#include <climits>

template<typename PointT>
void extractNormal(typename pcl::PointCloud<PointT>::Ptr& in, 
    pcl::PointCloud<pcl::Normal>::Ptr& out, double radiusSearch)
{
  typename pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
  // pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (in);
  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  normal_estimator.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
  // normal_estimator.setKSearch (50);
  normal_estimator.setRadiusSearch(radiusSearch);
  normal_estimator.compute (*out);
}

template<typename PointT>
double square_dis_point(PointT& p1, PointT& p2)
{
  return (SQ(p1.x-p2.x) + SQ(p1.y-p2.y) + SQ(p1.z - p2.z));
}

template<typename T>
bool compute_mu_sigma(T* in, int N, T& mu, T& sigma)
{
  if(N<=0)
  {
    mu = 0; 
    sigma = 0;
    return false;
  }

  // compute mu
  T total = 0; 
  for(int i=0; i<N; i++)
  {
    total += in[i];
  }
  mu = total/(T)(N); 
  
  // compute sigma
  total = 0;
  for(int i=0; i<N; i++)
  {
      total += SQ(in[i]-mu);
  }
  sigma = sqrt(total/(T)N);
  return true;
}

template<typename PointT>
PointT calculateCentroids(typename pcl::PointCloud<PointT>::Ptr& in, 
                        pcl::PointIndices::Ptr& index)
{
  vector<int>& indices = index->indices;
  PointT ret_pt;
  ret_pt.x = ret_pt.y = ret_pt.z = 0; 
  for(int i=0; i< indices.size(); i++)
  {
    PointT& pt = in->points[indices[i]]; 
    ret_pt.x += pt.x; 
    ret_pt.y += pt.y; 
    ret_pt.z += pt.z; 
  }
  float fn = indices.size();
  ret_pt.x /= fn; 
  ret_pt.y /= fn;
  ret_pt.z /= fn;
  return ret_pt;
}

template<typename PointT>
PointT calculateCentroids(typename pcl::PointCloud<PointT>::Ptr& in)
{
  pcl::PointIndices::Ptr index(new pcl::PointIndices); 
  index->indices.resize(in->size());
  for(int i=0; i<in->size(); i++)
    index->indices[i] = i; 
  return calculateCentroids<PointT>(in, index);
}

template<typename PointT>
void decomposePTwithNormals(pcl::PointCloud<pcl::PointNormal>::Ptr& in,
                          typename pcl::PointCloud<PointT>::Ptr& out, 
                          pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
  out->points.clear(); 
  normals->points.clear();
  for(int i=0; i<in->size(); i++)
  {
    pcl::PointNormal& pt = in->points[i];
    PointT d_pt; 
    pcl::Normal normal; 
    d_pt.x = pt.x; 
    d_pt.y = pt.y; 
    d_pt.z = pt.z; 
    normal.normal_x = pt.normal_x;
    normal.normal_y = pt.normal_y; 
    normal.normal_z = pt.normal_z;
    out->points.push_back(d_pt);
    normals->points.push_back(normal);
  }
  return ;
}


template<typename PointT>
void combinePTwithNormals(typename pcl::PointCloud<PointT>::Ptr& in, 
                          pcl::PointCloud<pcl::Normal>::Ptr& normals, 
                          pcl::PointCloud<pcl::PointNormal>::Ptr& out)
{
  assert(in->size() == normals->size()); 
  out->points.clear();
  for(int i=0; i<in->size(); i++)
  {
    PointT& pt = in->points[i]; 
    pcl::Normal& normal = normals->points[i]; 
    pcl::PointNormal dpt; 
    dpt.x = pt.x; 
    dpt.y = pt.y; 
    dpt.z = pt.z; 
    dpt.data[3] = 1.;
    dpt.normal_x = normal.normal_x; 
    dpt.normal_y = normal.normal_y; 
    dpt.normal_z = normal.normal_z; 
    out->points.push_back(dpt);
  }
  return ;
}

template<typename PointT>
void extractCloud(typename pcl::PointCloud<PointT>::Ptr& in, 
                typename pcl::PointCloud<PointT>::Ptr& out,
                pcl::PointIndices::Ptr& index, bool inv = false)
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(in);
  extract.setIndices(index);
  extract.setNegative(inv);
  extract.filter(*out);
}

template<typename PointT>
void g_passFilter(typename pcl::PointCloud<PointT>::ConstPtr& in,typename pcl::PointCloud<PointT>::Ptr& out, float lower, float upper, string field="z")
{
   // passthrough filter
    pcl::PassThrough<PointT > pass;
    pass.setInputCloud(in);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(lower, upper);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, _depth_limit);
    pass.filter(*out);
}

template<typename PointT>
void g_passFilter(typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out, float lower, float upper, string field="z")
{
   // passthrough filter
    pcl::PassThrough<PointT > pass;
    pass.setInputCloud(in);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(lower, upper);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.0, _depth_limit);
    pass.filter(*out);
}

template<typename PointT>
bool removeOutlier(boost::shared_ptr< pcl::PointCloud<PointT> >& in, boost::shared_ptr< pcl::PointCloud<PointT> >& out)
{
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<PointT> sor; 
  sor.setInputCloud(in); 
  sor.setMeanK (10); 
  sor.setStddevMulThresh(1.0);
  sor.filter(*out);
  return true;
}

template<typename PointT>
bool calculateBoundary(boost::shared_ptr< pcl::PointCloud<PointT> >& in, pcl::PointIndices::Ptr& inliers, float& minX, float& maxX, float& minZ, float& maxZ, float& minY, float& maxY )
{
    minY = minX = minZ = std::numeric_limits<float>::max();
    maxY = maxX = maxZ = -std::numeric_limits<float>::max();
   
    for(int i=0; i< inliers->indices.size(); i++)
    {
        PointT& pt = in->points[inliers->indices[i]];
        if(!pcl_isfinite(pt.x) || !pcl_isfinite(pt.y) || !pcl_isfinite(pt.z))
            continue;
        if(pt.x > maxX) 
        {
            maxX = pt.x; 
        }
        if(pt.x < minX)
        {
            minX = pt.x; 
        }
        if(pt.z > maxZ)
        {
            maxZ = pt.z;
        }
        if(pt.z < minZ)
        {
            minZ = pt.z;
        }      
        if(pt.y > maxY)
        {
            maxY = pt.y;
        }
        if(pt.y < minY)
        {
            minY = pt.y;
        }
    }
    // minZ += g_Z_expanded; // expand the area along Z axis 
}

template<typename PointT>
bool calculateBoundary(boost::shared_ptr<pcl::PointCloud<PointT> >& in, float& minX, float& maxX, float& minZ, float& maxZ, float& minY, float& maxY)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    inliers->indices.resize(in->points.size());
    for(int i=0; i<in->points.size(); i++)
    {
        inliers->indices[i] = i;
    }
    return calculateBoundary(in, inliers, minX, maxX, minZ, maxZ, minY, maxY);
}

template<typename T>
void medianFilterCol(vector<T>& array, int Width, int Height) // row first rank
{
  vector<T> tmp;
  vector<T> arrayCopy(array);
  for(int i=0; i<Width; i++)
  {
    for(int j=0; j<Height; j++)
    {
      tmp.clear();
      if(i-1 >= 0) // left 
      {
          tmp.push_back(arrayCopy[(i-1)*Height + j]);
          if(j-1 >=0) 
            tmp.push_back(arrayCopy[(i-1)*Height+j-1]);
          if(j+1 < Height)
            tmp.push_back(arrayCopy[(i-1)*Height+j+1]);
      }
      if(j-1 >= 0)
        tmp.push_back(arrayCopy[i*Height+j-1]);
      if(j+1 < Height)
        tmp.push_back(arrayCopy[i*Height+j+1]);
      tmp.push_back(arrayCopy[i*Height+j]);
      if(i+1 < Width) // right
      {
          tmp.push_back(arrayCopy[(i+1)*Height+j]);
          if(j-1>=0)
            tmp.push_back(arrayCopy[(i+1)*Height+j-1]);
          if(j+1<Height)
            tmp.push_back(arrayCopy[(i+1)*Height+j+1]);
      }
      // sort 
      sort(tmp.begin(), tmp.end());
      if(tmp.size()%2 == 0)
      {
        array[i*Height+j] = (tmp[tmp.size()/2-1]+tmp[tmp.size()/2])/2.;
      }else
      {
        array[i*Height+j] = tmp[tmp.size()/2];
      }
    }
  }
}



template<typename T>
void medianFilterRow(vector<T>& array, int Width, int Height) // row first rank
{
  vector<T> tmp;
  vector<T> arrayCopy(array);
  for(int i=0; i<Height; i++)
  {
    for(int j=0; j<Width; j++)
    {
      tmp.clear();
      if(i-1 >= 0) // upper 
      {
          tmp.push_back(arrayCopy[(i-1)*Width + j]);
          if(j-1 >=0) 
            tmp.push_back(arrayCopy[(i-1)*Width+j-1]);
          if(j+1 < Width)
            tmp.push_back(arrayCopy[(i-1)*Width+j+1]);
      }
      if(j-1 >= 0)
        tmp.push_back(arrayCopy[i*Width+j-1]);
      if(j+1 < Width)
        tmp.push_back(arrayCopy[i*Width+j+1]);
      tmp.push_back(arrayCopy[i*Width+j]);
      if(i+1 < Height)
      {
          tmp.push_back(arrayCopy[(i+1)*Width+j]);
          if(j-1>=0)
            tmp.push_back(arrayCopy[(i+1)*Width+j-1]);
          if(j+1<Width)
            tmp.push_back(arrayCopy[(i+1)*Width+j+1]);
      }
      // sort 
      sort(tmp.begin(), tmp.end());
      if(tmp.size()%2 == 0)
      {
        array[i*Width+j] = (tmp[tmp.size()/2-1]+tmp[tmp.size()/2])/2.;
      }else
      {
        array[i*Width+j] = tmp[tmp.size()/2];
      }
    }
  }
}

