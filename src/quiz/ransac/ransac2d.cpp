/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
/** 
 * @brief 	My answer
 * 		  	Point2Line
 * @author  Kucukcolllu
*/
// std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
// {
// 	std::unordered_set<int> inliersResult;
// 	srand(time(NULL));

// 	/////////////////////////////////////////////////////////////////////
// 	// TODO: Fill in this function

// 	for(size_t i=0; i<maxIterations; ++i)
// 	{
// 		std::unordered_set<int> local_inliers;
// 		std::cout << "-- For iteration [" << i << "] --" << std::endl;
		
// 		// select random 2 point
// 		auto point1_index = rand() % cloud->points.size();
// 		auto point2_index = rand() % cloud->points.size();

// 		while(point1_index == point2_index)
// 		{
// 			point2_index == rand() % cloud->points.size();
// 		}

// 		std::cout << "Point-1 index : " << point1_index << std::endl;
// 		std::cout << "Point-2 index : " << point2_index << std::endl;

// 		// x and y of point 1
// 		auto x1 = cloud->points.at(point1_index).x;
// 		auto y1 = cloud->points.at(point1_index).y;
		
// 		// x and y of point 2
// 		auto x2 = cloud->points.at(point2_index).x;
// 		auto y2 = cloud->points.at(point2_index).y;
	
// 		auto A = y1 - y2;
// 		auto B = x2 - x1;
// 		auto C = x1 * y2 - x2 * y1;
		
// 		for(size_t index=0; index<cloud->size(); index++)
// 		{
// 			if(index != point1_index && index != point2_index)
// 			{
// 				std::cout << "Current point index : " << index << std::endl;

// 				// current point
// 				auto point_x = cloud->points.at(index).x;
// 				auto point_y = cloud->points.at(index).y;

// 				auto distance = std::abs(A*point_x + B*point_y + C) / std::sqrt(std::pow(A,2)+std::pow(B,2));

// 				if(distance <= distanceTol)
// 				{
// 					local_inliers.insert(index);
// 				}
// 			}
// 		}

// 		if(local_inliers.size() > inliersResult.size())
// 		{
// 			inliersResult = local_inliers;
// 		}
// 	}
	
// 	/////////////////////////////////////////////////////////////////////

// 	return inliersResult;

// }


/**
 * @brief Course solution
 * 		  Point2Line
 */
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	/////////////////////////////////////////////////////////////////////
	// TODO: Fill in this function

	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 2)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		float x1, y1, x2, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = (y1 - y2);
		float b = (x2 - x1);
		float c = (x1*y2 - x2*y1);

		for(int index=0; index < cloud->points.size(); index++)
		{
			if(inliers.count(index) > 0)
				continue;
			
			pcl::PointXYZ point = cloud->points[index];

			float x3 = point.x;
			float y3 = point.y;

			float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);

			if(d <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		if(inliers.size()>inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	/////////////////////////////////////////////////////////////////////

	return inliersResult;
}

/**
 * @brief 	Point2Plane
 * 			Ax + By + Cz + D = 0      plane eq.
 * @author 	Kucukcolllu
 */
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliers_result;
	srand(time(NULL));

	/////////////////////////////////////////////////////////////////////

	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}

		/** @todo define a struct which handles a #D point and their calculations
		 * 		  or just use Eigen3 for cross product and vector operations
		 */
		double x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		// point1
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		*itr++;
		// point2
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		*itr++;
		// point3
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// calculate v1 (vector from point1 to point2)
		auto v1_x = x2 - x1;
		auto v1_y = y2 - y1;
		auto v1_z = x2 - y1;

		// calculate v2 (vector from point1 to point3)
		auto v2_x = x3 - x1;
		auto v2_y = y3 - y1;
		auto v2_z = z3 - z1;

		// calculate v1 x v2 (cross pruduct)
		auto cross_x = (y2-y1)*(z3-z1)-(z2*z1)*(y3-y1); // i term, A
		auto cross_y = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1); // j term, B
		auto cross_z = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1); // k term, C

		auto D = -(cross_x * x1 + cross_y * y1 + cross_z * z1);

		for(size_t index=0; index<cloud->points.size(); index++)
		{
			if(inliers.count(index) > 0)
			{
				continue;
			}

			auto point = cloud->points.at(index);

			auto d = std::abs(cross_x * point.x + cross_y * point.y + cross_z * point.z + D) / std::sqrt( std::pow(cross_x, 2) + std::pow(cross_y, 2) + std::pow(cross_z, 2) );

			if(d <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		if(inliers.size() > inliers_result.size())
		{
			inliers_result = inliers;
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "3D plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliers_result;

	/////////////////////////////////////////////////////////////////////
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
