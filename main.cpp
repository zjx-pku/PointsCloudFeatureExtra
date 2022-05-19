#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_plotter.h>

#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/grsd.h>


/// <summary>
/// 提取并绘制给定点云文件的VFH特征
/// </summary>
/// <param name="pcdPath">点云文件路径</param>
int VFH(char* pcdPath){

	// Cloud for storing the object.
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the VFH descriptor.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	// Note: you should have performed preprocessing to cluster out the object
	// from the cloud, and save it to this individual file.

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *object) != 0)
	{
		return -1;
	}

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(object);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// VFH estimation object.
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(object);
	vfh.setInputNormals(normals);
	vfh.setSearchMethod(kdtree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	vfh.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	vfh.setNormalizeDistance(false);

	vfh.compute(*descriptor);

	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 308);

	plotter.plot();
}


/// <summary>
/// 提取并绘制给定点云文件的ESF特征
/// </summary>
/// <param name="pcdPath">点云文件路径</param>
int ESF(char* pcdPath)
{
	// Cloud for storing the object.
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the ESF descriptor.
	pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

	// Note: you should have performed preprocessing to cluster out the object
	// from the cloud, and save it to this individual file.

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *object) != 0)
	{
		return -1;
	}

	// ESF estimation object.
	pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	esf.setInputCloud(object);

	esf.compute(*descriptor);

	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 640);

	plotter.plot();
}

/// <summary>
/// 提取并绘制给定点云文件的GRSD特征
/// </summary>
/// <param name="pcdPath">点云文件路径</param>
int GRSD(char* pcdPath)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the GRSD descriptors for each point.
	pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors(new pcl::PointCloud<pcl::GRSDSignature21>());

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *cloud) != 0)
	{
		return -1;
	}

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// GRSD estimation object.
	pcl::GRSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::GRSDSignature21> grsd;
	grsd.setInputCloud(cloud);
	grsd.setInputNormals(normals);
	grsd.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	grsd.setRadiusSearch(0.05);

	grsd.compute(*descriptors);

	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptors, 21);

	plotter.plot();
}


// Main函数，采用命令行参数调用，包含两个参数：pcd文件路径和所提取特征的名称(VFH, ESF, GRSD)
// 示例：main.exe pcd\test.pcd VFH
int main(int argc, char** argv){
	// 从命令行中获取pcd文件路径和处理方法
	char* pcdPath = argv[1];
	char* method = argv[2];
	
	// 根据传入参数采取相关特征提取方法处理点云数据
	if (method[0] == 'V') {
		std::cout << "正在计算点云文件" << pcdPath << "的VFH特征..." << std::endl;
		VFH(pcdPath);
		std::cout << "计算完成！" << std::endl;
	}
	else if (method[0] == 'E') {
		std::cout << "正在计算点云文件" << pcdPath << "的ESF特征..." << std::endl;
		ESF(pcdPath);
		std::cout << "计算完成！" << std::endl;
	}
	else if (method[0] == 'G') {
		std::cout << "正在计算点云文件" << pcdPath << "的GRSD特征..." << std::endl;
		GRSD(pcdPath);
		std::cout << "计算完成！" << std::endl;
	}
	else {
		std::cout << "参数设置有误！" << std::endl;
	}

	return 0;
}