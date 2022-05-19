#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/pcl_plotter.h>

#include <pcl/features/vfh.h>
#include <pcl/features/esf.h>
#include <pcl/features/grsd.h>


/// <summary>
/// ��ȡ�����Ƹ��������ļ���VFH����
/// </summary>
/// <param name="pcdPath">�����ļ�·��</param>
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
/// ��ȡ�����Ƹ��������ļ���ESF����
/// </summary>
/// <param name="pcdPath">�����ļ�·��</param>
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
/// ��ȡ�����Ƹ��������ļ���GRSD����
/// </summary>
/// <param name="pcdPath">�����ļ�·��</param>
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


// Main���������������в������ã���������������pcd�ļ�·��������ȡ����������(VFH, ESF, GRSD)
// ʾ����main.exe pcd\test.pcd VFH
int main(int argc, char** argv){
	// ���������л�ȡpcd�ļ�·���ʹ�����
	char* pcdPath = argv[1];
	char* method = argv[2];
	
	// ���ݴ��������ȡ���������ȡ���������������
	if (method[0] == 'V') {
		std::cout << "���ڼ�������ļ�" << pcdPath << "��VFH����..." << std::endl;
		VFH(pcdPath);
		std::cout << "������ɣ�" << std::endl;
	}
	else if (method[0] == 'E') {
		std::cout << "���ڼ�������ļ�" << pcdPath << "��ESF����..." << std::endl;
		ESF(pcdPath);
		std::cout << "������ɣ�" << std::endl;
	}
	else if (method[0] == 'G') {
		std::cout << "���ڼ�������ļ�" << pcdPath << "��GRSD����..." << std::endl;
		GRSD(pcdPath);
		std::cout << "������ɣ�" << std::endl;
	}
	else {
		std::cout << "������������" << std::endl;
	}

	return 0;
}