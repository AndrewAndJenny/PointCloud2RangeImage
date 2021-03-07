#ifndef RANGE_IMAGE_RUNABLE_H
#define RANGE_IMAGE_RUNABLE_H

#include"QtCore/QRunnable"
#include "QtCore/QThreadPool"

#include "DataStruct.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class RangeImagePlanar;

class RangeImageRunable:public QRunnable
{
public:
	RangeImageRunable(int imageid, std::string save_folder_path,const ImageLasInfo imgLasInfo, const BACameraIntrinsics intrinsic) :
		imageid_(imageid), save_folder_path_(save_folder_path), range_image_info_(imgLasInfo), camera_intrinsic_(intrinsic){ }

	~RangeImageRunable(){ }

	/**\brief generate single RangeImage*/
	void run() override;

protected:
	/**\brief load pointcloud from lasfiles according to range_image_correspondence*/
	bool loadPointCloudFromList(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

	/** \brief Calculate the image point and range from the given 3D point*/
	void getImagePoint(const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d);

	int imageid_;
	std::string save_folder_path_;

	ImageLasInfo range_image_info_;
	BACameraIntrinsics camera_intrinsic_;
};

#endif // RANGE_IMAGE_RUNABLE_H
