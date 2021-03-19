#ifndef RANGEIMAGETOOL_H
#define RANGEIMAGETOOL_H

#include "cmath"
#include "cfloat"
#include "auxiliary.h"
#include "Eigen/Dense"
#include "lasreader.hpp"
#include "BACameraFile.hpp"
#include "range_image_planar.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include<opencv2/core/eigen.hpp>
#define DEFAULT_MAX_ELEVATION 120

class RangeImageTool
{
public:
	// =====TYPEDEFS=====
	typedef std::shared_ptr<RangeImageTool> Ptr;
	typedef std::shared_ptr<const RangeImageTool> ConstPtr;

	// =====CONSTRUCTOR & DESTRUCTOR=====
	 /** Constructor */
	RangeImageTool();

	RangeImageTool(std::string cmrFile_path, std::string phtFile_path, std::string lasFolder_Path,std::string saveFolder_Path);
	/** Destructor */
	virtual ~RangeImageTool();

	// =====PUBLIC METHODS=====
	 /** \brief Get a shared pointer of a copy of this */
	inline Ptr makeShared() { return Ptr(new RangeImageTool(*this)); }

	/**\brief Set the max elevation of point cloud */
	inline void setMaxElevation(double elevation) { maxElevation = elevation; }

	/**\brief Set the absolute path of camera intrinsics file*/
	inline void setCmrPath(std::string cmrFile_path) { cmrPath = cmrFile_path; }

	/**\brief Set the windows radius for interpolating Dense range image*/
	inline void setGridNum(int ngrid) { grid = ngrid; }

	/**\brief Set the absolute path of camera extrinsic file*/
	inline void setPhtPath(std::string phtFile_path) { phtPath = phtFile_path; }

	/**\brief Set the absolute path of las files corresponding to the photos*/
	inline void setLasPath(std::string lasFolder_Path) {lasPath = lasFolder_Path;}

	/**\brief Set the absolute path of range image files */
	inline void setOutputFolder(std::string saveFolder_path) { saveFolder = saveFolder_path; }

	/**\brief load pointcloud from lasfiles according to range_image_correspondence
		*\param imgLasInfo the camera extrinsic
		*\param intrinsic the camera intrinsic
		*\param point_cloud_filter the pointer of cloud
		*/
	bool loadPointCloudFromList(const ImageLasInfo imgLasInfo, const BACameraIntrinsics intrinsic, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

	/**\brief generate  all RangeImage*/
	void generateRangeImage();

	/*\*brief generate single RangeImage
		*\param image_id the num of camera corresponding distance image
		*\param imgLasInfo include lasList and Rotate Orthogonal Matrix and Location
		*\param intrinsic is the imageid of camera intrinsic
		*/
	void generateSingleRangeImage(int image_id, ImageLasInfo imgLasInfo, BACameraIntrinsics intrinsic);

	/** \brief Calculate the image point and range from the given 3D point
		* \param intrinsic the camera intrinsic
		*\param imageinfo
		* \param point3d the 3D point
		* \param point2d the image point
		*/
	void getImagePoint(const BACameraIntrinsics& intrinsic, const ImageLasInfo& imageinfo, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d);

	/** \brief Calculate the dense range image from sparse range image using window weighted interpolation
		*\param difference_x a matrix that stores the distance between each pixel and the real point in the x direction
		*\param difference_y a matrix that stores the distance between each pixel and the real point in the y direction
		*\param sparse_range_image origin range image
		*\param dense_range_image ouput interpolation range image
		*/
	void getDenseRangeImage(Eigen::MatrixXf& difference_x, Eigen::MatrixXf& difference_y, Eigen::MatrixXf& sparse_range_image, Eigen::MatrixXf& dense_range_image);

protected:
	// =====PROTECT METHODS=====

	/**\brief Determine whether the point cloud corresponds to the image*/
	bool isHomologous(const BACameraIntrinsics& intrinsic, const ImageLasInfo& imageinfo, const BoundingBoxCorner3D& box);

	/**\brief According to the Camera intrinsics and extrinsic file,obtain the las file corresponding to imageid*/
	void getPointCloudIndex();

	/**\brief According to the Camera extrinsic, caculate transform matrix and f ,center_x,center_y
		*\param extrinsic the camera extrinsic
		* \param imageinfo record R and Translate
		*/
	void getRotationAndTranslate(const BACamera& extrinsic, ImageLasInfo& imageinfo);

	/**\brief Calculate the 3D coordinates of the dense distance image after interpolation, and store x, y, z as BGR three channels respectively
		*\param cofiguration_info 
		*\param range_image store range information
		*\param range_image_3Dposition result->CV_32FC3
		*/
	void caculateAll3DPoints(RangeImagePlanar::Ptr cofiguration_info, cv::Mat& range_image, cv::Mat& range_image_3Dposition);

	/**\brief export image after interpolating ,real 3-D coordination and 3-D coordination after interpolating
		*\param cofiguration_info
		*\param image_id the num of camera corresponding distance image
	*/
	bool exportInfo(RangeImagePlanar::Ptr cofiguration_info, int image_id);

	std::string cmrPath;
	std::string phtPath;
	std::string lasPath;
	std::string saveFolder;

	int grid;
	double maxElevation;

	std::map<int, ImageLasInfo> imageInfoMap;//The index between imageid and lasfile associated 
	BACameraFile::Ptr cameraInfo;
};

#endif //RANGEIMAGETOOL_H
