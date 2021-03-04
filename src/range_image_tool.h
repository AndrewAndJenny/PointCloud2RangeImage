#ifndef RANGEIMAGETOOL_H
#define RANGEIMAGETOOL_H

#include "cmath"
#include "cfloat"
#include "auxiliary.h"
#include "Eigen/Geometry"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lasreader.hpp"
#include "BACameraFile.hpp"

#include <gdal.h>
#include <gdal_priv.h>

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
	 /** \brief Get a boost shared pointer of a copy of this */
	inline Ptr makeShared() { return Ptr(new RangeImageTool(*this)); }

	/**\brief Set the absolute path of camera intrinsics file*/
	inline void setCmrPath(std::string cmrFile_path) { cmrPath = cmrFile_path; }

	/**\brief Set the absolute path of camera extrinsic file*/
	inline void setPhtPath(std::string phtFile_path) { phtPath = phtFile_path; }

	/**\brief Set the absolute path of las files corresponding to the photos*/
	inline void setLasPath(std::string lasFolder_Path) {lasPath = lasFolder_Path;}

	inline void setOutputFolder(std::string saveFolder_path) { saveFolder = saveFolder_path; }
	/**\brief load pointcloud from lasfiles according to range_image_correspondence
		*\param lasList lasfile absolute path
		*\param point_cloud store point cloud
		*\param box the boundingbox of point_cloud
		*/
	bool loadPointCloudFromList(const std::vector<std::string> lasList, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, BoundingBoxCorner3D& box);

	/**\brief generate  all RangeImage*/
	void generateRangeImage();

	/*\*brief generate single RangeImage
		*\param imageid the num of camera corresponding distance image
		*\param imgLasInfo include lasList and sensor_pose(to_world_system)
		*\param intrinsic is the imageid of camera intrinsic
		*/
	void generateSingleRangeImage(int imageid, const ImageLasInfo& imgLasInfo, const BACameraIntrinsics& intrinsic);

	/** \brief Calculate the image point and range from the given 3D point
		* \param intrinsic the camera intrinsic
		*\param imageinfo
		* \param point3d the 3D point
		* \param point2d the image point
		*/
	void getImagePoint(const BACameraIntrinsics& intrinsic, const ImageLasInfo& imageinfo, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d);

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

	std::string cmrPath;
	std::string phtPath;
	std::string lasPath;
	std::string saveFolder;

	std::map<int, ImageLasInfo> imageInfoMap;//The index between imageid and lasfile associated 

	BACameraFile::Ptr cameraInfo;

};

#endif //RANGEIMAGETOOL_H
