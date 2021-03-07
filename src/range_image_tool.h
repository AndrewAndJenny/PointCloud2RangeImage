#ifndef RANGEIMAGETOOL_H
#define RANGEIMAGETOOL_H

#include "chrono"
#include "cmath"
#include "cfloat"

#include "auxiliary.h"
#include "BACameraFile.hpp"

#include "DataStruct.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define SINGLE_THREAD 1

class RangeImageTool
{
public:
	// =====TYPEDEFS=====
	typedef std::shared_ptr<RangeImageTool> Ptr;
	typedef std::shared_ptr<const RangeImageTool> ConstPtr;

	// =====CONSTRUCTOR & DESTRUCTOR=====
	 /** Constructor */
	RangeImageTool();

	RangeImageTool(std::string cmrFile_path, std::string phtFile_path, std::string lasFolder_Path,std::string saveFolder_Path, int thread_num);
	/** Destructor */
	virtual ~RangeImageTool();

	// =====PUBLIC METHODS=====
	 /** \brief Get a boost shared pointer of a copy of this */
	inline Ptr makeShared() { return Ptr(new RangeImageTool(*this)); }

	/**\brief set the absolute path of camera intrinsics file*/
	inline void setCmrPath(std::string cmrFile_path) { cmrPath = cmrFile_path; }

	/**\brief set the absolute path of camera extrinsic file*/
	inline void setPhtPath(std::string phtFile_path) { phtPath = phtFile_path; }

	/**\brief set the absolute path of las files corresponding to the photos*/
	inline void setLasPath(std::string lasFolder_Path) {lasPath = lasFolder_Path;}

	/**\brief set the absolute path of range image files */
	inline void setOutputFolder(std::string saveFolder_path) { saveFolder = saveFolder_path; }

	/**\brief set the num of parall process thread*/
	inline void setThreadNum(int thread_num) { parall_thread_num_ = thread_num; }

	/**\brief generate  all RangeImage*/
	void generateRangeImage();

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

	int parall_thread_num_;

	std::map<int, ImageLasInfo> imageInfoMap;//The index between imageid and lasfile associated 

	BACameraFile::Ptr cameraInfo;

};

#endif //RANGEIMAGETOOL_H
