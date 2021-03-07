#include "range_image_tool.h"

#include "lasreader.hpp"
#include "Eigen/Core"
#include <gdal.h>
#include <gdal_priv.h>
#include "range_image_runable.h"

RangeImageTool::RangeImageTool()
{
	cameraInfo = std::make_shared<BACameraFile>();
}

RangeImageTool::RangeImageTool(std::string cmrFile_path, std::string phtFile_path, std::string lasFolder_Path,std::string saveFolder_Path,int thread_num) :
	cmrPath(cmrFile_path), phtPath(phtFile_path), lasPath(lasFolder_Path),saveFolder(saveFolder_Path), parall_thread_num_(thread_num)
{
	cameraInfo = std::make_shared<BACameraFile>();
}

RangeImageTool::~RangeImageTool()
{
	using ITER = std::map<int, ImageLasInfo>::iterator;
	if (imageInfoMap.size() != 0)
		for (ITER iter = imageInfoMap.begin(); iter != imageInfoMap.end();)
			imageInfoMap.erase(iter++);
}

// =====PUBLIC METHODS=====
/////////////////////////////////////////////////////////////////////////
void RangeImageTool::generateRangeImage()
{
	if (parall_thread_num_ <= 0)
	{
		std::cerr << "The number of thread must be more than one!\n";
		exit(1);
	}
	getPointCloudIndex();
	std::cout << "Calculate the mapping relationship between point cloud and image successfully !\n";
	int imageid, intrinsics_group;

	QThreadPool pool;
	pool.setMaxThreadCount(parall_thread_num_);

	auto start_time = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < cameraInfo->m_cameras.size(); i++)
	{
		imageid = cameraInfo->m_cameras[i].imageid;
		intrinsics_group = cameraInfo->m_cameras[i].intrinsics_group;

		ImageLasInfo imgLasInfo = imageInfoMap[imageid];
		BACameraIntrinsics& intrinsic = cameraInfo->m_intrinsics[intrinsics_group];

		if (!imgLasInfo.las_list.empty())
		{
			RangeImageRunable* sub_task = new RangeImageRunable(imageid, saveFolder, imgLasInfo, intrinsic);
			pool.start(sub_task);
		}

	}
	pool.waitForDone(-1);

	auto end_time = std::chrono::high_resolution_clock::now();
	auto cost_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
	std::cout << "Task Finished! the cost of time is " << cost_time.count() << "s" << std::endl;
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::getImagePoint(const BACameraIntrinsics& intrinsic, const ImageLasInfo& imageinfo, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d)
{
	double format_x_mm = intrinsic.format_x;
	double format_y_mm = intrinsic.format_y;
	double x0_mm = intrinsic.intrins(OFFSET_X0);
	double y0_mm = intrinsic.intrins(OFFSET_Y0);
	double f0_mm = intrinsic.intrins(OFFSET_F0);

	Eigen::Matrix3d Rotate_ = imageinfo.rotate_;
	Eigen::Vector3d Location_ = imageinfo.translate_;

	Rotate_.transposeInPlace();

	Eigen::Vector3d transformedPoint = Rotate_ * (point3d - Location_);

	double x_mm = -f0_mm * transformedPoint[0] / transformedPoint[2];
	double y_mm = -f0_mm * transformedPoint[1] / transformedPoint[2];

	point2d[0] = (x_mm + x0_mm + format_x_mm * 0.5) / intrinsic.pixelsize;
	point2d[1] = (-y_mm + y0_mm + format_y_mm * 0.5) / intrinsic.pixelsize;

}

// =====PROTECT METHODS=====
/////////////////////////////////////////////////////////////////////////
void RangeImageTool::getPointCloudIndex()
{
	//Load camera intrinsics file and extrinsic file 
	cameraInfo->cmrFile_path = cmrPath;
	cameraInfo->phtFile_path = phtPath;
	cameraInfo->LoadCameraData();

	//Load the list of lasfile' absolute path;
	std::vector<std::string> lasFileList;
	std::map<std::string, BoundingBoxCorner3D> lasFileInfo;

	GetFiles(lasPath, ".las", lasFileList);

	//Get every lasfile the bottom face of boundingbox
	BoundingBoxCorner3D tmp;
	for (auto lasfile_name : lasFileList)
	{
		GetBoundingBox(lasfile_name, tmp);
		lasFileInfo.insert(std::pair<std::string, BoundingBoxCorner3D>(lasfile_name, tmp));
	}
	
	for (int i = 0; i < cameraInfo->m_cameras.size(); i++)
	{
		int imageid = cameraInfo->m_cameras[i].imageid;
		int intrinsics_group = cameraInfo->m_cameras[i].intrinsics_group;

		ImageLasInfo imglasinfo_tmp;

		getRotationAndTranslate(cameraInfo->m_cameras[i], imglasinfo_tmp);

		for (auto lasfile_name : lasFileList)
		{
			tmp = lasFileInfo[lasfile_name];

			bool flag = isHomologous(cameraInfo->m_intrinsics[intrinsics_group], imglasinfo_tmp, tmp);
			if (flag)
			{
				imglasinfo_tmp.las_list.push_back(lasfile_name);
				imglasinfo_tmp.box_list.push_back(tmp);
			}
		}
		imageInfoMap.insert(std::pair<int, ImageLasInfo>(imageid, imglasinfo_tmp));

	}
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::getRotationAndTranslate(const BACamera& extrinsic, ImageLasInfo& imageinfo)
{
	double phi = extrinsic.Posture(EOS_PHI);
	double omega = extrinsic.Posture(EOS_OMG);
	double kappa = extrinsic.Posture(EOS_KAP);

	Eigen::Matrix3d extrinsicMatrix = Eigen::Matrix3d::Identity();

	extrinsicMatrix(0, 0) = std::cos(phi)*std::cos(kappa) - std::sin(phi)*std::sin(omega)*std::sin(kappa);
	extrinsicMatrix(0, 1) = -std::cos(phi)* std::sin(kappa) - std::sin(phi)* std::sin(omega)* std::cos(kappa);
	extrinsicMatrix(0, 2) = -std::sin(phi)* std::cos(omega);

	extrinsicMatrix(1, 0) = std::cos(omega)* std::sin(kappa);
	extrinsicMatrix(1, 1) = std::cos(omega)* std::cos(kappa);
	extrinsicMatrix(1, 2) = -std::sin(omega);

	extrinsicMatrix(2, 0) = std::sin(phi)* std::cos(kappa) + std::cos(phi)* std::sin(omega)* std::sin(kappa);
	extrinsicMatrix(2, 1) = -std::sin(phi)* std::sin(kappa) + std::cos(phi)* std::sin(omega)* std::cos(kappa);
	extrinsicMatrix(2, 2) = std::cos(phi)* std::cos(omega);

	imageinfo.rotate_ = extrinsicMatrix;
	imageinfo.translate_ = extrinsic.Location;

}

/////////////////////////////////////////////////////////////////////////
bool RangeImageTool::isHomologous(const BACameraIntrinsics& intrinsic, const ImageLasInfo& imageinfo, const BoundingBoxCorner3D& box)
{
	bool flag = false;
	int sampleSize = 3;

	double format_x = intrinsic.format_x / intrinsic.pixelsize;
	double format_y = intrinsic.format_y / intrinsic.pixelsize;

	double pcLength = box.max_x - box.min_x;
	double pcWidth = box.max_y - box.min_y;

	double pcLengthInternal = pcLength / sampleSize;
	double pcWidthInternal = pcWidth / sampleSize;

	Eigen::Vector2d point2d;
	double tmp_x, tmp_y, tmp_z;
	tmp_z = box.min_z;

	for(int i=0;i<= sampleSize;++i)
		for (int j = 0; j <= sampleSize; ++j)
		{
			tmp_x = box.min_x + i * pcLengthInternal;
			tmp_y = box.min_y + j * pcWidthInternal;
			Eigen::Vector3d point3d(tmp_x, tmp_y, tmp_z);
			getImagePoint(intrinsic, imageinfo, point3d, point2d);

			if ((point2d[0] >= 0 && point2d[0] < format_x && point2d[1] >= 0 && point2d[1] < format_y))
			{
				flag = true;
				break;
			}
			
		}

	return flag;
}