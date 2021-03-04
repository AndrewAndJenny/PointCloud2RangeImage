#include "range_image_tool.h"
#include "range_image_planar.h"

RangeImageTool::RangeImageTool()
{
	cameraInfo = std::make_shared<BACameraFile>();
}

RangeImageTool::RangeImageTool(std::string cmrFile_path, std::string phtFile_path, std::string lasFolder_Path,std::string saveFolder_Path) :
	cmrPath(cmrFile_path), phtPath(phtFile_path), lasPath(lasFolder_Path),saveFolder(saveFolder_Path)
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
bool RangeImageTool::loadPointCloudFromList(const std::vector<std::string> lasList, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, BoundingBoxCorner3D& box)
{
	if (lasList.size() == 0)
	{
		std::cout << "The laslist is empty, couldn't load ant point cloud!\n";
		return false;
	}

	box.min_x = DBL_MAX, box.min_y = DBL_MAX, box.min_z = DBL_MAX;
	box.max_z = -DBL_MAX, box.max_y = -DBL_MAX, box.max_z = -DBL_MAX;

	double x, y, z;
	pcl::PointXYZ point_tmp;

	for (auto lasFilePath : lasList)
	{
		LASreadOpener lasreadopener;
		LASheader las_header_read;
		LASreader *lasreader;

		// Open the LAS file
		lasreadopener.set_file_name(lasFilePath.c_str());

		if (!lasreadopener.active()) {
			std::cout << "ERROR: no input specified" << std::endl;
			continue;;
		}

		lasreader = lasreadopener.open();
		if (!lasreader) {
			std::cerr << "ERROR: could not open LAS file: " << lasFilePath << std::endl;
			continue;
		}

		las_header_read = lasreader->header;
		las_header_read.user_data_after_header = nullptr;
		U8 point_type = las_header_read.point_data_format;
		U16 point_size = las_header_read.point_data_record_length;

		LASpoint point_r;
		point_r.init(&las_header_read, point_type, point_size, &las_header_read);

		while (lasreader->read_point())
		{
			point_r = lasreader->point;

			point_tmp.x = static_cast<float>(point_r.get_x());
			point_tmp.y = static_cast<float>(point_r.get_y());
			point_tmp.z = static_cast<float>(point_r.get_z());

			point_cloud->points.push_back(point_tmp);
		}

		box.min_x = box.min_x > las_header_read.min_x ? las_header_read.min_x : box.min_x;
		box.min_y = box.min_y > las_header_read.min_y ? las_header_read.min_y : box.min_y;
		box.min_z = box.min_z > las_header_read.min_z ? las_header_read.min_z : box.min_z;
		box.max_x = box.max_x > las_header_read.max_x ? box.max_x : las_header_read.max_x;
		box.max_y = box.max_y > las_header_read.max_y ? box.max_y : las_header_read.max_y;
		box.max_z = box.max_z > las_header_read.max_z ? box.max_z : las_header_read.max_z;

		// Close the LAS file
		lasreader->close();
	}

	point_cloud->width = point_cloud->points.size();
	point_cloud->height = 1;

	return true;
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::generateRangeImage()
{
	//get every  images' lasfiles index 
	getPointCloudIndex();

	int imageid, intrinsics_group_;
	for (int i = 0; i < cameraInfo->m_cameras.size(); i++)
	{
		imageid = cameraInfo->m_cameras[i].imageid;
		intrinsics_group_ = cameraInfo->m_cameras[i].intrinsics_group;

		ImageLasInfo imgLasInfo_ = imageInfoMap[imageid];
		BACameraIntrinsics& intrinsic_ = cameraInfo->m_intrinsics[intrinsics_group_];

		if (!imgLasInfo_.lasList.empty())
			generateSingleRangeImage(imageid, imgLasInfo_, intrinsic_);
	}
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::generateSingleRangeImage(int imageid, const ImageLasInfo& imgLasInfo, const BACameraIntrinsics& intrinsic_)
{
	//先读取lasList里所有点云，进行分块划分提取相关点云，最后调用RangeImagePlanar api
	BoundingBoxCorner3D box;
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	loadPointCloudFromList(imgLasInfo.lasList, point_cloud, box);

	int format_x = static_cast<int>(intrinsic_.format_x / intrinsic_.pixelsize);
	int format_y = static_cast<int>(intrinsic_.format_y / intrinsic_.pixelsize);

	float x0 = static_cast<float>(intrinsic_.intrins[OFFSET_X0] / intrinsic_.pixelsize);
	float y0 = static_cast<float>(intrinsic_.intrins[OFFSET_Y0] / intrinsic_.pixelsize);
	float f0 = static_cast<float>(intrinsic_.intrins[OFFSET_F0]);

	//Photogrammetry defination
	float cx = static_cast<float>(format_x *0.5) + x0;
	float cy = static_cast<float>(format_y *0.5) + y0;

	RangeImagePlanar RangeImagePlanar;

	Eigen::Matrix3f Rotation = imgLasInfo.rotate_.cast<float>();
	Eigen::Vector3f Location = imgLasInfo.translate_.cast<float>();
	//只有pixelsize,f0单位为mm,其他都是像素坐标，没有单位
	RangeImagePlanar.createFromPointCloudWithFixedSize(*point_cloud, format_x, format_y, cx, cy, f0, intrinsic_.pixelsize, Rotation, Location);

	/*std::vector<std::string> lasFileList;

	GetFiles(lasPath, "las", lasFileList);
	RangeImagePlanar.createTest(lasFileList, format_x, format_y, cx, cy, f0, intrinsic_.pixelsize, Rotation, Location);*/

	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

	GDALDriver *pDriver = GetGDALDriverManager()->GetDriverByName("GTiFF"); //图像驱动
	char** ppszOptions = NULL;
	//ppszOptions = CSLSetNameValue(ppszOptions, "BIGTIFF", "IF_NEEDED"); //配置图像信息

	std::string dstPath = saveFolder + "/" + std::to_string(imageid).append(".tif");
	int bufWidth = format_x;
	int bufHeight = format_y;
	int bandNum = 1;
	GDALDataset* dst = pDriver->Create(dstPath.c_str(), bufWidth, bufHeight, bandNum, GDT_Float32, ppszOptions);
	if (dst == nullptr)
	{
		printf("Can't Write Image!");
		return;
	}

	size_t imgBufNum = (size_t)bufWidth * bufHeight * bandNum;
	float *imgBuf = new float[imgBufNum];
	memset(imgBuf, 0, imgBufNum * sizeof(float));

	for (int y = 0; y < format_y; ++y)
	{
		for (int x = 0; x < format_x; ++x)
		{
			pcl::PointWithRange& point = RangeImagePlanar.points[y*format_x + x];
			if (!std::isinf(point.range))
				imgBuf[y*format_x + x] = point.range;
		}
	}

	dst->RasterIO(GF_Write, 0, 0, bufWidth, bufHeight, imgBuf, bufWidth, bufHeight,
		GDT_Float32, bandNum, NULL, 0,0,0);
	
	delete[] imgBuf;
	imgBuf = nullptr;
	GDALClose(dst);

	std::cout << imageid<< " image conver to range image completed!\n";
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

	GetFiles(lasPath, "las", lasFileList);

	//Get every lasfile the bottom face of boundingbox
	BoundingBoxCorner3D tmp;
	for (auto lasfile_name : lasFileList)
	{
		GetBoundingBox(lasfile_name, tmp);
		lasFileInfo.insert(std::pair<std::string, BoundingBoxCorner3D>(lasfile_name, tmp));
	}
	//获取每张相片的正交旋转矩阵，然后遍历将所有点云box投影到相片上，判断哪些las文件属于这一张相片
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
				imglasinfo_tmp.lasList.push_back(lasfile_name);
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