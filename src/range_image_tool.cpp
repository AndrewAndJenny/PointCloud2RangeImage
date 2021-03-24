#include "range_image_tool.h"

RangeImageTool::RangeImageTool()
{
	maxElevation = DEFAULT_MAX_ELEVATION;
	cameraInfo = std::make_shared<BACameraFile>();
}

RangeImageTool::RangeImageTool(std::string cmrFile_path, std::string phtFile_path, std::string lasFolder_Path,std::string saveFolder_Path) :
	cmrPath(cmrFile_path), phtPath(phtFile_path), lasPath(lasFolder_Path),saveFolder(saveFolder_Path)
{
	maxElevation = DEFAULT_MAX_ELEVATION;
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
bool RangeImageTool::loadPointCloudFromList(const ImageLasInfo imgLasInfo, const BACameraIntrinsics intrinsic,pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
	int Grid_Level = 6;

	std::vector<std::string> las_list = imgLasInfo.las_list;
	std::vector<BoundingBoxCorner3D> box_list = imgLasInfo.box_list;

	if (las_list.size() == 0)
	{
		std::cout << "The laslist is empty, couldn't load ant point cloud!\n";
		return false;
	}

	pcl::PointXYZ point_tmp;

	for (int i=0;i< las_list.size();++i)
	{

		std::string las_file_path = las_list[i];
		BoundingBoxCorner3D box = box_list[i];

		LASreadOpener lasreadopener;
		LASheader las_header_read;
		LASreader *lasreader;

		// Open the LAS file
		lasreadopener.set_file_name(las_file_path.c_str());

		if (!lasreadopener.active()) {
			std::cout << "ERROR: no input specified" << std::endl;
			continue;
		}

		lasreader = lasreadopener.open();
		if (!lasreader) {
			std::cout << "ERROR: could not open LAS file: " << las_file_path << std::endl;
			continue;
		}

		//build mapping map
		double format_x = intrinsic.format_x / intrinsic.pixelsize;
		double format_y = intrinsic.format_y / intrinsic.pixelsize;

		double width = box.max_x - box.min_x;
		double height = box.max_y - box.min_y;

		double grid_threshold = (width + height)*0.5 / Grid_Level;

		int grid_nx = static_cast<int>(std::floor(width / grid_threshold)) + 1;
		int grid_ny = static_cast<int>(std::floor(height / grid_threshold)) + 1;

		size_t size = grid_ny*grid_nx ;
		int* point_map = new int[size];
		memset(point_map, 0, size * sizeof(*point_map));

		Eigen::Vector3d lest_point3d, best_point3d;
		Eigen::Vector2d lest_point2d, best_point2d;

		for (int i = 0; i < grid_ny; ++i)
		{
			for (int j = 0; j < grid_nx; ++j)
			{
				lest_point3d[0] = box.min_x + j * grid_threshold > box.max_x ? box.max_x : box.min_x + j * grid_threshold;
				lest_point3d[1] = box.min_y + i * grid_threshold > box.max_y ? box.max_y : box.min_y + i * grid_threshold;
				lest_point3d[2] = box.min_z;

				best_point3d[0] = box.min_x + (j+1) * grid_threshold > box.max_x ? box.max_x : box.min_x + (j + 1) * grid_threshold;
				best_point3d[1] = box.min_y + (i+1) * grid_threshold > box.max_y ? box.max_y : box.min_y + (i+1) * grid_threshold;
				best_point3d[2] = box.min_z;

				getImagePoint(intrinsic, imgLasInfo, lest_point3d, lest_point2d);
				getImagePoint(intrinsic, imgLasInfo, best_point3d, best_point2d);

				if ((lest_point2d[0] >= 0 && lest_point2d[0] < format_x && lest_point2d[1] >= 0 && lest_point2d[1] < format_y)
					|| (best_point2d[0] >= 0 && best_point2d[0] < format_x && best_point2d[1] >= 0 && best_point2d[1] < format_y))
					point_map[i*grid_nx + j] = 1;
				//std::cout << point_map[i*grid_nx + j] << "\t";
			}
			//std::cout << std::endl;
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

			if(point_tmp.z>maxElevation)
				continue;
			int temp_y = static_cast<int>(std::floor((point_tmp.y - box.min_y) / grid_threshold));
			int temp_x = static_cast<int>(std::floor((point_tmp.x - box.min_x) / grid_threshold));

			int point_ny = temp_y >0? temp_y :0;
			int point_nx = temp_x >0? temp_x :0;
			
			if(point_map[point_ny*grid_nx+ point_nx])
				point_cloud->points.push_back(point_tmp);
		}

		// Close the LAS file
		lasreader->close();
		delete[] point_map;
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
	std::cout << "Calculate the mapping relationship between point cloud and image successfully !\n";

	int imageid, intrinsics_group;
	for (int i = 0; i < cameraInfo->m_cameras.size(); i++)
	{
		imageid = cameraInfo->m_cameras[i].imageid;
		intrinsics_group = cameraInfo->m_cameras[i].intrinsics_group;

		ImageLasInfo imgLasInfo = imageInfoMap[imageid];
		BACameraIntrinsics& intrinsic = cameraInfo->m_intrinsics[intrinsics_group];

		if (!imgLasInfo.las_list.empty())
			generateSingleRangeImage(imageid, imgLasInfo, intrinsic);

	}
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::generateSingleRangeImage(int image_id, ImageLasInfo imgLasInfo, BACameraIntrinsics intrinsic)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	loadPointCloudFromList(imgLasInfo, intrinsic, point_cloud);
	float pixel_size = intrinsic.pixelsize;
	int format_x = static_cast<int>(intrinsic.format_x / pixel_size);
	int format_y = static_cast<int>(intrinsic.format_y / pixel_size);

	float x0 = static_cast<float>(intrinsic.intrins[OFFSET_X0] / pixel_size);
	float y0 = static_cast<float>(intrinsic.intrins[OFFSET_Y0] / pixel_size);
	float f0 = static_cast<float>(intrinsic.intrins[OFFSET_F0]);

	//Photogrammetry defination
	float cx = static_cast<float>(format_x *0.5) + x0;
	float cy = static_cast<float>(format_y *0.5) + y0;

	RangeImagePlanar range_image_planar;
	
	Eigen::Matrix3f Rotation = imgLasInfo.rotation.cast<float>();
	Eigen::Vector3f Location = imgLasInfo.translation.cast<float>();
	//只有pixelsize,f0单位为mm,其他都是像素坐标，没有单位
	range_image_planar.createFromPointCloudWithFixedSize(*point_cloud, format_x, format_y, cx, cy, f0, pixel_size, Rotation, Location);

	RangeImagePlanar::Ptr range_image_planar_ptr = range_image_planar.makeShared();

	exportInfo(range_image_planar_ptr, image_id);

	std::cout << image_id<< " image conver to range image completed!\n";
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::getImagePoint(const BACameraIntrinsics& intrinsic, const ImageLasInfo& imageinfo, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d)
{
	double format_x_mm = intrinsic.format_x;
	double format_y_mm = intrinsic.format_y;
	double x0_mm = intrinsic.intrins(OFFSET_X0);
	double y0_mm = intrinsic.intrins(OFFSET_Y0);
	double f0_mm = intrinsic.intrins(OFFSET_F0);

	Eigen::Matrix3d rotation = imageinfo.rotation;
	Eigen::Vector3d translation = imageinfo.translation;

	rotation.transposeInPlace();

	Eigen::Vector3d transformedPoint = rotation * (point3d - translation);

	double x_mm = -f0_mm * transformedPoint[0] / transformedPoint[2];
	double y_mm = -f0_mm * transformedPoint[1] / transformedPoint[2];

	point2d[0] = (x_mm + x0_mm + format_x_mm * 0.5) / intrinsic.pixelsize;
	point2d[1] = (-y_mm + y0_mm + format_y_mm * 0.5) / intrinsic.pixelsize;
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::getDenseRangeImage(Eigen::MatrixXf& difference_x, Eigen::MatrixXf& difference_y, Eigen::MatrixXf& sparse_range_image, Eigen::MatrixXf& dense_range_image)
{
	int windows_size = 2 * grid + 1;

	int rows = difference_x.rows();
	int cols = difference_x.cols();

	int dense_rows = rows - windows_size + 1;
	int dense_cols = cols - windows_size + 1;

	std::vector<std::vector<Eigen::MatrixXf>> KmX(windows_size);
	std::vector<std::vector<Eigen::MatrixXf>> KmY(windows_size);
	std::vector<std::vector<Eigen::MatrixXf>> KmD(windows_size);

	Eigen::MatrixXf tmp = Eigen::MatrixXf::Zero(dense_rows, dense_cols);
	for (int i = 0; i < windows_size; ++i)
		for (int j = 0; j < windows_size; ++j)
		{
			tmp.fill(i - grid);
			Eigen::MatrixXf KmX_ij = difference_x.block(i,j, dense_rows, dense_cols) - tmp;
			KmX[i].emplace_back(KmX_ij);

			tmp.fill(j - grid);
			Eigen::MatrixXf KmY_ij = difference_y.block(i, j, dense_rows, dense_cols) - tmp;
			KmY[i].emplace_back(KmY_ij);

			Eigen::MatrixXf KmD_ij = sparse_range_image.block(i, j, dense_rows, dense_cols);
			KmD[i].emplace_back(KmD_ij);
		}

	Eigen::MatrixXf sum_range = Eigen::MatrixXf::Zero(dense_rows, dense_cols);
	Eigen::MatrixXf sum_distance = Eigen::MatrixXf::Zero(dense_rows, dense_cols);

	Eigen::MatrixXf  tmp_distance;

	for (int i = 0; i < windows_size; ++i)
		for (int j = 0; j < windows_size; ++j)
		{
			tmp_distance = KmX[i][j].cwiseProduct(KmX[i][j]) + KmY[i][j].cwiseProduct(KmY[i][j]);
			tmp_distance = tmp_distance.cwiseSqrt();//sqrt()

			matrixInverse(tmp_distance);				//1./

			sum_range = sum_range + tmp_distance.cwiseProduct(KmD[i][j]);
			sum_distance = sum_distance + tmp_distance;
		}

	KmX.erase(KmX.begin(), KmX.end());
	KmY.erase(KmY.begin(), KmY.end());
	KmD.erase(KmD.begin(), KmD.end());

	auto mask = sum_distance.array() == 0;
	sum_distance = sum_distance + mask.matrix().cast<float>();

	dense_range_image = Eigen::MatrixXf::Zero(rows, cols);

	dense_range_image.block(grid, grid, dense_rows, dense_cols) = sum_range.cwiseQuotient(sum_distance);

}

/////////////////////////////////////////////////////////////////////////
bool RangeImageTool::exportInfo(RangeImagePlanar::Ptr cofiguration_info, int image_id)
{
	int format_y = cofiguration_info->getHeight();
	int format_x = cofiguration_info->getWidth();

	Eigen::MatrixXf real_range = Eigen::MatrixXf::Zero(format_y, format_x);
	Eigen::MatrixXf real_zvalue = Eigen::MatrixXf::Zero(format_y, format_x);
	
	cv::Mat real_xvalue_cv = cv::Mat::zeros(format_y, format_x, CV_32FC1);
	cv::Mat real_yvalue_cv = cv::Mat::zeros(format_y, format_x, CV_32FC1);
	for (int y = 0; y < format_y; ++y)
	{
		float* real_xvalue_ptr = real_xvalue_cv.ptr<float>(y);
		float* real_yvalue_ptr = real_yvalue_cv.ptr<float>(y);
		for (int x = 0; x < format_x; ++x)
		{
			pcl::PointWithRange& point = cofiguration_info->points[y*format_x + x];
			if (!std::isinf(point.range))
			{
				real_xvalue_ptr[x] = point.x;
				real_yvalue_ptr[x] = point.y;

				real_zvalue(y, x) = point.z;
				real_range(y, x) = point.range;
			}
		}
	}

	//Interpolation
	Eigen::MatrixXf interpolate_range, interpolate_zvalue;
	getDenseRangeImage(cofiguration_info->difference_x, cofiguration_info->difference_y, real_range, interpolate_range);
	getDenseRangeImage(cofiguration_info->difference_x, cofiguration_info->difference_y, real_zvalue, interpolate_zvalue);

	//format conversion
	cv::Mat interpolate_range_cv, interpolate_zvalue_cv, real_zvalue_cv;

	cv::eigen2cv(interpolate_range, interpolate_range_cv);
	cv::eigen2cv(interpolate_zvalue, interpolate_zvalue_cv);
	cv::eigen2cv(real_zvalue, real_zvalue_cv);

	//Calculate  three-dimensional coordinates 
	cv::Mat interpolate_range_position = cv::Mat::zeros(format_y, format_x, CV_32FC3);
	caculateAll3DPoints(cofiguration_info, interpolate_range_cv, interpolate_range_position);

	//Strech 'interpolate_zvalue_cv' to 0-255
	double min_gray_value, max_gray_value;

	cv::Mat mask = interpolate_zvalue_cv > 0;
	cv::minMaxLoc(interpolate_zvalue_cv, &min_gray_value, &max_gray_value, nullptr, nullptr, mask);

	float breadth = static_cast<float>(max_gray_value) - static_cast<float>(min_gray_value);

	cv::Mat min_mat = cv::Mat::zeros(format_y, format_x, CV_32FC1);
	min_mat.setTo(static_cast<float>(min_gray_value), mask);

	cv::Mat strech_dense_image_zvalue = (interpolate_zvalue_cv - min_mat) / breadth * 255;

	std::vector<cv::Mat> split_channels(3);
	cv::split(interpolate_range_position, split_channels);

	bool flag;
	//export strech dense zvalue image
	std::string interpolate_zvalue_file_path = saveFolder + "/" + std::to_string(image_id).append("_inpol_zvalue.jpg");
	flag = cv::imwrite(interpolate_zvalue_file_path, strech_dense_image_zvalue);

	//export real x,y,z image
	std::string real_x_file_path = saveFolder + "/" + std::to_string(image_id).append("_real_x.tiff");
	std::string real_y_file_path = saveFolder + "/" + std::to_string(image_id).append("_real_y.tiff");
	std::string real_z_file_path = saveFolder + "/" + std::to_string(image_id).append("_real_z.tiff");

	flag = cv::imwrite(real_x_file_path, real_xvalue_cv);
	flag = cv::imwrite(real_y_file_path, real_yvalue_cv);
	flag = cv::imwrite(real_z_file_path, real_zvalue_cv);

	//export x,y,z image after interpolating
	std::string inpol_x_file_path = saveFolder + "/" + std::to_string(image_id).append("_inpol_x.tiff");
	std::string inpol_y_file_path = saveFolder + "/" + std::to_string(image_id).append("_inpol_y.tiff");
	std::string inpol_z_file_path = saveFolder + "/" + std::to_string(image_id).append("_inpol_z.tiff");

	flag = cv::imwrite(inpol_x_file_path, split_channels[0]);
	flag = cv::imwrite(inpol_y_file_path, split_channels[1]);
	flag = cv::imwrite(inpol_z_file_path, split_channels[2]);

	return flag;
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

	Eigen::Matrix3d erotation = Eigen::Matrix3d::Identity();

	erotation(0, 0) = std::cos(phi)*std::cos(kappa) - std::sin(phi)*std::sin(omega)*std::sin(kappa);
	erotation(0, 1) = -std::cos(phi)* std::sin(kappa) - std::sin(phi)* std::sin(omega)* std::cos(kappa);
	erotation(0, 2) = -std::sin(phi)* std::cos(omega);

	erotation(1, 0) = std::cos(omega)* std::sin(kappa);
	erotation(1, 1) = std::cos(omega)* std::cos(kappa);
	erotation(1, 2) = -std::sin(omega);

	erotation(2, 0) = std::sin(phi)* std::cos(kappa) + std::cos(phi)* std::sin(omega)* std::sin(kappa);
	erotation(2, 1) = -std::sin(phi)* std::sin(kappa) + std::cos(phi)* std::sin(omega)* std::cos(kappa);
	erotation(2, 2) = std::cos(phi)* std::cos(omega);

	imageinfo.rotation = erotation;
	imageinfo.translation = extrinsic.Location;

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

	for (int i = 0; i <= sampleSize; ++i)
	{
		for (int j = 0; j <= sampleSize; ++j)
		{
			tmp_x = box.min_x + i * pcLengthInternal;
			tmp_y = box.min_y + j * pcWidthInternal;
			Eigen::Vector3d point3d(tmp_x, tmp_y, tmp_z);
			getImagePoint(intrinsic, imageinfo, point3d, point2d);

			if ((point2d[0] >= 0 && point2d[0] < format_x && point2d[1] >= 0 && point2d[1] < format_y))
				return true;
		}
	}
	return false;
}

/////////////////////////////////////////////////////////////////////////
void RangeImageTool::caculateAll3DPoints(RangeImagePlanar::Ptr cofiguration_info, cv::Mat& range_image, cv::Mat& range_image_3Dposition)
{
	//configuration information
	float pixel_size = cofiguration_info->getPixelSize();
	float focal_length = cofiguration_info->getFocalLength();
	int format_x = range_image.cols;
	int format_y = range_image.rows;

	float center_x = cofiguration_info->getCenterX();
	float center_y = cofiguration_info->getCenterY();

	Eigen::Matrix3f to_world_system = cofiguration_info->getToWorldSystem();
	Eigen::Vector3f camera_world_location = cofiguration_info->getCameraWorldPosition();

	//work
	int height = range_image_3Dposition.rows;
	int width = range_image_3Dposition.cols;

	cv::Mat point_temp = cv::Mat::zeros(3, 1, CV_32FC1);
	float range;
	for (int y = 0; y < static_cast<int> (height); ++y)
	{
		float* range_ptr = range_image.ptr<float>(y);
		for (int x = 0; x < static_cast<int> (width); ++x)
		{
			range = range_ptr[x];
			if (range > 0)
			{
				float delta_x = (static_cast<float> (x) - center_x)*pixel_size;
				float delta_y = (-static_cast<float> (y) + center_y)*pixel_size;

				Eigen::Vector3f deltaPoint(delta_x, delta_y, -focal_length);
				Eigen::Vector3f transformedPoint = to_world_system * deltaPoint;

				float projectScale = range / transformedPoint.norm();

				Eigen::Vector3f point = projectScale * transformedPoint + camera_world_location;

				cv::Vec3f& point_cv = range_image_3Dposition.at<cv::Vec3f>(y, x);
				point_cv[0] = point[0];
				point_cv[1] = point[1];
				point_cv[2] = point[2];
			}
			
		}
	}
}