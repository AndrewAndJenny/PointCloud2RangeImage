#include "range_image_runable.h"

#include "range_image_planar.h"
#include "DataStruct.h"
#include "lasreader.hpp"
#include "Eigen/Core"

#include "gdal.h"
#include "gdal_priv.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

bool RangeImageRunable::loadPointCloudFromList(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
	//const ImageLasInfo imgLasInfo, const BACameraIntrinsics intrinsic,
	int Grid_Level = 6;

	std::vector<std::string> las_list = range_image_info_.las_list;
	std::vector<BoundingBoxCorner3D> box_list = range_image_info_.box_list;

	if (las_list.size() == 0)
	{
		std::cout << "The laslist is empty, couldn't load ant point cloud!\n";
		return false;
	}

	pcl::PointXYZ point_tmp;

	for (int i = 0; i < las_list.size(); ++i)
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
		double format_x = camera_intrinsic_.format_x / camera_intrinsic_.pixelsize;
		double format_y = camera_intrinsic_.format_y / camera_intrinsic_.pixelsize;

		double width = box.max_x - box.min_x;
		double height = box.max_y - box.min_y;

		double grid_threshold = (width + height)*0.5 / Grid_Level;

		int grid_nx = static_cast<int>(std::floor(width / grid_threshold)) + 1;
		int grid_ny = static_cast<int>(std::floor(height / grid_threshold)) + 1;

		size_t size = grid_ny * grid_nx;
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

				best_point3d[0] = box.min_x + (j + 1) * grid_threshold > box.max_x ? box.max_x : box.min_x + (j + 1) * grid_threshold;
				best_point3d[1] = box.min_y + (i + 1) * grid_threshold > box.max_y ? box.max_y : box.min_y + (i + 1) * grid_threshold;
				best_point3d[2] = box.min_z;

				getImagePoint(lest_point3d, lest_point2d);
				getImagePoint(best_point3d, best_point2d);

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

			int temp_y = static_cast<int>(std::floor((point_tmp.y - box.min_y) / grid_threshold));
			int temp_x = static_cast<int>(std::floor((point_tmp.x - box.min_x) / grid_threshold));

			int point_ny = temp_y > 0 ? temp_y : 0;
			int point_nx = temp_x > 0 ? temp_x : 0;

			if (point_map[point_ny*grid_nx + point_nx])
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

void RangeImageRunable::getImagePoint(const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d)
{
	double format_x_mm = camera_intrinsic_.format_x;
	double format_y_mm = camera_intrinsic_.format_y;
	double x0_mm = camera_intrinsic_.intrins(OFFSET_X0);
	double y0_mm = camera_intrinsic_.intrins(OFFSET_Y0);
	double f0_mm = camera_intrinsic_.intrins(OFFSET_F0);

	Eigen::Matrix3d Rotate_ = range_image_info_.rotate_;
	Eigen::Vector3d Location_ = range_image_info_.translate_;

	Rotate_.transposeInPlace();

	Eigen::Vector3d transformedPoint = Rotate_ * (point3d - Location_);

	double x_mm = -f0_mm * transformedPoint[0] / transformedPoint[2];
	double y_mm = -f0_mm * transformedPoint[1] / transformedPoint[2];

	point2d[0] = (x_mm + x0_mm + format_x_mm * 0.5) / camera_intrinsic_.pixelsize;
	point2d[1] = (-y_mm + y0_mm + format_y_mm * 0.5) / camera_intrinsic_.pixelsize;

}

void RangeImageRunable::run()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	loadPointCloudFromList(point_cloud);

	int format_x = static_cast<int>(camera_intrinsic_.format_x / camera_intrinsic_.pixelsize);
	int format_y = static_cast<int>(camera_intrinsic_.format_y / camera_intrinsic_.pixelsize);

	float x0 = static_cast<float>(camera_intrinsic_.intrins[OFFSET_X0] / camera_intrinsic_.pixelsize);
	float y0 = static_cast<float>(camera_intrinsic_.intrins[OFFSET_Y0] / camera_intrinsic_.pixelsize);
	float f0 = static_cast<float>(camera_intrinsic_.intrins[OFFSET_F0]);

	//Photogrammetry defination
	float cx = static_cast<float>(format_x *0.5) + x0;
	float cy = static_cast<float>(format_y *0.5) + y0;

	RangeImagePlanar rangeImagePlanar;

	Eigen::Matrix3f Rotation = range_image_info_.rotate_.cast<float>();
	Eigen::Vector3f Location = range_image_info_.translate_.cast<float>();
	//ֻ��pixelsize,f0��λΪmm,���������������꣬û�е�λ
	rangeImagePlanar.createFromPointCloudWithFixedSize(*point_cloud, format_x, format_y, cx, cy, f0, camera_intrinsic_.pixelsize, Rotation, Location);

	//write into tif
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

	GDALDriver *pDriver = GetGDALDriverManager()->GetDriverByName("GTiFF");
	char** ppszOptions = NULL;
	//ppszOptions = CSLSetNameValue(ppszOptions, "BIGTIFF", "IF_NEEDED"); 

	std::string dstPath = save_folder_path_ + "/" + std::to_string(imageid_).append(".tif");
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
			pcl::PointWithRange& point = rangeImagePlanar.points[y*format_x + x];
			if (!std::isinf(point.range))
				imgBuf[y*format_x + x] = point.range;
		}
	}

	dst->RasterIO(GF_Write, 0, 0, bufWidth, bufHeight, imgBuf, bufWidth, bufHeight,
		GDT_Float32, bandNum, NULL, 0, 0, 0);

	delete[] imgBuf;
	imgBuf = nullptr;
	GDALClose(dst);

	std::cout << imageid_ << " image conver to range image completed!\n";
}