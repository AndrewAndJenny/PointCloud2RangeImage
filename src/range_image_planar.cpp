#include "range_image_planar.h"


/////////////////////////////////////////////////////////////////////////
void RangeImagePlanar::createFromPointCloudWithFixedSize(const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
	int di_width, int di_height, float di_center_x, float di_center_y, float di_focal_length, float pixel_size,
	const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation,
	float noise_level, float min_range)
{
	//std::cout << "Starting to create range image from "<<point_cloud.points.size ()<<" points.\n";

	width = di_width;
	height = di_height;
	center_x_ = di_center_x;
	center_y_ = di_center_y;
	focal_length = di_focal_length;

	pixel_size_ = pixel_size;

	is_dense = false;

	to_world_system_ = rotation;

	to_range_image_system_ = rotation;
	to_range_image_system_.transposeInPlace();

	location_ = translation;
	
	difference_x = Eigen::MatrixXf::Zero(height, width);
	difference_y = Eigen::MatrixXf::Zero(height, width);

	unsigned int size = width * height;
	points.clear();
	points.resize(size, unobserved_point);

	int top = height, right = -1, bottom = -1, left = width;
	doZBuffer(point_cloud, noise_level, min_range, top, right, bottom, left);

	recalculate3DPointPositions();
}

void RangeImagePlanar::doZBuffer(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float noise_level, float min_range, int& top, int& right, int& bottom, int& left)
{
	typedef  pcl::PointCloud<pcl::PointXYZ>::PointType PointType2;
	const  pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;

	unsigned int size = width * height;
	int* counters = new int[size];
	memset(counters, 0, size * sizeof(*counters));

	top = height; right = -1; bottom = -1; left = width;

	float x_real, y_real, range_of_current_point;
	int x, y;
	for (typename pcl::PointCloud<PointType2>::VectorType::const_iterator it = points2.begin(); it != points2.end(); ++it)
	{
		if (!isFinite(*it))  // Check for NAN etc
			continue;
		pcl::Vector3fMapConst current_point = it->getVector3fMap();

		getImagePoint(current_point, x_real, y_real, range_of_current_point);
		real2DToInt2D(x_real, y_real, x, y);

		if (range_of_current_point < min_range || !isInImage(x, y))
			continue;

		// Do some minor interpolation by checking the three closest neighbors to the point, that are not filled yet.
		int floor_x = pcl_lrint(floor(x_real)), floor_y = pcl_lrint(floor(y_real)),
			ceil_x = pcl_lrint(ceil(x_real)), ceil_y = pcl_lrint(ceil(y_real));

		int neighbor_x[4], neighbor_y[4];
		neighbor_x[0] = floor_x; neighbor_y[0] = floor_y;
		neighbor_x[1] = floor_x; neighbor_y[1] = ceil_y;
		neighbor_x[2] = ceil_x;  neighbor_y[2] = floor_y;
		neighbor_x[3] = ceil_x;  neighbor_y[3] = ceil_y;

		for (int i = 0; i < 4; ++i)
		{
			int n_x = neighbor_x[i], n_y = neighbor_y[i];

			if (n_x == x && n_y == y)
				continue;
			if (isInImage(n_x, n_y))
			{
				int neighbor_array_pos = n_y * width + n_x;
				if (counters[neighbor_array_pos] == 0)
				{
					float& neighbor_range = points[neighbor_array_pos].range;
					neighbor_range = (pcl_isinf(neighbor_range) ? range_of_current_point : (std::min) (neighbor_range, range_of_current_point));
					top = (std::min) (top, n_y); right = (std::max) (right, n_x); bottom = (std::max) (bottom, n_y); left = (std::min) (left, n_x);
				}
			}
		}

		// The point itself
		int arrayPos = y * width + x;
		float& range_at_image_point = points[arrayPos].range;
		int& counter = counters[arrayPos];
		bool addCurrentPoint = false, replace_with_current_point = false;

		if (counter == 0)
		{
			replace_with_current_point = true;
		}
		else
		{
			if (range_of_current_point < range_at_image_point - noise_level)
			{
				replace_with_current_point = true;
			}
			else if (fabs(range_of_current_point - range_at_image_point) <= noise_level)
			{
				addCurrentPoint = true;
			}
		}

		if (replace_with_current_point)
		{
			counter = 1;
			range_at_image_point = range_of_current_point;
			top = (std::min) (top, y); right = (std::max) (right, x); bottom = (std::max) (bottom, y); left = (std::min) (left, x);

			difference_x(y, x) = x_real - x;
			difference_y(y, x) = y_real - y;
		}
		else if (addCurrentPoint)
		{
			++counter;
			range_at_image_point += (range_of_current_point - range_at_image_point) / counter;

			difference_x(y, x) = x_real - x;
			difference_y(y, x) = y_real - y;
		}
	}

	delete[] counters;
}

bool RangeImagePlanar::isInImage(int x, int y) const
{
	return (x >= 0 && x < static_cast<int> (width) && y >= 0 && y < static_cast<int> (height));
}

void RangeImagePlanar::real2DToInt2D(float x, float y, int& xInt, int& yInt) const
{
	xInt = static_cast<int> (pcl_lrintf(x));
	yInt = static_cast<int> (pcl_lrintf(y));
}

void RangeImagePlanar::calculate3DPoint(float image_x, float image_y, float range, pcl::PointWithRange& point) const {
	point.range = range;
	Eigen::Vector3f tmp_point;
	calculate3DPoint(image_x, image_y, range, tmp_point);
	point.x = tmp_point[0];  point.y = tmp_point[1];  point.z = tmp_point[2];
}

void RangeImagePlanar::recalculate3DPointPositions()
{
	for (int y = 0; y < static_cast<int> (height); ++y)
	{
		for (int x = 0; x < static_cast<int> (width); ++x)
		{
			pcl::PointWithRange& point = points[y*width + x];
			if (!std::isinf(point.range))
				calculate3DPoint(static_cast<float> (x), static_cast<float> (y), point.range, point);
		}
	}
}

/////////////////////////////////////////////////////////////////////////
void RangeImagePlanar::calculate3DPoint(float image_x, float image_y, float range, Eigen::Vector3f& point) const
{
	float delta_x = (image_x - center_x_)*pixel_size_,
		delta_y = (-image_y + center_y_)*pixel_size_;

	Eigen::Vector3f deltaPoint(delta_x, delta_y, -focal_length);
	Eigen::Vector3f transformedPoint = to_world_system_ * deltaPoint;

	float projectScale = range / transformedPoint.norm();

	point = projectScale * transformedPoint + location_;
}

/////////////////////////////////////////////////////////////////////////
inline void RangeImagePlanar::getImagePoint(const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const
{
	Eigen::Vector3f deltaPoint = point - location_;
	Eigen::Vector3f transformedPoint = to_range_image_system_ * deltaPoint;

	range = deltaPoint.norm();

	float x_mm = -focal_length * transformedPoint[0] / transformedPoint[2];
	float y_mm = -focal_length * transformedPoint[1] / transformedPoint[2];

	image_x = x_mm / pixel_size_ + center_x_;
	image_y = -y_mm / pixel_size_ + center_y_;

}