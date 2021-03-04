#ifndef RANGEIMAGEPLANAR_H
#define RANGEIMAGEPLANAR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/common/vector_average.h>

class RangeImagePlanar:public pcl::PointCloud<pcl::PointWithRange>
{
public:
	// =====TYPEDEFS=====
	typedef pcl::PointCloud<pcl::PointWithRange> BaseClass;
	typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > VectorOfEigenVector3f;
	typedef boost::shared_ptr<RangeImagePlanar> Ptr;
	typedef boost::shared_ptr<const RangeImagePlanar> ConstPtr;

	// =====CONSTRUCTOR & DESTRUCTOR=====
	/** Constructor */
	RangeImagePlanar() : focal_length(0.0f),
		pixel_size_(0.0f),
		center_x_(0.0f), center_y_(0.0f) , to_range_image_system_(Eigen::Matrix3f::Identity()),
		to_world_system_(Eigen::Matrix3f::Identity())
	{
		reset();
		unobserved_point.x = unobserved_point.y = unobserved_point.z = std::numeric_limits<float>::quiet_NaN();
		unobserved_point.range = -std::numeric_limits<float>::infinity();
	};
	/** Destructor */
	virtual ~RangeImagePlanar() {};

	// =====PUBLIC METHODS=====
	/** \brief Get a boost shared pointer of a copy of this */
	inline Ptr
		makeShared() { return Ptr(new RangeImagePlanar(*this)); }

	void reset()
	{
		is_dense = true;
		width = height = 0;
		points.clear();
		pixel_size_ = 0;
		focal_length = 0;
		center_x_ = center_y_ = 0;
		to_range_image_system_.setIdentity();
		to_world_system_.setIdentity();
		
	}
	/** Create the image from an existing point cloud.
	  * \param point_cloud the source point cloud
	  * \param di_width the disparity image width
	  * \param di_height the disparity image height
	  * \param di_center_x the x-coordinate of the camera's center of projection
	  * \param di_center_y the y-coordinate of the camera's center of projection
	  * \param di_focal_length the camera's focal length
	  * \param Rotation the Rotate Orthogonal matrix
	  * \param Location the Camera center world coordinates
	  * \param noise_level what is the typical noise of the sensor - is used for averaging in the z-buffer
	  * \param min_range minimum range to consifder points
	  */
	template <typename PointCloudType> void
		createFromPointCloudWithFixedSize(const PointCloudType& point_cloud,
			int di_width, int di_height, float di_center_x, float di_center_y, float di_focal_length, float pixel_size,
			const Eigen::Matrix3f& Rotation,const Eigen::Vector3f& Location,
			 float noise_level = 0.0f,float min_range = 0.0f);

	/** \brief Integrate the given point cloud into the current range image using a z-buffer
		* \param point_cloud the input point cloud
		* \param noise_level - The distance in meters inside of which the z-buffer will not use the minimum,
		*                      but the mean of the points. If 0.0 it is equivalent to a normal z-buffer and
		*                      will always take the minimum per cell.
		* \param min_range the minimum visible range
		* \param top    returns the minimum y pixel position in the image where a point was added
		* \param right  returns the maximum x pixel position in the image where a point was added
		* \param bottom returns the maximum y pixel position in the image where a point was added
		* \param top returns the minimum y position in the image where a point was added
		* \param left   returns the minimum x pixel position in the image where a point was added
		*/
	template <typename PointCloudType> void
		doZBuffer(const PointCloudType& point_cloud, float noise_level, float min_range, int& top, int& right, int& bottom, int& left);

	/** Calculate the 3D point according to the given image point and range */
	inline void calculate3DPoint(float image_x, float image_y, float range, pcl::PointWithRange& point) const;

	/** Recalculate all 3D point positions according to their pixel position and range */
	void recalculate3DPointPositions();

	/** Transforms an image point in float values to an image point in int values */
	inline void
		real2DToInt2D(float x, float y, int& xInt, int& yInt) const;

	/** Check if a point is inside of the image */
	inline bool
		isInImage(int x, int y) const;

	/** \brief Calculate the 3D point according to the given image point and range
	  * \param image_x the x image position
	  * \param image_y the y image position
	  * \param range the range
	  * \param point the resulting 3D point
	  * \note Implementation according to planar range images (compared to spherical as in the original)
	  */
	inline void
		calculate3DPoint(float image_x, float image_y, float range, Eigen::Vector3f& point) const;

	/** \brief Calculate the image point and range from the given 3D point
	  * \param point the resulting 3D point
	  * \param image_x the resulting x image position
	  * \param image_y the resulting y image position
	  * \param range the resulting range
	  * \note Implementation according to planar range images (compared to spherical as in the original)
	  */
	virtual inline void
		getImagePoint(const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const;

	//! Getter for the focal length
	inline float
		getFocalLengthX() const { return focal_length; }

	//! Getter for the principal point in X
	inline float
		getCenterX() const { return center_x_; }

	//! Getter for the principal point in Y
	inline float
		getCenterY() const { return center_y_; }

	// =====MEMBER VARIABLES=====
	// BaseClass:
	//   roslib::Header header;
	//   std::vector<PointT> points;
	//   uint32_t width;
	//   uint32_t height;
	//   bool is_dense;

protected:
	Eigen::Matrix3f to_range_image_system_;  /**< Inverse of to_world_system_ */
	Eigen::Matrix3f to_world_system_;        /**< Inverse of to_range_image_system_ */
	Eigen::Vector3f location_;		/*<the Camera center world coordinates*/
	float pixel_size_;		//<The pixel size													Unit mm
	float focal_length; //< The focal length of the image					    Unit mm
	float center_x_, center_y_;      //< The principle point of the image

	pcl::PointWithRange unobserved_point;
};

/////////////////////////////////////////////////////////////////////////
template <typename PointCloudType> void
RangeImagePlanar::createFromPointCloudWithFixedSize(const PointCloudType& point_cloud,
	int di_width, int di_height, float di_center_x, float di_center_y, float di_focal_length, float pixel_size,
	const Eigen::Matrix3f& Rotation, const Eigen::Vector3f& Location,
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

	to_world_system_ = Rotation;
	
	to_range_image_system_ = Rotation;
	to_range_image_system_.transposeInPlace();

	location_ = Location;

	unsigned int size = width * height;
	points.clear();
	points.resize(size, unobserved_point);

	int top = height, right = -1, bottom = -1, left = width;
	doZBuffer(point_cloud, noise_level, min_range, top, right, bottom, left);

	recalculate3DPointPositions();
}

template <typename PointCloudType> void RangeImagePlanar::doZBuffer(const PointCloudType& point_cloud, float noise_level, float min_range, int& top, int& right, int& bottom, int& left)
{
	typedef typename PointCloudType::PointType PointType2;
	const typename pcl::PointCloud<PointType2>::VectorType &points2 = point_cloud.points;

	unsigned int size = width * height;
	int* counters = new int[size];
	ERASE_ARRAY(counters, size);

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
		}
		else if (addCurrentPoint)
		{
			++counter;
			range_at_image_point += (range_of_current_point - range_at_image_point) / counter;
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
	float delta_x = (image_x  - center_x_)*pixel_size_,
		delta_y = (-image_y+center_y_)*pixel_size_;

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

#endif // RANGEIMAGEPLANAR_H
