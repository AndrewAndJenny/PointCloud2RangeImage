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
		pixel_size_(0.0f),center_x_(0.0f), center_y_(0.0f) , 
		to_range_image_system_(Eigen::Matrix3f::Identity()),
		to_world_system_(Eigen::Matrix3f::Identity())
	{
		reset();
		unobserved_point.x = unobserved_point.y = unobserved_point.z = std::numeric_limits<float>::quiet_NaN();
		unobserved_point.range = -std::numeric_limits<float>::infinity();
	};
	/** Destructor */
	virtual ~RangeImagePlanar() {};

	// =====PUBLIC METHODS=====
	/** \brief Get a std shared pointer of a copy of this */
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
		void createFromPointCloudWithFixedSize(const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
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
		void doZBuffer(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, float noise_level, float min_range, int& top, int& right, int& bottom, int& left);

	/** Calculate the 3D point according to the given image point and range */
	inline void calculate3DPoint(float image_x, float image_y, float range, pcl::PointWithRange& point) const;

	/** Recalculate all 3D point positions according to their pixel position and range */
	void recalculate3DPointPositions();

	/** Transforms an image point in float values to an image point in int values */
	inline void real2DToInt2D(float x, float y, int& xInt, int& yInt) const;

	/** Check if a point is inside of the image */
	inline bool isInImage(int x, int y) const;

	/** \brief Calculate the 3D point according to the given image point and range
	  * \param image_x the x image position
	  * \param image_y the y image position
	  * \param range the range
	  * \param point the resulting 3D point
	  * \note Implementation according to planar range images (compared to spherical as in the original)
	  */
	inline void calculate3DPoint(float image_x, float image_y, float range, Eigen::Vector3f& point) const;

	/** \brief Calculate the image point and range from the given 3D point
	  * \param point the resulting 3D point
	  * \param image_x the resulting x image position
	  * \param image_y the resulting y image position
	  * \param range the resulting range
	  * \note Implementation according to planar range images (compared to spherical as in the original)
	  */
	inline void getImagePoint(const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const;

	//!Getter for the to_world_system
	inline Eigen::Matrix3f getToWorldSystem()const { return to_world_system_; }

	//!Getter for the camera world position
	inline Eigen::Vector3f getCameraWorldPosition()const { return location_; }

	//!Getter for the pixel size
	inline float getPixelSize()const { return pixel_size_; }

	//! Getter for the focal length
	inline float getFocalLength() const { return focal_length; }

	//! Getter for the principal point in X
	inline float getCenterX() const { return center_x_; }

	//! Getter for the principal point in Y
	inline float getCenterY() const { return center_y_; }

	//!Getter for the width
	inline int getWidth() const { return width; }

	//!Getter for the height
	inline int getHeight() const { return height; }
	// =====MEMBER VARIABLES=====
	// BaseClass:
	//   roslib::Header header;
	//   std::vector<PointT> points;
	//   uint32_t width;
	//   uint32_t height;
	//   bool is_dense;

public:
	 Eigen::MatrixXf difference_x;
	 Eigen::MatrixXf difference_y;

protected:
	Eigen::Matrix3f to_range_image_system_;  /**< Inverse of to_world_system_ */
	Eigen::Matrix3f to_world_system_;        /**< Inverse of to_range_image_system_ */
	Eigen::Vector3f location_;		/*<the Camera center world coordinates*/

	float pixel_size_;		//<The pixel size													Unit mm
	float focal_length; //< The focal length of the image					    Unit mm
	float center_x_, center_y_;      //< The principle point of the image

	pcl::PointWithRange unobserved_point;
};

#endif // RANGEIMAGEPLANAR_H
