#ifndef _DATASTRUCT_H_
#define _DATASTRUCT_H_

#include <vector>
#include <string>
#include "Eigen/Core"

#define PER_IOS_BLOCK_NUM 10

enum OFFSET{
	OFFSET_F0,
	OFFSET_X0,
	OFFSET_Y0,
	OFFSET_K1,
	OFFSET_K2,
	OFFSET_K3,
	OFFSET_P1,
	OFFSET_P2,
	OFFSET_B1,
	OFFSET_B2
};

enum EOS_LOCATION
{
	EOS_XS = 0,
	EOS_YS = 1,
	EOS_ZS = 2
};

enum EOS_POSTURE
{
	EOS_PHI = 0,
	EOS_OMG = 1,
	EOS_KAP = 2
};

struct BACameraIntrinsics
{
	BACameraIntrinsics() {}
	BACameraIntrinsics(const BACameraIntrinsics& intrinsics) :
		intrins(intrinsics.intrins) {
		format_x = format_y = 1.0;
		pixelsize = 0.006;
	}
	double format_x, format_y, pixelsize; //ibundle内部接口,单位为mm
	Eigen::Matrix<double, PER_IOS_BLOCK_NUM, 1> intrins;
};

struct BACamera
{
	BACamera() : imageid(-1), intrinsics_group(0) { }
	BACamera(const BACamera& c) :
		imageid(c.imageid), intrinsics_group(c.intrinsics_group),
		Location(c.Location), Posture(c.Posture),
		nStripID(c.nStripID), Attrib(c.Attrib), bFlag(c.bFlag),
		BlockID(c.BlockID), nCamera(c.nCamera) { }

	int imageid;
	int intrinsics_group; //内参数组别，适用于多相机,默认一个
	Eigen::Vector3d Location;
	Eigen::Vector3d Posture; //phi, omg, kap
	int nStripID, Attrib, bFlag, BlockID, nCamera; //iBundle接口
};


struct BoundingBoxCorner3D
{
	BoundingBoxCorner3D() { }
	BoundingBoxCorner3D(const BoundingBoxCorner3D& bdc):
		min_x(bdc.min_x), min_y(bdc.min_y), min_z(bdc.min_z),
		max_x(bdc.max_x), max_y(bdc.max_y), max_z(max_z){ }

	double min_x, min_y, min_z;
	double max_x, max_y, max_z;
};

struct ImageLasInfo
{
	ImageLasInfo() { }
	ImageLasInfo(const ImageLasInfo& ilm):
		lasList(ilm.lasList), rotate_(ilm.rotate_), translate_(ilm.translate_){ }

	std::vector<std::string> lasList;
	Eigen::Matrix3d rotate_;
	Eigen::Vector3d translate_;

};

#endif
