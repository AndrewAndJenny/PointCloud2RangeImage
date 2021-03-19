#ifndef BACAMERAFILE_HPP_
#define BACAMERAFILE_HPP_

#include "DataStruct.h"
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <map>
#include <memory.h>

#undef sscanf_s
#define sscanf sscanf_s

class BACameraFile
{
public:
	// =====TYPEDEFS=====
	typedef std::shared_ptr<BACameraFile> Ptr;
	typedef std::shared_ptr<const BACameraFile> ConstPtr;

	// =====CONSTRUCTOR & DESTRUCTOR=====
	 /** Constructor */
	BACameraFile()
	{
		bcmrFile = false;
		bphtFile = false;
	};
	/** Destructor */
	~BACameraFile() { ReleaseData(); }
	
	// =====PUBLIC METHODS=====
	/** \brief Get a boost shared pointer of a copy of this */
	void LoadCameraData()
	{
		LoadCmr(cmrFile_path, m_intrinsics);
		LoadPht(phtFile_path, m_cameras);
	}

	void ReleaseData()
	{
		if (m_intrinsics.size() != 0)        m_intrinsics.~vector();
		if (m_cameras.size() != 0)           m_cameras.~vector();
	}

	bool LoadCmr(const std::string cmrIntrinsicFile, std::vector<BACameraIntrinsics> &all_intrinsics)
	{
		std::ifstream fp(cmrIntrinsicFile);
		if (!fp) {
			std::cout << "Couldn't find camera intrinsic file:" << cmrIntrinsicFile<<std::endl;
			return false;
		}
		std::string buf;
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);

		int ncmr;
		sscanf(buf.c_str(), "%d", &ncmr);
		all_intrinsics.resize(ncmr);

		double x0, y0, f, formatx, formaty, pixelsize, k0, k1, k2, k3, p1, p2, b1, b2;
		int cmridx, attrib;

		int tmp;
		for (int i = 0; i < ncmr; i++)
		{
			std::getline(fp, buf);
			tmp = sscanf(buf.c_str(), "%d%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%d", &cmridx,
				&x0, &y0, &f, &formatx, &formaty, &pixelsize,
				&k0, &k1, &k2, &k3, &p1, &p2, &b1, &b2, &attrib);

			if (tmp == 16) {
				all_intrinsics[i].intrins(OFFSET_F0) = f;
				all_intrinsics[i].intrins(OFFSET_X0) = x0;
				all_intrinsics[i].intrins(OFFSET_Y0) = y0;
				all_intrinsics[i].intrins(OFFSET_K1) = k1;
				all_intrinsics[i].intrins(OFFSET_K2) = k2;
				all_intrinsics[i].intrins(OFFSET_K3) = k3;
				all_intrinsics[i].intrins(OFFSET_P1) = p1;
				all_intrinsics[i].intrins(OFFSET_P2) = p2;
				all_intrinsics[i].intrins(OFFSET_B1) = b1;
				all_intrinsics[i].intrins(OFFSET_B2) = b2;
			}
			all_intrinsics[i].format_x = formatx;
			all_intrinsics[i].format_y = formaty;
			all_intrinsics[i].pixelsize = pixelsize;

		}

		bcmrFile = true;
		return true;
	}

	bool LoadPht(const std::string cameraFile, std::vector<BACamera> &all_cameras)
	{
		std::ifstream fp(cameraFile);
		if (!fp) {
			std::cout << "Couldn't find camera extrinsic file:" << cameraFile << std::endl;
			return false;
		}

		std::string buf;
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);
		std::getline(fp, buf);

		int nimg;
		sscanf(buf.c_str(), "%d", &nimg);

		all_cameras.resize(nimg);

		int ImageID, StripID, Attrib, CameraID, bFlag, BlockID;
		double Xs, Ys, Zs, phi, omega, kappa;

		for (int i = 0; i < nimg; i++)
		{
			std::getline(fp, buf);
			sscanf(buf.c_str(), "%d%lf%lf%lf%lf%lf%lf%d%d%d%d%d", &ImageID,
				&Xs, &Ys, &Zs, &phi, &omega, &kappa,
				&StripID, &Attrib, &CameraID, &bFlag, &BlockID);

			all_cameras[i].Location(EOS_XS) = Xs;
			all_cameras[i].Location(EOS_YS) = Ys;
			all_cameras[i].Location(EOS_ZS) = Zs;
			all_cameras[i].Posture(EOS_PHI) = phi;
			all_cameras[i].Posture(EOS_OMG) = omega;
			all_cameras[i].Posture(EOS_KAP) = kappa;
			all_cameras[i].imageid = i;
			all_cameras[i].intrinsics_group = CameraID;

			m_imageSortMap.insert(std::pair<int, int>(ImageID, i));  //便于pts快速索引

			all_cameras[i].nStripID = StripID;
			all_cameras[i].nCamera = CameraID;
			all_cameras[i].Attrib = Attrib;
			all_cameras[i].bFlag = bFlag;
			all_cameras[i].BlockID = BlockID;
		}

		bphtFile = true;
		return true;
	}

	std::string cmrFile_path;
	std::string phtFile_path;

	bool bcmrFile;
	bool bphtFile;

	std::vector<BACameraIntrinsics> m_intrinsics;
	std::vector<BACamera> m_cameras;

	std::map<int, int> m_imageSortMap;  //IBundle中imagID->imgid(从0开始)
};

#endif // BAFILE_HPP_
