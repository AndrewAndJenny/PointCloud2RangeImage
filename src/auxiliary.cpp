#ifdef  _WIN32
#include<io.h>
#else defined linux
#include <sys/io.h>
#endif

#include "auxiliary.h"

bool GetBoundingBox(std::string lasFilePath, BoundingBoxCorner3D& box)
{
	LASreadOpener lasreadopener;
	LASheader las_header_read;
	LASreader *lasreader;

	// Open the LAS file
	lasreadopener.set_file_name(lasFilePath.c_str());

	if (!lasreadopener.active()) {
		std::cout << "ERROR: no input specified" << std::endl;
		return false;
	}

	lasreader = lasreadopener.open();
	if (!lasreader) {
		std::cerr << "ERROR: could not open LAS file: " << lasFilePath << std::endl;
		return false;
	}

	las_header_read = lasreader->header;
	las_header_read.user_data_after_header = nullptr;

	box.min_x = las_header_read.min_x;
	box.min_y = las_header_read.min_y;
	box.min_z = las_header_read.min_z;
	box.max_x = las_header_read.max_x;
	box.max_y = las_header_read.max_y;
	box.max_z = las_header_read.max_z;

	// Close the LAS file
	lasreader->close();

	return true;
}

void GetFiles(std::string path, std::string ext,std::vector<std::string>& files)
{
	long long hFile = 0;

	struct _finddata_t fileinfo;
	std::string tmp_path;
	if ((hFile = _findfirst(tmp_path.assign(path).append("/*.").append(ext).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if (fileinfo.attrib & _A_SUBDIR)
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					GetFiles(tmp_path.assign(path).append("/").append(fileinfo.name), ext,files);
			}
			else
			{
				files.push_back(tmp_path.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);

		_findclose(hFile);
	}
}