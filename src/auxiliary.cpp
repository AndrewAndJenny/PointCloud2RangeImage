
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

void GetFiles(std::string path, std::string ext, std::vector<std::string>& files)
{
	boost::filesystem::path folder_path(path);
	if (!boost::filesystem::exists(folder_path))
	{
		return;
	}

	boost::filesystem::directory_iterator end_iter;
	for (boost::filesystem::directory_iterator iter(folder_path); iter != end_iter; ++iter)
	{
		if (boost::filesystem::is_regular_file(iter->status())&& iter->path().extension()==ext)
			files.push_back(iter->path().string());

		if (boost::filesystem::is_directory(iter->status()))
			GetFiles(iter->path().string(), ext,files);
	}

}