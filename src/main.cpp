#ifdef  _WIN32
#include<io.h>
#include<direct.h>
#else defined linux
#include <sys/io.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include "range_image_tool.h"

static void Usage(const char* pszErrorMsg = NULL)
{
	fprintf(stderr, "Usage:\n");
	fprintf(stderr, "RangeImageTool [-cmrPath] *.txt [-phtPath] *,txt [-lasPath] folderPath\n");
	fprintf(stderr, "options:\n");
	fprintf(stderr, "[-help,-h]										[produce help message]\n");
	fprintf(stderr, "[-cmrPath]									[input the absolute path of camera intrinsics file]\n");
	fprintf(stderr, "[-phtPath]									[input the absolute path of camera extrinsic file]\n");
	fprintf(stderr, "[-maxEle]									[Maximum elevation of saved point cloud]\n");
	fprintf(stderr, "[-gridNum]									[Dense image interpolate windows radius(sugguest 1-4) too big cause memory overflow]\n");
	fprintf(stderr, "[-lasPath]										[input the absolute path of las files corresponding to the photos\n");
	fprintf(stderr, "[-saveFolder]								[output the absolute path of range images\n");
	if (pszErrorMsg != NULL)
		fprintf(stderr, "\nFAILURE: %s\n", pszErrorMsg);

	exit(1);
}

int main(int argc, char** argv)
{
	std::string cmrFile_path = "", phtFile_path = "", lasFolder_Path = "", saveFolder_path = "";
	int ngrid = 2;
	double max_elevation = 150;

	for (int i = 1; i < argc; i++)
	{
		if (strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0)
		{
			Usage();
		}
		else if (strcmp(argv[i], "-cmrPath") == 0)
		{
			i++; if (i >= argc) continue;
			cmrFile_path = argv[i];
		}
		else if (strcmp(argv[i], "-phtPath") == 0) {
			i++; if (i >= argc) continue;
			phtFile_path = argv[i];
		}
		else if (strcmp(argv[i], "-maxEle") == 0) {
			i++; if (i >= argc) continue;
			max_elevation = atof(argv[i]);
		}
		else if (strcmp(argv[i], "-gridNum") == 0) {
			i++; if (i >= argc) continue;
			ngrid = atoi(argv[i]);
		}
		else if (strcmp(argv[i], "-lasPath") == 0) {
			i++; if (i >= argc) continue;
			lasFolder_Path = argv[i];
		}
		else if (strcmp(argv[i], "-saveFolder") == 0) {
			i++; if (i >= argc) continue;
			saveFolder_path = argv[i];
		}
		else
		{
			Usage("Too many command options.");
		}
	}
#ifdef _WIN32
	if (0 != _access(lasFolder_Path.c_str(), 0))
	{
		std::cout << lasFolder_Path << " isn't exist!\n";
		exit(1);
	}

	if (0 != _access(saveFolder_path.c_str(), 0))
		_mkdir(saveFolder_path.c_str());

#else defined linux
	if (0 != eaccess(lasFolder_Path.c_str(), F_OK))
	{
		std::cout << lasFolder_Path << " isn't exist!\n";
		exit(1);
	}

	if (0 != eaccess(saveFolder_path.c_str(), F_OK))
		int flag = mkdir(saveFolder_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);

#endif

	RangeImageTool::Ptr pc_rangeImage(new RangeImageTool);
	pc_rangeImage->setCmrPath(cmrFile_path);
	pc_rangeImage->setPhtPath(phtFile_path);
	pc_rangeImage->setMaxElevation(max_elevation);
	pc_rangeImage->setGridNum(ngrid);

	pc_rangeImage->setLasPath(lasFolder_Path);
	pc_rangeImage->setOutputFolder(saveFolder_path);

	pc_rangeImage->generateRangeImage();

}
