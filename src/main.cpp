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
	fprintf(stderr, "[-lasPath]										[input the absolute path of las files corresponding to the photos\n");
	fprintf(stderr, "[-saveFolder]								[output the absolute path of range images\n");
	fprintf(stderr, "[-threadNum]								[the number of parallel processing threads\n");
	if (pszErrorMsg != NULL)
		fprintf(stderr, "\nFAILURE: %s\n", pszErrorMsg);

	exit(1);
}

int main(int argc, char** argv)
{
	std::string cmrFile_path = "", phtFile_path = "", lasFolder_Path = "", saveFolder_path = "";
	int thread_num = 4;

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
		else if (strcmp(argv[i], "-lasPath") == 0) {
			i++; if (i >= argc) continue;
			lasFolder_Path = argv[i];
		}
		else if (strcmp(argv[i], "-saveFolder") == 0) {
			i++; if (i >= argc) continue;
			saveFolder_path = argv[i];
		}
		else if (strcmp(argv[i], "-threadNum") == 0) {
			i++; if (i >= argc) continue;
			thread_num = std::atoi(argv[i]);
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

	RangeImageTool::Ptr range_image_tool(new RangeImageTool);
	range_image_tool->setCmrPath(cmrFile_path);
	range_image_tool->setPhtPath(phtFile_path);
	range_image_tool->setLasPath(lasFolder_Path);
	range_image_tool->setOutputFolder(saveFolder_path);
	range_image_tool->setThreadNum(thread_num);

	range_image_tool->generateRangeImage();

}
