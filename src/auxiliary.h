#ifndef AUXILIARY_H
#define AUXILIARY_H

#include "lasreader.hpp"
#include "Eigen/Geometry"
#include "DataStruct.h"
#include <iostream>
#include <boost/filesystem.hpp>

bool GetBoundingBox(std::string lasFilePath, BoundingBoxCorner3D& box);
void GetFiles(std::string path, std::string ext, std::vector<std::string>& files);
#endif // AUXILIARY_H
