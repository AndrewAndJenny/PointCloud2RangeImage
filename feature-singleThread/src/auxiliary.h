#ifndef AUXILIARY_H
#define AUXILIARY_H

#include "lasreader.hpp"
#include"opencv2/core.hpp"
#include "DataStruct.h"
#include <iostream>

bool GetBoundingBox(std::string lasFilePath, BoundingBoxCorner3D& box);

void GetFiles(std::string path, std::string ext, std::vector<std::string>& files);

void matrixInverse(Eigen::MatrixXf& matrix);

#endif // AUXILIARY_H
