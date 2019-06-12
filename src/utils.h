#ifndef UTILS_H
#define UTILS_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <cctype>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


#include "feature.h"


void display(cv::Mat &trajectory, const cv::Point2f &pose);



// --------------------------------
// Transformation
// --------------------------------
void integrateOdometryStereo(cv::Mat &rigid_body_transformation, cv::Mat &frame_pose, const cv::Mat &rotation,
                             const cv::Mat &translation_stereo);

bool isRotationMatrix(cv::Mat &R);

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);

// --------------------------------
// I/O
// --------------------------------

void loadImageLeft(cv::Mat& image_color, cv::Mat& image_gary, int frame_id, std::string filepath);

void loadImageRight(cv::Mat& image_color, cv::Mat& image_gary, int frame_id, std::string filepath);

// read time gyro txt file with format of timestamp, gx, gy, gz

#endif
