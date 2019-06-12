//
// Created by prostoichelovek on 12.06.19.
//

#ifndef VISUALODOMETRY_WRAPPER_HPP
#define VISUALODOMETRY_WRAPPER_HPP


#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <iterator>
#include <vector>

#include "src/feature.h"
#include "src/utils.h"
#include "src/visualOdometry.h"

using namespace cv;
using namespace std;

class Wrapper {
public:
    float fx, fy, cx, cy, bf;
    Mat projMatrl, projMatrr;
    Mat rotation = Mat::eye(3, 3, CV_64F);
    Mat translation_stereo = Mat::zeros(3, 1, CV_64F);

    Mat pose = Mat::zeros(3, 1, CV_64F);
    Mat Rpose = Mat::eye(3, 3, CV_64F);

    Mat frame_pose = Mat::eye(4, 4, CV_64F);
    Mat frame_pose32 = Mat::eye(4, 4, CV_32F);

    FeatureSet currentVOFeatures;
    Mat points4D, points3D;

    Wrapper(float fx, float fy, float cx, float cy, float bf, Mat &projMatrl, Mat &projMatrr) :
            fx(fx), fy(fy), cx(cx), cy(cy), bf(bf), projMatrl(projMatrl), projMatrr(projMatrr) {

    }

    Point computePos(Mat &imageLeft, Mat &imageRight) {
        if (imageLeft_t0.cols == 0) {
            imageLeft_t0 = imageLeft;
            imageRight_t0 = imageRight;
            return Point(-1, -1);
        }

        vector<Point2f> oldPointsLeft_t0 = currentVOFeatures.points;
        vector<Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;
        matchingFeatures(imageLeft_t0, imageRight_t0,
                         imageLeft, imageRight,
                         currentVOFeatures,
                         pointsLeft_t0, pointsRight_t0,
                         pointsLeft_t1, pointsRight_t1);

        imageLeft_t0 = imageLeft;
        imageRight_t0 = imageRight;

        vector<Point2f> &currentPointsLeft_t0 = pointsLeft_t0;
        vector<Point2f> &currentPointsLeft_t1 = pointsLeft_t1;

        vector<Point2f> newPoints;
        vector<bool> valid; // valid new points are true

        // ---------------------
        // Triangulate 3D Points
        // ---------------------
        Mat points3D_t0, points4D_t0;
        triangulatePoints(projMatrl, projMatrr, pointsLeft_t0, pointsRight_t0, points4D_t0);
        convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

        Mat points3D_t1, points4D_t1;

        triangulatePoints(projMatrl, projMatrr, pointsLeft_t1, pointsRight_t1, points4D_t1);
        convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);

        // -----------------------
        // Tracking transformation
        // -----------------------
        trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation,
                            translation_stereo);
        displayTracking(imageLeft, pointsLeft_t0, pointsLeft_t1);


        points4D = points4D_t0;
        frame_pose.convertTo(frame_pose32, CV_32F);
        points4D = frame_pose32 * points4D;
        convertPointsFromHomogeneous(points4D.t(), points3D);

        // -----------
        // Integrating
        // -----------

        Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

        Mat rigid_body_transformation;

        if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1
            && abs(rotation_euler[2]) < 0.1) {
            integrateOdometryStereo(rigid_body_transformation, frame_pose, rotation,
                                    translation_stereo);
        }

        Rpose = frame_pose(Range(0, 3), Range(0, 3));
        Vec3f Rpose_euler = rotationMatrixToEulerAngles(Rpose);

        int x = int(frame_pose.col(3).at<double>(0));
        int y = int(frame_pose.col(3).at<double>(2));

        return Point(x, y);
    }

private:
    vector<FeaturePoint> oldFeaturePointsLeft;
    vector<FeaturePoint> currentFeaturePointsLeft;
    Mat imageLeft_t0, imageRight_t0;
};


#endif //VISUALODOMETRY_WRAPPER_HPP
