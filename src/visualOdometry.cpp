#include "visualOdometry.h"


void checkValidMatch(std::vector<cv::Point2f> &points, std::vector<cv::Point2f> &points_return,
                     std::vector<bool> &status,
                     int threshold) {
    int offset;
    for (int i = 0; i < points.size(); i++) {
        offset = std::max(std::abs(points[i].x - points_return[i].x), std::abs(points[i].y - points_return[i].y));
        // std::cout << offset << ", ";

        status.push_back(offset <= threshold);
    }
}

void removeInvalidPoints(std::vector<cv::Point2f> &points, const std::vector<bool> &status) {
    int index = 0;
    for (bool statu : status) {
        if (!statu) {
            points.erase(points.begin() + index);
        } else {
            index++;
        }
    }
}


void matchingFeatures(cv::Mat &imageLeft_t0, cv::Mat &imageRight_t0,
                      cv::Mat &imageLeft_t1, cv::Mat &imageRight_t1,
                      FeatureSet &currentVOFeatures,
                      std::vector<cv::Point2f> &pointsLeft_t0,
                      std::vector<cv::Point2f> &pointsRight_t0,
                      std::vector<cv::Point2f> &pointsLeft_t1,
                      std::vector<cv::Point2f> &pointsRight_t1) {
    // ----------------------------
    // Feature detection using FAST
    // ----------------------------
    std::vector<cv::Point2f> pointsLeftReturn_t0;   // feature points to check cicular mathcing validation


    if (currentVOFeatures.size() < 2000) {

        // append new features with old features
        appendNewFeatures(imageLeft_t0, currentVOFeatures);
        // std::cout << "Current feature set size: " << currentVOFeatures.points.size() << std::endl;
    }

    // --------------------------------------------------------
    // Feature tracking using KLT tracker, bucketing and circular matching
    // --------------------------------------------------------
    int bucket_size = 50;
    int features_per_bucket = 4;
    bucketingFeatures(imageLeft_t0, currentVOFeatures, bucket_size, features_per_bucket);

    pointsLeft_t0 = currentVOFeatures.points;

    circularMatching(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
                     pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1, pointsLeftReturn_t0,
                     currentVOFeatures);

    std::vector<bool> status;
    checkValidMatch(pointsLeft_t0, pointsLeftReturn_t0, status, 0);

    removeInvalidPoints(pointsLeft_t0, status);
    removeInvalidPoints(pointsLeft_t1, status);
    removeInvalidPoints(pointsRight_t0, status);
    removeInvalidPoints(pointsRight_t1, status);

    currentVOFeatures.points = pointsLeft_t1;

}


void trackingFrame2Frame(cv::Mat &projMatrl, cv::Mat &projMatrr,
                         std::vector<cv::Point2f> &pointsLeft_t0,
                         std::vector<cv::Point2f> &pointsLeft_t1,
                         cv::Mat &points3D_t0,
                         cv::Mat &rotation,
                         cv::Mat &translation) {

    // Calculate frame to frame transformation

    // -----------------------------------------------------------
    // Rotation(R) estimation using Nister's Five Points Algorithm
    // -----------------------------------------------------------
    double focal = projMatrl.at<float>(0, 0);
    cv::Point2d principle_point(projMatrl.at<float>(0, 2), projMatrl.at<float>(1, 2));

    //recovering the pose and the essential cv::matrix
    cv::Mat E, mask;
    cv::Mat translation_mono = cv::Mat::zeros(3, 1, CV_64F);
    E = cv::findEssentialMat(pointsLeft_t1, pointsLeft_t0, focal, principle_point, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, pointsLeft_t1, pointsLeft_t0, rotation, translation_mono, focal, principle_point, mask);
    // std::cout << "recoverPose rotation: " << rotation << std::endl;

    // ------------------------------------------------
    // Translation (t) estimation by use solvePnPRansac
    // ------------------------------------------------
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat inliers;
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0,
                                                                                                        1), projMatrl.at<float>(
            0, 2),
            projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
            projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));

    int iterationsCount = 500;        // number of Ransac iterations.
    float reprojectionError = 2.0;    // maximum allowed distance to consider it an inlier.
    float confidence = 0.95;          // RANSAC successful confidence.
    bool useExtrinsicGuess = true;
    int flags = cv::SOLVEPNP_ITERATIVE;

    cv::solvePnPRansac(points3D_t0, pointsLeft_t1, intrinsic_matrix, distCoeffs, rvec, translation,
                       useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                       inliers, flags);

    translation = -translation;
    // std::cout << "inliers size: " << inliers.size() << std::endl;

}

void displayTracking(cv::Mat &imageLeft_t1,
                     std::vector<cv::Point2f> &pointsLeft_t0,
                     std::vector<cv::Point2f> &pointsLeft_t1) {
    // -----------------------------------------
    // Display feature racking
    // -----------------------------------------
    int radius = 2;
    cv::Mat vis;

    cv::cvtColor(imageLeft_t1, vis, cv::COLOR_GRAY2BGR, 3);


    for (auto &i : pointsLeft_t0) {
        cv::circle(vis, cv::Point(i.x, i.y), radius, CV_RGB(0, 255, 0));
    }

    for (auto &i : pointsLeft_t1) {
        cv::circle(vis, cv::Point(i.x, i.y), radius, CV_RGB(255, 0, 0));
    }

    for (int i = 0; i < pointsLeft_t1.size(); i++) {
        cv::line(vis, pointsLeft_t0[i], pointsLeft_t1[i], CV_RGB(0, 255, 0));
    }

    cv::imshow("vis ", vis);
}


