#include "utils.h"


void display(cv::Mat &trajectory, const cv::Point2f &pose) {
    circle(trajectory, cv::Point(pose.x + 300, pose.y + 100), 1, CV_RGB(255, 0, 0), 2);

    cv::imshow("Trajectory", trajectory);
    cv::waitKey(1);
}



// --------------------------------
// Transformation
// --------------------------------


void integrateOdometryStereo(cv::Mat &rigid_body_transformation, cv::Mat &frame_pose, const cv::Mat &rotation,
                             const cv::Mat &translation_stereo) {

    // std::cout << "rotation" << rotation << std::endl;
    // std::cout << "translation_stereo" << translation_stereo << std::endl;


    cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

    cv::hconcat(rotation, translation_stereo, rigid_body_transformation);
    cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

    // std::cout << "rigid_body_transformation" << rigid_body_transformation << std::endl;

    double scale = sqrt((translation_stereo.at<double>(0)) * (translation_stereo.at<double>(0))
                        + (translation_stereo.at<double>(1)) * (translation_stereo.at<double>(1))
                        + (translation_stereo.at<double>(2)) * (translation_stereo.at<double>(2)));

    // frame_pose = frame_pose * rigid_body_transformation;

    // rigid_body_transformation = rigid_body_transformation.inv();
    // if ((scale>0.1)&&(translation_stereo.at<double>(2) > translation_stereo.at<double>(0)) && (translation_stereo.at<double>(2) > translation_stereo.at<double>(1))) 
    if (scale > 0.05 && scale < 10) {
        // std::cout << "Rpose" << Rpose << std::endl;

        frame_pose = frame_pose * rigid_body_transformation;

    } else {
        std::cout << "[WARNING] scale below 0.1, or incorrect translation" << std::endl;
    }
}

bool isRotationMatrix(cv::Mat &R) {
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    return norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R) {

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2(-R.at<double>(2, 0), sy);
        z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);

}

// --------------------------------
// I/O
// --------------------------------

void loadImageLeft(cv::Mat &image_color, cv::Mat &image_gary, int frame_id, std::string filepath) {
    char file[200];
    sprintf(file, "image_0/%06d.png", frame_id);

    // sprintf(file, "image_0/%010d.png", frame_id);
    std::string filename = filepath + std::string(file);

    image_color = cv::imread(filename, cv::IMREAD_COLOR);
    cvtColor(image_color, image_gary, cv::COLOR_BGR2GRAY);
}

void loadImageRight(cv::Mat &image_color, cv::Mat &image_gary, int frame_id, std::string filepath) {
    char file[200];
    sprintf(file, "image_1/%06d.png", frame_id);

    // sprintf(file, "image_0/%010d.png", frame_id);
    std::string filename = filepath + std::string(file);

    image_color = cv::imread(filename, cv::IMREAD_COLOR);
    cvtColor(image_color, image_gary, cv::COLOR_BGR2GRAY);
}











