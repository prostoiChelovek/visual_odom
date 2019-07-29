#include <opencv2/opencv.hpp>

#include "Wrapper.hpp"

using namespace std;

int main(int argc, char **argv) {

    // -----------------------------------------
    // Load images and calibration parameters
    // -----------------------------------------
    if (argc < 3) {
        cerr << "Usage: ./run path_to_sequence path_to_calibration [optional]path_to_ground_truth_pose" << endl;
        return 1;
    }

    // Sequence
    string filepath = string(argv[1]);
    cout << "Filepath: " << filepath << endl;

    // Camera calibration
    string strSettingPath = string(argv[2]);
    cout << "Calibration Filepath: " << strSettingPath << endl;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    float bf = fSettings["Camera.bf"];

    cv::Mat projMatrl = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0, 0., 1., 0.);
    cv::Mat projMatrr = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0, 0., 1., 0.);

    cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    int init_frame_id = 0;

    float fps;

    // -----------------------------------------
    // Run visual odometry
    // -----------------------------------------
    clock_t tic = clock();

    Wrapper wp(projMatrl, projMatrr);

    for (int frame_id = init_frame_id + 1; frame_id < 9000; frame_id++) {
        // ------------
        // Load images
        // ------------
        cv::Mat imageLeft_color, imageLeft;
        loadImageLeft(imageLeft_color, imageLeft, frame_id, filepath);
        cv::Mat imageRight_color, imageRight;
        loadImageRight(imageRight_color, imageRight, frame_id, filepath);

        Point pos = wp.computePos(imageLeft, imageRight);

        clock_t toc = clock();
        fps = float(frame_id - init_frame_id) / (toc - tic) * CLOCKS_PER_SEC;

        std::cout << "FPS: " << fps << std::endl;

        displayTracking(imageLeft, wp.pointsLeft_t0, wp.pointsLeft_t1);
        display(trajectory, pos);
    }
    return EXIT_SUCCESS;
}