#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <System.h>

using namespace std;
bool b_continue_session = true;

void exit_handler(int s) {
    cout << "\nShutting down and saving trajectory..." << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {
    if(argc != 3) {
        cerr << "Usage: ./mono_realsense_D435i vocab settings" << endl;
        return 1;
    }

    signal(SIGINT, exit_handler);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    cout << endl << "-------" << endl;
    cout << "ORB-SLAM3 with D435i started!" << endl;
    cout << "Press Ctrl+C in terminal to quit and save trajectory." << endl;

    cv::Mat im;
    double tframe = 0.0;
    auto start = chrono::steady_clock::now();

    while(b_continue_session) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        if(!color) continue;

        im = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        auto now = chrono::steady_clock::now();
        tframe = chrono::duration_cast<chrono::duration<double>>(now - start).count();

        SLAM.TrackMonocular(im, tframe);
        cv::imshow("ORB-SLAM3: D435i", im);
        
        if(cv::waitKey(1) == 'q')
            break;
    }

    cout << "Saving trajectory..." << endl;
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout << "Saved to KeyFrameTrajectory.txt" << endl;
    return 0;
}
