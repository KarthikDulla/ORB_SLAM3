#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_realsense_D435i path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // RealSense pipeline
    rs2::pipeline pipe;
    rs2::config cfg;

    // USB 2.1 compatible settings: 640x480 @ 15fps
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);

    rs2::pipeline_profile profile = pipe.start(cfg);

    // Get depth scale
    rs2::device dev = profile.get_device();
    rs2::depth_sensor ds = dev.first<rs2::depth_sensor>();
    float depth_scale = ds.get_depth_scale();

    // Align depth to color
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Press 'q' in the image window to quit." << endl;

    double tframe = 0.0;
    auto start = chrono::steady_clock::now();

    while(true)
    {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        rs2::video_frame color_frame = frameset.get_color_frame();
        rs2::depth_frame depth_frame = frameset.get_depth_frame();

        if(!color_frame || !depth_frame)
        {
            cerr << "Missing frame!" << endl;
            continue;
        }

        // Convert to OpenCV
        cv::Mat im(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        auto now = chrono::steady_clock::now();
        tframe = chrono::duration_cast<chrono::duration<double>>(now - start).count();

        SLAM.TrackRGBD(im, depth, tframe);

        cv::imshow("ORB-SLAM3: Current Frame", im);
        
        if(cv::waitKey(1) == 'q')
            break;
    }

    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    cout << endl << "Trajectory saved to CameraTrajectory.txt" << endl;

    return 0;
}
