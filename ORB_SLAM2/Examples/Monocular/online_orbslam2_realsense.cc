/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <unistd.h>

#include <librealsense/rs.hpp>
#include <sys/time.h>
using namespace std;

/////////////////////////////// code added /////////////////////////////

using namespace cv;
using namespace rs;
// Window size and frame rate
int const INPUT_WIDTH      = 640;
int const INPUT_HEIGHT     = 480;
int const FRAMERATE        = 60;


context      _rs_ctx;
device&      _rs_camera = *_rs_ctx.get_device( 0 );
intrinsics   _depth_intrin;
intrinsics  _color_intrin;
bool         _loop = true;


// Initialize the application state. Upon success will return the static app_state vars address

bool initialize_streaming( )
{
       bool success = false;
       if( _rs_ctx.get_device_count( ) > 0 )
       {
             _rs_camera.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
             //_rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
             _rs_camera.start( );

             success = true;
       }
       return success;
}

///////////////////////////////////////////////////// origin code ///////////////////////


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./online_orbslam2_realsense_monocular path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing frame stream ..." << endl;

    //// Main loop
    //camera initialization
    rs::log_to_console( rs::log_severity::warn );
    if( !initialize_streaming( ) )
    {
     std::cout << "Unable to locate a camera" << std::endl;
     rs::log_to_console( rs::log_severity::fatal );
     return EXIT_FAILURE;
    }
    cv::Mat im;
    struct timeval tv;
    std::string s1,s2,s;
    double d;
    std::cout << "press q to quit!" << std::endl;
    while(true) //for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        //im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe; // = vTimestamps[ni];
	if( _rs_camera.is_streaming( ) ){
                    _rs_camera.wait_for_frames( );
		   //debug chenjp 1 row
		   //cout << "is streaming!" << endl;
		}
	_color_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::color );
	
	// Create color image
        im = cv::Mat( _color_intrin.height,
                            _color_intrin.width,
                            CV_8UC3,
                            (uchar *)_rs_camera.get_frame_data( rs::stream::color ) );
	cv::cvtColor(im,im,cv::COLOR_BGR2RGB);
	
	//create time stamp
	gettimeofday(&tv,NULL);
	s1 = std::to_string(tv.tv_sec);
	s2 = std::to_string(tv.tv_usec);
	s = s1 + "." + s2;
	d = std::stod(s);
	tframe = d;
	//debug chenjp 2rows
	//cout << "file  names:" << vstrImageFilenames[ni] << endl;
	//cout << "time stamps:" << vTimestamps[ni] << endl;
	
        if(im.empty())
        {
            cerr << endl << "Failed to load image from camera stream!" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // Wait to load the next frame
        //double T=0;
        //if(ni<nImages-1)
        //    T = vTimestamps[ni+1]-tframe;
        //else if(ni>0)
        //    T = tframe-vTimestamps[ni-1];

        //if(ttrack<T)
         //   usleep((T-ttrack)*1e6);
	if(cv::waitKey(1) == 'q'){break;}
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    int nImages = vTimesTrack.size();
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}


