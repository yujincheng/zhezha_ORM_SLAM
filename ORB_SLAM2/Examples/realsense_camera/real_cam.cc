//#include <iostream>
//#include <stdio.h>
//#include <algorithm>

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <librealsense/rs.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>

//#include<System.h>
#include <unistd.h>
#include <sys/time.h>
#include <iomanip>

using namespace cv;
using namespace rs;


// Window size and frame rate
//int const INPUT_WIDTH      = 320;
//int const INPUT_HEIGHT     = 240;
//int const FRAMERATE        = 60;
int const INPUT_WIDTH      = 640;
int const INPUT_HEIGHT     = 480;
int const FRAMERATE        = 30;

// Named windows
char* const WINDOW_DEPTH = "Depth Image";
char* const WINDOW_RGB     = "RGB Image";


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
             _rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
             _rs_camera.start( );

             success = true;
       }
       return success;
}


int main(int argc, char * argv[]){
	struct timeval tv;
	//struct timezone tz;
	std::string s1;
	std::string s2,s;
	double d;
	std::cout << "hello realsense camera!" << std::endl;
	cv::Mat im;
	im = cv::imread("/media/nvidia/ZJU/orbslam2/TUM/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png",CV_LOAD_IMAGE_UNCHANGED);
	cv::namedWindow("image", WINDOW_AUTOSIZE);
	cv::imshow("image",im);
	cv::waitKey();
	rs::log_to_console( rs::log_severity::warn );

       if( !initialize_streaming( ) )
       {
             std::cout << "Unable to locate a camera" << std::endl;
             rs::log_to_console( rs::log_severity::fatal );
             return EXIT_FAILURE;
       }
	
	//display cam pictures
	while(true){
	 if( _rs_camera.is_streaming( ) ){
                    _rs_camera.wait_for_frames( );
		   //debug chenjp 1 row
		   //cout << "is streaming!" << endl;
		}
	_color_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::color );
	_depth_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::depth );
	// Create color image
        //im = cv::Mat( _color_intrin.height,
        //                    _color_intrin.width,
        //                    CV_8UC3,
        //                    (uchar *)_rs_camera.get_frame_data( rs::stream::color ) );

	// create depth image
	im = cv::Mat( _depth_intrin.height,
                                  _depth_intrin.width,
                                  CV_16U,
                                  (uchar *)_rs_camera.get_frame_data( rs::stream::depth ) );
	d = double(im.at<Vec3b>(100,100)[0]);
	std::cout << d << std::endl;
	//cv::cvtColor(im,im,cv::COLOR_BGR2RGB);
	//im = rgb_img.clone();
	//cv::imshow("image",im);
	//gettimeofday(&tv,NULL);
	//s1 = std::to_string(tv.tv_sec);
	//s2 = std::to_string(tv.tv_usec);
	//s = s1 + "." + s2;
	//std::cout << "string s:" << s << std::endl;
	//d = std::stod(s);
	//std::cout << "d" << std::setprecision(16) << d << std::endl;
	//std::cout << tv.tv_sec << "." << tv.tv_usec << std::endl;
	//std::cout << std::setprecision(16) <<tv.tv_sec+tv.tv_usec*10e-6 << std::endl;
	if(cv::waitKey(1) == 'q')
		break;
	}
	
	return(0);
}
