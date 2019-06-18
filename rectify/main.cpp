/*
 *  stereo_match.cpp
 *  calibration
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/videoio.hpp>


#include <string>
#include <sstream>
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    std::string intrinsic_filename = "intrinsics.yml";
    std::string extrinsic_filename = "extrinsics.yml";

    Mat frame_left;
    Mat frame_right;

    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap_left;
    VideoCapture cap_right;
    
    cap_left.open("/Users/jingpeilu/Desktop/research/data/left.avi");
    cap_right.open("/Users/jingpeilu/Desktop/research/data/right.avi");
    
    int i = 0;
    
    for (; ;)
    {
        // wait for a new frame from camera and store it into 'frame'
        
        cap_left.read(frame_left);
        cap_right.read(frame_right);
        // check if we succeeded
        if (frame_left.empty() || frame_right.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        Mat img1 = frame_left;
        Mat img2 = frame_right;
        
        Size img_size = img1.size();
        
        Rect roi1, roi2;
        Mat Q;
        
        if( !intrinsic_filename.empty() )
        {
            // reading intrinsic parameters
            FileStorage fs(intrinsic_filename, FileStorage::READ);
            if(!fs.isOpened())
            {
                printf("Failed to open file %s\n", intrinsic_filename.c_str());
                return -1;
            }
            
            Mat M1, D1, M2, D2;
            fs["M1"] >> M1;
            fs["D1"] >> D1;
            fs["M2"] >> M2;
            fs["D2"] >> D2;
            
            
            fs.open(extrinsic_filename, FileStorage::READ);
            if(!fs.isOpened())
            {
                printf("Failed to open file %s\n", extrinsic_filename.c_str());
                return -1;
            }
            
            Mat R, T, R1, P1, R2, P2;
            fs["R"] >> R;
            fs["T"] >> T;
            
            stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
            
            Mat map11, map12, map21, map22;
            initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
            initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
            
            Mat img1r, img2r;
            remap(img1, img1r, map11, map12, INTER_LINEAR);
            remap(img2, img2r, map21, map22, INTER_LINEAR);
            
            img1 = img1r;
            img2 = img2r;
            std::string filename1;
            std::string filename2;
            char nf[10];
            sprintf(nf, "%06d", i++);
            filename1 = string("/Users/jingpeilu/Desktop/research/data/left/") + nf + string(".png");
            //imwrite(filename1, img1);
            filename2 = string("/Users/jingpeilu/Desktop/research/data/right/") + nf + string(".png");

            //imwrite(filename2, img2);
        }
        
        
        
        // show live and wait for a key with timeout long enough to show images
        imshow("left", img1);
        imshow("right", img2);

        if (waitKey(5) >= 0)
            break;
    }

    return 0;
}


