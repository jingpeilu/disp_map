/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "DataExporter.hpp"
#include "opencv2/core/core.hpp"

#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;


static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
           "[--no-display] [-o=<disparity_image>] [-p=<point_cloud_file>]\n");
}

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            //if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

static void cal_disp(Mat& img1,Mat& img2,Mat& img_out){
    
    int SADWindowSize = 3;
    int numberOfDisparities = 16;
    float scale = 0.5;

    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }
    
    Size img_size = img1.size();
    
    Rect roi1, roi2;
    Mat Q;
    

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
    
    //sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(16);
    
    int cn = img1.channels();
    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(5);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    sgbm->setMode(StereoSGBM::MODE_SGBM);

    
    Mat disp, disp8;
    sgbm->compute(img1, img2, disp);
    img_out = disp;


    
    
}

int main(int argc, char** argv)
{
    
    cv::CommandLineParser parser(argc, argv,
                                 "{@arg1|/Users/jingpeilu/Desktop/research/daVinci/test/image_0/000715.png|}{@arg2|/Users/jingpeilu/Desktop/research/daVinci/test/image_1/000715.png|}{help h||}{algorithm|sgbm|}{max-disparity|16|}{blocksize|3|}{no-display|false|}{scale|0.5|}{i||}{e||}{o|aa.jpg|}{p|p.txt|}");
    
    
    Mat img1 = imread("/Users/jingpeilu/Desktop/research/daVinci/test/image_0/000715.png", -1);
    Mat img2 = imread("/Users/jingpeilu/Desktop/research/daVinci/test/image_1/000715.png", -1);
    
    if (img1.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (img2.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }
    Mat disp, disp8;
    cal_disp(img1, img2, disp);
    
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    
    cout<< disp;
    imwrite("b.jpg", disp);
    

    
    return 0;
}

