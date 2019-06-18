

#include <iostream>
#include "elas.h"
#include "image.h"
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


Mat generateDisparityMap(Mat& left, Mat& right) {
    if (left.empty() || right.empty())
        return left;
    const Size imsize = left.size();
    const int32_t dims[3] = {imsize.width, imsize.height, imsize.width};
    Mat leftdpf = Mat::zeros(imsize, CV_32F);
    Mat rightdpf = Mat::zeros(imsize, CV_32F);
    Size out_img_size = Size(imsize.width, imsize.height);
    Elas::parameters param(Elas::MIDDLEBURY);
    param.postprocess_only_left = true;
    Elas elas(param);
    elas.process(left.data, right.data, leftdpf.ptr<float>(0), rightdpf.ptr<float>(0), dims);
    Mat dmap = Mat(out_img_size, CV_8UC1, Scalar(0));
    leftdpf.convertTo(dmap, CV_8UC1, 1.);
    return dmap;
}

static void cal_disp(Mat& img1,Mat& img2,Mat& img_out){
    
    Size imgSize = img1.size();
    int numberOfDisparities = ((imgSize.width / 8) + 15) & -16;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
    sgbm->setPreFilterCap(32);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = img1.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

    Mat disp, disp8;
    sgbm->compute(img1, img2, disp);
    disp = abs(disp);
    img_out = disp;
    
//    int SADWindowSize = 3;
//    int numberOfDisparities = 16;
//    float scale = 1;
//
//    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
//    if (scale != 1.f)
//    {
//        Mat temp1, temp2;
//        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
//        resize(img1, temp1, Size(), scale, scale, method);
//        img1 = temp1;
//        resize(img2, temp2, Size(), scale, scale, method);
//        img2 = temp2;
//    }
//
//    Size img_size = img1.size();
//
//    Rect roi1, roi2;
//    Mat Q;
//
//
//    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
//    //numberOfDisparities = ((img_size.width/8) + 15) & -16;
//    //sgbm->setPreFilterCap(63);
//    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
//    sgbm->setBlockSize(16);
//
//    int cn = img1.channels();
//    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
//    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
//    sgbm->setMinDisparity(5);
//    sgbm->setNumDisparities(numberOfDisparities);
//    sgbm->setUniquenessRatio(10);
//    sgbm->setSpeckleWindowSize(100);
//    sgbm->setSpeckleRange(32);
//    sgbm->setDisp12MaxDiff(1);
//
//    sgbm->setMode(StereoSGBM::MODE_SGBM);
//
//
//    Mat disp, disp8;
//    sgbm->compute(img1, img2, disp);
//    disp.convertTo(disp8, CV_8U);
//    img_out = disp8;
    
}
void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap)
{
    //int type = dispMap.type();
    
    float fx = 751.706129;
    //float fy = 766.315580;
    //float cx = 338.665467;
    //float cy = 257.986032;
    float baseline = 57; // 5.7cm
    
    if (1)
    {
        const float PI = 3.14159265358;
        int height = dispMap.rows;
        int width = dispMap.cols;
        
        uchar* dispData = (uchar*)dispMap.data;
        ushort* depthData = (ushort*)depthMap.data;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                int id = i*width + j;
                if (!dispData[id])  continue;  //防止0除
                depthData[id] = ushort( (float)fx *baseline*5 / ((float)dispData[id]) );
                //depthData[id] = 255;
            }
        }
    }
    else
    {
        cout << "please confirm dispImg's type!" << endl;
        cv::waitKey(0);
    }
}

void insertDepth32f(cv::Mat& depth)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = max(0, left);
                right = min(right, width - 1);
                top = max(0, top);
                bot = min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
}

int main (int argc, char** argv) {
    
    
    for (int n = 0; n < 600 ; n ++){
        
        std::string filename1;
        std::string filename2;
        char nf[10];
        sprintf(nf, "%06d", n);
        filename1 = string("/Users/jingpeilu/Desktop/research/data/left/") + nf + string(".png");
        filename2 = string("/Users/jingpeilu/Desktop/research/data/right/") + nf + string(".png");
        
        
        Mat disp;
        Mat img_save = imread(filename1, -1);
        Mat img1 = imread(filename1, 0);
        Mat img2 = imread(filename2, 0);
        disp = generateDisparityMap(img1, img2);
        //cal_disp(img1, img2, disp);
        //Apply bilateral filter
        Mat disp1;
        bilateralFilter ( disp, disp1, 13, 60, 60 );
        disp = disp1;

        
        //cout<< disp.type() <<endl;
        Mat depth = Mat::zeros(disp.size(), CV_16UC1);
        disp2Depth(disp,depth);
        //cout<< depth << endl;
        insertDepth32f(depth);
        //blur( depth, depth, Size( 11, 11 ), Point(-1,-1));
        GaussianBlur( depth, depth, Size( 11, 11 ), 15, 15 );
        //cout<< depth << endl;

        

        
        
        // Find the max for scaling the image colour
//        float disp_max = 0;
//        float disp_min = 50;
//
//        int width = disp.cols, height = disp.rows;
//        for (int i=0; i<height; i++)
//        {
//
//            for(int j = 0; j < width; j++){
//                //cout<< Mi[j]<< endl;
//                float intensity = disp.at<float>(i, j);
//                if (intensity>disp_max) {
//                    disp_max = intensity;
//                }
//                if (intensity<disp_min) {
//                    disp_min = intensity;
//                }
//            }
//        }
//        //cout<<disp_max<<endl;
//        //cout<<disp_min<<endl;
//
//        Mat depth = Mat::zeros(disp.size(), CV_32F);
//        int k = 0;
//        for (int i=0; i<height; i++)
//        {
//            for(int j = 0; j < width; j++){
//
//                float intensity = disp.at<float>(i, j);
//                if (intensity>disp_min){
//                    //k++;
//                    depth.at<float>(i, j) = 3500/intensity;
//                }
//            }
//        }
        cout <<n<<endl;
        string save_depth = string("/Users/jingpeilu/Desktop/research/data/depth1/") + nf + string("-depth.png");
        string save_color = string("/Users/jingpeilu/Desktop/research/data/depth1/") + nf + string("-color.png");
        imwrite(save_depth, depth);
//
//        //Mat depth = (500)/disp;
        imwrite(save_color, img_save);


    }
    //Mat a = imread("/Users/jingpeilu/Downloads/rgbd-scenes-v2/imgs/scene_01/00000-depth.png",-1);
    //cout<< a <<endl;



    
    return 0;
}



