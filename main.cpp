#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "linux/videodev2.h"
#include<vector>
#include<iostream>
#include<math.h>
#include<sstream>
#include<armor_finder.h>
#include<rmvideocapture.h>
#include<anglesolver.hpp>
#include<armordetectpr.hpp>
#include<string>
#include<dnn.hpp>
#include"cameraapi.h"
#include"crc_check.h"
#include<classifier_num.h>
#include <serialport.h>
std::string Convert_main(float Num);
using namespace std;
using namespace cv;

unsigned char * g_pRgbBuffer;     //处理后数据缓存区

int main(int argc, char *argv[]){
    ///  串口的东西
    SerialPort port("/dev/ttyUSB0");
    port.initSerialPort();
    VisionData vdata;

    /// 相机的东西
    //////////////// Describe Appliance Info ////////////////////
    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     channel=3;
    ////////////////// Init Camera //////////////////////////////
    CameraSdkInit(1);
    ////////////////// Enum Devices /////////////////////////////
    iStatus = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    printf("Find camera count = %d\n", iCameraCounts);
    //没有连接设备
    if(iCameraCounts==0){
        return 0;
    }
    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
    if (iStatus != CAMERA_STATUS_SUCCESS){
        return 0;
    }else{
        cout << "Init Camera Succeeeded!" << endl;
    }
    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera,&tCapability);

    g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
    CameraSetAeState(hCamera, FALSE);
    ////////////////////// End ////////////////////////////////////////


    //////////////////////set param ///////////////////////////////////
    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    iStatus = CameraSetExposureTime(hCamera, EXPOSURE);
    if (iStatus != CAMERA_STATUS_SUCCESS)
        return 0;
        else cout << "Init SDK Succeeeded!" << endl;

    //////////////////// 设置分辨率 //////////////////////

    tSdkImageResolution pImageResolution = {0};
    pImageResolution.iIndex = 0xff;
    pImageResolution.iHeight = 720;
    pImageResolution.iHeightFOV = 720;
    pImageResolution.iVOffsetFOV = 0;
    pImageResolution.iHOffsetFOV = 0;
    pImageResolution.iWidth = 1280;
    pImageResolution.iWidthFOV = 1280;
    iStatus = CameraSetImageResolution(hCamera, &pImageResolution);

    /*
        设置图像处理的输出格式，彩色黑白都支持RGB24位
    */
    if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }


    while(1){
        if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS){
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);

            // 得到一张图片
            cv::Mat matImage(
                    cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),
                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                    g_pRgbBuffer);
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
            imshow("test",matImage);


        }
        // 处理


        // 处理之后
        float pitch,yaw,dis;
        vdata = {pitch,yaw,dis,0,1,0,0};
        port.TransformData(vdata);
        port.send();
    }

    // 反初始化再free
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);
}

Mat logImagee(Mat s, float logg = 1.0)
{
    float clog = logg;
    if(!s.empty())
    {
        float pixels[256];
        for(int i = 0;i<256;i++)
            pixels[i] = log(1+i);
        CV_Assert(s.channels()==3);//judge
        Mat image_log(s.size(),CV_32FC3);
//        s.convertTo(image_log,CV_32FC3);
        for(int i = 0;i<s.rows;i++)
        {
            for(int j = 0; j<s.cols;j++)
            {
                image_log.at<Vec3f>(i, j)[0] = clog*pixels[s.at<Vec3b>(i, j)[0]];
                image_log.at<Vec3f>(i, j)[1] = clog*pixels[s.at<Vec3b>(i, j)[1]];
                image_log.at<Vec3f>(i, j)[2] = clog*pixels[s.at<Vec3b>(i, j)[2]];
            }
        }
        normalize(image_log,image_log,0,255,CV_MINMAX);
        convertScaleAbs(image_log,image_log);
        return image_log;
    }
    std::cout << "log error!" << std::endl;
    return Mat();
}
#ifdef before
int main()
{
    VideoCapture capture;
    capture.open(
//                "/media/nuc/文件/ak.avi");
    "/media/nuc/文件/rp/num_src/train_src/green/NEW_NUM/videos/two.avi"); //improve,1/30 in 5, error;547
//    "/media/nuc/LBK/test.avi");
    double K[3][3] = {1381.0, 0, 682.5,
                      0, 1387.5976, 378.89,
                      0, 0, 1};
    double coeff[1][5] = {-0.4141,0.2561,0,0,0};
    Mat matrix(3,3,CV_64FC1,K);
    Mat cof(1,5,CV_64FC1,coeff);
    vector<Point2f> P;
    Mat s;
    ArmorDetector armor_d;
    armor_d.enemy_color = BLUE;

    while (1)
    {
//        cout << "111";
        P.clear();
        capture >> s;
//        Mat ss = imread("/media/nuc/文件/rp/num_src/train_src/1/one_5.jpg",IMREAD_COLOR);
        if (s.empty()) break;
        imshow("src",s);
//        Mat l = logImagee(s,0.8);
//        namedWindow("log",WINDOW_NORMAL);
//        imshow("log",l);
//        waitKey(0);
//        return 0;
        Mat rot,trans;
        double dis;
        Mat src1 = s.clone();
        GaussianBlur(src1, src1, Size(3, 3), 0);
//        armor_d.setImage(src1);
//        RotatedRect src_res = armor_d.getLastResult();
        vector<Point2f> target_P;
        RotatedRect RES;
        RES = armor_d.getTargetAera(src1, 0, 0);
//        imshow("armor",armor_d.getSrc()((Rect)armor_d._rect).clone());
        if(armor_d.isSamllArmor())
        {
//            RES = armor_d.getTargetAera(src1, 0, 0);
            AngleSolver slove(matrix, cof, 14, 5.5);
            slove.getTarget2dPoinstion(RES, target_P);
            slove.solvePnP4Points(target_P,rot,trans,dis);
        }
        else
        {
//            RES = armor_d.getTargetAera(src1, 0, 0);
            AngleSolver slove(matrix, cof, 22.5, 5.5);
            slove.getTarget2dPoinstion(RES, target_P);
            slove.solvePnP4Points(target_P,rot,trans,dis);
        }
        uint8_t anti = 1;
//        double x = trans.at<double>(0,0);
        ArmorFinder finder(anti);
        finder.set_target_box(armor_d);
        if(armor_d._rect.area()>0)
//        cout << "_rect: " << armor_d._rect.size() << endl;
        finder.antiTop();

//        cout << trans << endl << dis << endl;
        Mat out = armor_d.getSrc().clone();
        string temp1 = Convert_main((int)trans.at<double>(0,2));
        putText(out, temp1, Point(70,30), CV_FONT_NORMAL, 1, Scalar(255, 0, 0), 2);
//        src = src.addText(trans_red)  ;
//        imshow("src",src.getMat());

//        finder.set_last_box(armor_d);
//        Mat out = armor_d.getSrc().clone();
//        cout << finder._x << endl;
//        string temp = Convert_main(finder._x);
//        putText(out, temp, Point(30,30), CV_FONT_NORMAL, 1, Scalar(255, 0, 0), 2);
//        temp = Convert_main(finder._z);
//        putText(out, temp, Point(30,50), CV_FONT_NORMAL, 1, Scalar(255, 0, 0), 2);
        imshow("out", out);
//        int delay = 32;
//        waitKey(0);
//        if(delay>=0&&waitKey (delay)>=32)
//            waitKey(0);
//        else if(waitKey(delay)==27)
//            break;
        waitKey(1);
//        cout << endl;
    }
    return 0;
}
#endif
std::string Convert_main(float Num)
{
    ostringstream oss;
    oss<<Num;
    string str(oss.str());
    return str;
}
