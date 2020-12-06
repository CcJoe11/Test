#ifndef CLASSIFIER_NUM_H
#define CLASSIFIER_NUM_H
#include "opencv2/dnn/dnn.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<iostream>
using namespace cv;

class classifier_num
{
private:
    cv::dnn::Net LeNet;
    cv::Mat Armor;
//    Mat writem;
public:
    classifier_num()
    {
        Armor = cv::Mat();
        LeNet = cv::dnn::readNetFromCaffe("/home/nuc/chad/caffe/armor/state_zhiwu_sec/deploy_7.prototxt",                   //7class
            "/home/nuc/chad/caffe/armor/state_zhiwu_sec/green/armornet_solver_iter_15000.caffemodel");
//               "/home/nuc/chad/caffe/armor/state_zhiwu_sec/7class_lr0.01/armornet_solver_iter_10000.caffemodel");
    }
    bool isEight();
    void make_input_safe();
    int getNum(RotatedRect final_rect, Mat src);
};
//Mat logImage(Mat s, float logg = 1.0);

#endif // CLASSIFIER_NUM_H
