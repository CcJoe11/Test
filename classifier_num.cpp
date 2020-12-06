#include "classifier_num.h"
#include <opencv2/opencv.hpp>
#include "sstream"
#include <vector>
//#define log
//#define NO_RED
using namespace cv;
std::string name = "two_";
std::string str = "/media/nuc/文件/rp/num_src/wrong/green/2/";
bool makeRectSafe(cv::Rect & rect, cv::Size size){
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        // 如果发现矩形是空的，则返回false
        return false;
    return true;
}
bool classifier_num::isEight()
{
    // ***************judge is eight or not first********************
    Mat Armor_bin;
    cvtColor(Armor,Armor_bin,CV_BGR2GRAY);
    threshold(Armor_bin, Armor_bin, 30, 255, CV_THRESH_BINARY);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    erode(Armor_bin,Armor_bin,Mat());
    dilate(Armor_bin,Armor_bin,kernel);

    std::vector<Vec4i> hierarchy;
    std::vector<std::vector<Point2i> > contours_max;
    findContours(Armor_bin, contours_max, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE);
//    std::cout << "(int)contours_max.size():  " << (int)contours_max.size() << std::endl;
    if(hierarchy.size() == 3 && hierarchy[1][3] == 0 && hierarchy[2][3] == 0)
        return true;
    return false;
}

Mat logImagee(Mat ss, float logg = 1.0)
{
    float clog = logg;
    if(!ss.empty())
    {
        float pixels[256];
        for(int i = 0;i<256;i++)
            pixels[i] = log(1+i);
        CV_Assert(ss.channels()==3);//judge
        Mat image_log(ss.size(),CV_32FC3);
//        s.convertTo(image_log,CV_32FC3);
        for(int i = 0;i<ss.rows;i++)
        {
            for(int j = 0; j<ss.cols;j++)
            {
                image_log.at<Vec3f>(i, j)[0] = clog*pixels[ss.at<Vec3b>(i, j)[0]];
                image_log.at<Vec3f>(i, j)[1] = clog*pixels[ss.at<Vec3b>(i, j)[1]];
                image_log.at<Vec3f>(i, j)[2] = clog*pixels[ss.at<Vec3b>(i, j)[2]];
//                std::cout << "i j:  " << i << "   " << j << ":  " <<image_log.at<Vec3f>(i, j)[0] << "  " << image_log.at<Vec3f>(i, j)[1] << "   " << image_log.at<Vec3f>(i, j)[2] << std::endl;
            }
        }
        normalize(image_log,image_log,0,255,CV_MINMAX);
        convertScaleAbs(image_log,image_log);
        return image_log;
    }
    std::cout << "log error!" << std::endl;
    return Mat();
}

Mat change_red(Mat ss)
{
    if(!ss.empty())
    {
        CV_Assert(ss.channels()==3);//judge
        cvtColor(ss, ss, CV_BGR2RGB);
        namedWindow("before_red",WINDOW_NORMAL);
        imshow("before_red", ss);
        Mat image_red(ss.size(),CV_32FC3);
//        s.convertTo(image_red,CV_32FC3);
//        vector<Mat> splited;
//        split(ss,splited);
        for(int i = 0;i<ss.rows;i++)
        {
            for(int j = 0; j<ss.cols;j++)
            {
                image_red.at<Vec3f>(i, j)[0] = ss.at<Vec3f>(i, j)[0] > 230 ? 0 : image_red.at<Vec3f>(i, j)[0];
                image_red.at<Vec3f>(i, j)[1] = ss.at<Vec3b>(i, j)[1];
                image_red.at<Vec3f>(i, j)[2] = ss.at<Vec3b>(i, j)[2];
//                std::cout << "i j:  " << i << "   " << j << ":  " <<image_red.at<Vec3f>(i, j)[0] << "  " << image_red.at<Vec3f>(i, j)[1] << "   " << image_red.at<Vec3f>(i, j)[2] << std::endl;
            }
        }
        normalize(image_red,image_red,0,255,CV_MINMAX);
        convertScaleAbs(image_red,image_red);
        namedWindow("delete_red",WINDOW_NORMAL);
        imshow("delete_red", image_red);
        return image_red;
    }
    std::cout << "log error!" << std::endl;
    return Mat();
}

void classifier_num::make_input_safe()
{
//    Rect dect_rect;
//    std::cout << Armor.cols << ' ' << Armor.rows << std::endl;
//    if(Armor.cols > Armor.rows)
//        dect_rect = Rect((Armor.cols-Armor.rows)/2, 0,Armor.rows, Armor.rows);
//    else
//        dect_rect = Rect(0, 0, Armor.cols, Armor.rows);
//    makeRectSafe(dect_rect, Armor.size());
//    Armor = Armor(dect_rect);
    if(Armor.empty())
        std::cout << "Armor is empty!!!!!" << std::endl;
    namedWindow("armor1",WINDOW_NORMAL);
    imshow("armor1",Armor);
//    waitKey(0);
    Mat input = Armor.clone();

#ifdef log
    resize(input,input,Size(28,28),INTER_LINEAR);
    input = logImagee(input,1);
    namedWindow("log",WINDOW_NORMAL);
    imshow("log",input);
#endif
#ifdef NO_RED
    input = change_red(input);
#endif
    Mat inputBlob = dnn::blobFromImage(input,0.00390625f, Size(28, 28), Scalar(), false); //load image
    LeNet.setInput(inputBlob,"data");
//    std::cout << "make_input_safe ok!" << std::endl;
}
//#ifdef
std::string Convert1(int Num)
{
    std::ostringstream oss;
    oss<<Num;
    std::string str(oss.str());
    return str;
}
//#endif
int countnn = 0;

int classifier_num::getNum(RotatedRect final_rect, Mat src)
{
    Rect rect = final_rect.boundingRect();

    Point2f p1(final_rect.center.x - rect.width/2*0.7, final_rect.center.y - rect.width/2*1.2);
    Point2f p2(final_rect.center.x + rect.width/2*0.7, final_rect.center.y + rect.width/2);
    Rect res1(p1,p2);
    makeRectSafe(res1, src.size());
    Mat NUM = src(res1);
//    imshow("NUM",NUM);
    Armor = src(res1);
//    namedWindow("Armor",WINDOW_NORMAL);
//    imshow("Armor",Armor);
    if(Armor.empty())
    {
        std::cout << "armor is empty!" << std::endl;
    }
    if(isEight())
        return 8;

    make_input_safe();

    Mat prob = LeNet.forward("prob");
    Mat probMat = prob.reshape(1,1);
    double classProb;
    Point classNumber;
    minMaxLoc(probMat, nullptr, &classProb, nullptr, &classNumber);

    // imwrite wrong photo
    if(classNumber.x != 2)
    {
        std::string tmp = name;
        std::string tmp1 = Convert1(countnn);
        std::string tmp2 = Convert1(classNumber.x);
        tmp += tmp1;
        countnn++;
//        std::cout << "Armor.size:  " << Armor.size << std::endl;
        namedWindow("writed",WINDOW_NORMAL);
        imshow("writed",Armor);
        waitKey(0);
        std::cout << tmp << std::endl;
        imwrite(str + tmp + ".jpg", Armor);
    }
    std::cout << classProb*100 << "%" << std::endl;
    std::cout << "real_num:  " << classNumber.x << std::endl;
    int out;
    if(classNumber.x == 6)
        out = 1;
    else
        out = classNumber.x;
    return out;
}
