#include "opencv2/core/core.hpp"
#include <armordetectpr.hpp>
#include <roundqueue.h>
#include<system_time.h>
#include<iostream>
#include<armor_finder.h>
//#include
#define PI 3.1415925
#define IMAGE_CENTER_X 640
#define FOCUS_PIXAL_8MM  (1488)
#define FOCUS_PIXAL_5MM  (917)
#define FOCUS_PIXAL      FOCUS_PIXAL_5MM
#define angle_range PI*2/3 //hit's range
#define tens_angle_range angle_range/10
#define k angle_range*2/PI
using namespace std;
using namespace cv;
long deltaT_close[30] = {0}; //1-4m , <1m,sui_bian_da?
long deltaT_far[30] = {0};   //4-7m
double _angle[10] = {0, tens_angle_range, tens_angle_range*2, tens_angle_range*3, tens_angle_range*4, tens_angle_range*5, tens_angle_range*6, tens_angle_range*7,tens_angle_range*8, tens_angle_range*9};
template<int length>
static double mean(RoundQueue<double, length> &vec) {
    double sum = 0;
    if(vec.size() > 8 ) // 8 need to be changed
    {
        vec.pop(vec[vec.getHead()]);
    }
    for (int i = 0; i < vec.size(); i++) {
        sum += vec[i];
    }
    return sum / length;
}

static systime getFrontTime(const vector<systime> time_seq, const vector<float> angle_seq) {
    double A = 0, B = 0, C = 0, D = 0;
    int len = time_seq.size();
    for (int i = 0; i < len; i++) {
        A += angle_seq[i] * angle_seq[i];
        B += angle_seq[i];
        C += angle_seq[i] * time_seq[i];
        D += time_seq[i];
        cout << "(" << angle_seq[i] << ", " << time_seq[i] << ") ";
    }
    double b = (A * D - B * C) / (len * A - B * B);
//    cout << b << endl;
    return b;
}

double getPointLength(const cv::Point2f &p) {
    return sqrt(p.x * p.x + p.y * p.y);
}

void ArmorFinder::antiTop() //no rotatedRect
{/*
    //debug
    if(!target_box._rect.size().empty())
        cout << "target_rect_size:  " << target_box._rect.size() << endl;*/

    if (target_box._rect.empty()) return;
    if (getPointLength(last_box.getCenter() - target_box.getCenter()) > last_box._rect.height * 1.5)  //change armor
    {
        systime star_time, curr_time;
        getsystime(star_time);
        systime front_time = getFrontTime(time_seq, angle_seq);
        systime once_periodms = getTimeIntervalms(front_time, last_front_time); // return 时间差
        //LOGM(STR_CTR(WORD_GREEN, "Top period: %.1lf"), once_periodms);
        top_periodms.push(once_periodms); // once_periodms --- T/4
        systime periodms = mean(top_periodms);// top_periodms---陀螺周期循环队列 , periodms 为T/4的平均值
        systime _T = periodms * k;
        getsystime(curr_time);//now_time
//        uint16_t shoot_delay = front_time + periodms * 2 - curr_time;
        long deltaT, _deltaT;
        double next_angle, org_angle, final_angle;
        int cur = 0, _cur = 4; //curr_time = front_time,_cur = 4
        _deltaT = curr_time - front_time;
        while(_deltaT > 0)
        {
            _deltaT -= _T/10;
            ++_cur;
        }
        _cur /= 10;
        org_angle = _angle[_cur];
        if (anti_top_cnt < 4) // hit middle
        {
            set_x_z(angle_range/2);
        }
         else if (abs(once_periodms - top_periodms[-1]) > 50) //angle = 0
        {
            set_x_z(angle_range/2);
        }
        else
        {
            int dis = target_box.getIntDistance();
            if(dis < 400)
                deltaT = deltaT_close[dis/10-10];
            else
                deltaT = deltaT_far[dis/10-40];
//            deltaT %= _T, _deltaT %= _T;
            while(deltaT >= _T)
            {
                deltaT -= _T;
            }
            while(_deltaT >= _T)
            {
                _deltaT -= _T;
            }
            while(deltaT > 0)
            {
                deltaT -= _T/10;
                ++cur;
            }
            cur /= 10;
            next_angle = _angle[cur];
            final_angle = next_angle + org_angle;
            while(final_angle >= angle_range)
            {
                final_angle -= angle_range;
            }
            set_x_z(final_angle);
        }
        time_seq.clear();
        angle_seq.clear();
        last_front_time = front_time;
        last_box = target_box;

    }
    else
    {
        time_seq.push_back(frame_time);
        double dx = target_box._rect.x + target_box._rect.width / 2 - IMAGE_CENTER_X;
        double yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
        angle_seq.push_back(yaw);
    }
    anti_top_cnt++;
}
