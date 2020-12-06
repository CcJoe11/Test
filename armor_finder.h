#include<system_time.h>
#include<armordetectpr.hpp>
#include<roundqueue.h>
using namespace std;

/********************* 自瞄类定义 **********************/
class ArmorFinder{
public:
    ArmorFinder(const uint8_t &anti_top);
//    ArmorFinder(/*Serial &u,*/ const string &paras_folder, const uint8_t &anti_top);
//    ~ArmorFinder() = default;
    float _x, _z;                                        // x,z change
//    float r;
private:
//    typedef cv::TrackerKCF TrackerToUse;                // Tracker类型定义

/*    typedef enum{
        SEARCHING_STATE, TRACKING_STATE, STANDBY_STATE
    } State;                                     */       // 自瞄状态枚举定义

    systime frame_time;                                 // 当前帧对应时间
//    const uint8_t &enemy_color;                         // 敌方颜色，引用外部变量，自动变化
    const uint8_t &is_anti_top;                         // 进入反陀螺，引用外部变量，自动变化
//    State state;                                        // 自瞄状态对象实例
    ArmorDetector target_box, last_box;                      // 目标装甲板
    int anti_switch_cnt;                                // 防止乱切目标计数器
//    cv::Ptr<cv::Tracker> tracker;                       // tracker对象实例
//    Classifier classifier;                              // CNN分类器对象实例，用于数字识别
    int contour_area;                                   // 装甲区域亮点个数，用于数字识别未启用时判断是否跟丢（已弃用）
    int tracking_cnt;                                   // 记录追踪帧数，用于定时退出追踪
//    Serial &serial;                                     // 串口对象，引用外部变量，用于和能量机关共享同一个变量
    systime last_front_time;                            // 上次陀螺正对时间
    int anti_top_cnt;
    RoundQueue<double, 4> top_periodms;                 // 陀螺周期循环队列
//    RoundQueue<int, 10> top_angle;
    vector<systime> time_seq;                           // 一个周期内的时间采样点
    vector<float> angle_seq;                            // 一个周期内的角度采样点

//    bool findLightBlobs(const cv::Mat &src, LightBlobs &light_blobs);
//    bool findArmorBox(const cv::Mat &src, ArmorBox &box);
//    bool matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes);

//    bool stateSearchingTarget(cv::Mat &src);            // searching state主函数
//    bool stateTrackingTarget(cv::Mat &src);             // tracking state主函数
//    bool stateStandBy();                                // stand by state主函数（已弃用）
    void set_x_z(double angle);                         //set_pian_yi_xz

public:
    void antiTop();                                     // 反小陀螺
    bool sendBoxPosition(uint16_t shoot);               // 和主控板通讯
    void set_target_box(ArmorDetector P);
    void set_last_box(ArmorDetector P){last_box = P;}
//    void run(cv::Mat &src);                             // 自瞄主函数
};
