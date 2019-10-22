#ifndef MISSINGBOX_H
#define MISSINGBOX_H
#include<vector>
#include "defthreshold.h"
#include "mybox.h"
class missingbox
{
public:
    missingbox() {}
    int missing_frame;
    int missing_ID;
    int missing_type;
    float missing_area;
    float x;
    float y;
    float wideth;
    float height;
    float dx;//有正负
    float dy;//有正负
    float k;//绝对值
    float vx;//有正负
    float vy;//有正负
    float v_sum;
    //画当量线段
    Rnode A;//中心点
    Rnode B;//由速度方向确定点
    bool flag_runaway;//驶出画面或驶远：true
    bool flag_accident=false;//事故发生flag
    bool flag_cout=false;
    int like_check=0;//记录疑似对象的二次检查次数，超过10次未判出事故不再检测
//    std::vector<int> missing_frame;
//    std::vector<int> missing_ID;
//    std::vector<int> missing_type;
//    std::vector<float> missing_area;

    float missbox_areathod(const int&missing_type){
        float areathod;
        switch (missing_type) {
        case 1:
             areathod=person_area;
            break;
        case 2:
            areathod=bicycle_area;
            break;
        case 3:
            areathod=tricycle_area;
            break;
        case 4:
            areathod=car_area;
            break;
        case 5:
            areathod=suv_area;
            break;
        case 6:
            areathod=truck_area;
            break;
        case 7:
            areathod=heavytruck_area;
            break;
        default:
            areathod=coach_area;
            break;
        }
        return areathod;
    }



};






#endif // MISSINGBOX_H

