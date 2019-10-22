#ifndef MYBOX_H
#define MYBOX_H

#include <opencv2/core.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
//#include <vector>
class Rnode
{
public:
    float x;
    float y;
};

class mybox
{
public:
   cv::Rect boxings;
   int typenumber;//记录对象类型：1-person  2-bicycle  3-tricycle  4-car  5-suv  6-truck  7-heavytruck  8-coach
   int ID_number;
   int accident_frame;//事故帧
   float k;//运动方向的比例系数k=dy/dx,绝对值
   float dy;//下一帧y-本帧y，有正负
   float dx;//下一帧x-本帧x，有正负
   bool flag_accident=false;//事故发生flag
   //画当量线段
   Rnode A;//中心点
   Rnode B;//由速度方向确定点
   float ax;//x方向加速度，带正负
   float ay;//y方向加速度，带正负
   float a_sum;//总加速度
   float vx;//x方向速度，带正负
   float vy;//y方向速度，带正负
   float v_sum;//总速度
   bool flag_cout=false;

   float a_threshold=0;//不同的类型的加速度阈值不同

   void init(float a,float b,float c,float d,float e,int f,float g,float h,float i,float j){
       boxings.x=a;
       boxings.y=b;
       boxings.width=c;
       boxings.height=d;
       typenumber=e;
       ID_number=f;
       ax=g;
       ay=h;
       a_sum=sqrt(ax*ax+ay*ay);
       vx=i;
       vy=j;
       v_sum=sqrt(vx*vx+vy*vy);

       switch (typenumber) {
       case 1:
           a_threshold=1;break;
       case 2:
           a_threshold=2;break;
       case 3:
           a_threshold=3;break;
       case 4:
           a_threshold=4;break;
       case 5:
           a_threshold=5;break;
       case 6:
           a_threshold=6;break;
       case 7:
           a_threshold=7;break;
       default:
           a_threshold=8;
           break;
       }
   }

    void init(float a,float b,float c,float d){
       boxings.x=a;
       boxings.y=b;
       boxings.width=c;
       boxings.height=d;

   }

};



#endif // MYBOX_H

