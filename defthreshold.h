//#ifndef DEFTHRESHOLD_H
//#define DEFTHRESHOLD_H
// //一帧经过的时间
//#define delta_T 0.033333f
// //正无穷大
//#define infinity std::numeric_limits<double>::infinity()
// //对象的面积阈值
//constexpr float person_area=1;
//constexpr float bicycle_area=1.5;
//constexpr float tricycle_area=3;
//constexpr float car_area=4;
//constexpr float suv_area=4.5;
//constexpr float truck_area=6;
//constexpr float heavytruck_area=7;
//constexpr float coach_area=7;

// //视频的像素
//   //图像的大小
//constexpr float img_wideth=850;//手动更改
//constexpr float img_height=480;

//constexpr float edge_left=img_wideth/10;
//constexpr float edge_right=(img_wideth-img_wideth/10);
//constexpr float edge_top=img_height/5;
//constexpr float edge_below=(img_height-img_height/10);

// //范围r,可大一些,尽量使相撞物体包含在内
//constexpr float r_threshold=img_height/5;

// //碰撞距离阈值
//constexpr float crash_distance=img_height/20;

// //加速度阈值
//constexpr float asum_threshold=1.412;//没用,因为不同类型的加速度阈值不同

// //疑似碰撞对象的坐标移动阈值
//constexpr float r_move=img_height/15;

// //疑似对象的检查帧数
//constexpr int like_check_thre=11;
//#endif // DEFTHRESHOLD_H



#ifndef DEFTHRESHOLD_H
#define DEFTHRESHOLD_H
#include "cmath"
//
//#define AREA1 (1920*1080)
//#define W1      (1920)
//#define H1      (1080)
//一帧经过的时间
#define delta_T 0.033333f
//正无穷大
#define infinity std::numeric_limits<double>::infinity()
//对象的面积阈值
 float person_area;
 float bicycle_area;
 float tricycle_area;
 float car_area;
 float suv_area;
 float truck_area;
 float heavytruck_area;
 float coach_area;

 //图像的大小
 float img_wideth;
 float img_height;
 float img_area;
 //图像边缘区域
 float edge_left;
 float edge_right;
 float edge_top;
 float edge_below;

 //范围r,可大一些,尽量使相撞物体包含在内
  float r_threshold;
  float r_big_threshold;//truck,heavytruck,coach

  //碰撞距离阈值
  float crash_distance;

  //疑似碰撞对象的坐标移动阈值
  float r_move;//not used


  //v filter
  float v_bic=sqrt(25*25+10*10);//bicycle
  float v_car=sqrt(30*30+30*30);//car,suv
  float v_truck=sqrt(30*30+30*30);//truck,heavytruck,coach



 void RXY_threinit(float& img_wideth,float& img_height){
     float W1=1920.0;float H1=1080.0;
     float AREA1=W1*H1;

      edge_left=img_wideth*(94.5/W1);
      edge_right=img_wideth*(1801/W1);
      edge_top=img_height*(300/H1);
      edge_below=img_height*(886/H1);

      r_threshold=img_wideth*(400/W1);//
      r_big_threshold=img_wideth*(440/W1);

      crash_distance=img_wideth*(150/W1);

      img_area=img_height*img_wideth;

      person_area=img_area*(786/AREA1);
      bicycle_area=img_area*(786/AREA1);
      tricycle_area=img_area*(1475/AREA1);
      car_area=img_area*(999/AREA1);
      suv_area=img_area*(1338/AREA1);
      truck_area=img_area*(3067/AREA1);
      heavytruck_area=img_area*(4180/AREA1);
      coach_area=img_area*(3143/AREA1);
 }




//加速度阈值
// float asum_threshold=1.412;//没用,因为不同类型的加速度阈值不同

//移动因数:检测物体移动大于kmove*wideth时为移动
constexpr float kmove=0.2;

//疑似对象的检查帧数
constexpr int like_check_thre=11;
//未出事故的消除对象检查帧数
constexpr int noacci_count_thre=1;
//事故对象的相撞对象的检查帧数
constexpr int neigh_check_thre=5;
//速度滤波
constexpr float vfilter_thre=300;
constexpr float vfilter_lowthre=20;// 低速下限
constexpr float vy_filter_thre=3.8;// y向行驶相撞物体的vyf下限
constexpr float vyf_gain=1;// vy_filter的放大增益

constexpr float vsum_staythre=10;
constexpr float slide_staythre=5;
constexpr int stay_checkthre=100;
#endif // DEFTHRESHOLD_H


