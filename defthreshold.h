#ifndef DEFTHRESHOLD_H
#define DEFTHRESHOLD_H
//一帧经过的时间
#define delta_T 0.033333f
//正无穷大
#define infinity std::numeric_limits<double>::infinity()
//对象的面积阈值
constexpr float person_area=1;
constexpr float bicycle_area=1.5;
constexpr float tricycle_area=3;
constexpr float car_area=4;
constexpr float suv_area=4.5;
constexpr float truck_area=6;
constexpr float heavytruck_area=7;
constexpr float coach_area=7;

//视频的像素
   //图像的大小
constexpr float img_wideth=850;//手动更改
constexpr float img_height=480;

constexpr float edge_left=img_wideth/10;
constexpr float edge_right=(img_wideth-img_wideth/10);
constexpr float edge_top=img_height/5;
constexpr float edge_below=(img_height-img_height/10);

//范围r,可大一些,尽量使相撞物体包含在内
constexpr float r_threshold=img_height/5;

//碰撞距离阈值
constexpr float crash_distance=img_height/20;

//加速度阈值
constexpr float asum_threshold=1.412;//没用,因为不同类型的加速度阈值不同

//疑似碰撞对象的坐标移动阈值
constexpr float r_move=img_height/15;

//疑似对象的检查帧数
constexpr int like_check_thre=11;
#endif // DEFTHRESHOLD_H


