#include "mybox.h"

//#include "mybox.h"
#include "getline.h"
#include "distance.h"
//#include "missingbox.h"
//#include "defthreshold.h"
#include<vector>
#include<string>
#include <iostream>
#include <fstream>



/*****读取txt文件****/
std::vector<std::vector<mybox>> RXY_processLabel(std::ifstream& label_file) {
    // Process labels - group bounding boxes by frame index
    std::vector<std::vector<mybox>> bbox;//放全部帧
    std::vector<mybox> bbox_per_frame;//一个放mybox的动态数组,放一帧
    // Label index starts from 1
    int current_frame_index = 1;
    std::string line;

    while (std::getline(label_file, line))//label_file作为输入流，逐行读取，将读取内容放入line
    {
        std::stringstream ss(line);
        // Label format <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>
        //实际我们上一步传下来的txt中只有：<frame>，<x>（中心点）, <y>（中心点），<w>,<h>
        std::vector<float> label;
        std::string data;
        while ( std::getline(ss , data, ',') )//读取到','时结束
        {
            std::string::iterator it;
            bool flag_number=true;
            for(it=data.begin();it<data.end();++it){
                if((*it<'z')&&(*it>'a'))
                    flag_number=false;
                    break;
            }
            if(flag_number){
                label.push_back(std::stof(data));
            }
            else
            {
                if(data.find("person",0)!=std::string::npos){label.push_back(1);}
                else if(data.find("bicycle",0)!=std::string::npos){label.push_back(2);}
                else if(data.find("tricycle",0)!=std::string::npos){label.push_back(3);}
                else if(data.find("car",0)!=std::string::npos){label.push_back(4);}
                else if(data.find("suv",0)!=std::string::npos){label.push_back(5);}
                else if(data.find("heavytruck",0)!=std::string::npos){label.push_back(7);}
                else if(data.find("truck",0)!=std::string::npos){label.push_back(6);}
                else{label.push_back(8);}
             }
            }


        if (label[0] != current_frame_index) {
            current_frame_index = static_cast<int>(label[0]);
            bbox.push_back(bbox_per_frame);
            bbox_per_frame.clear();
        }
        mybox bbbox;
        bbbox.init(label[4],label[5],label[6],label[7],label[3],label[1],label[8],label[9],label[10],label[11]);
//输入文档格式：frame,ID,___,class,x,y,W,H,ax,ay,vx,vy
            bbox_per_frame.emplace_back(bbbox);

    }
    // 最后一帧
    bbox.push_back(bbox_per_frame);
    return bbox;
}

/*1.判断前后帧之间是否有ID消失，若有，返回ID，type,frame;*/
std::vector<missingbox> RXY_findmissingbox(std::vector<mybox>& detections1,std::vector<mybox>& detections2,size_t& frame){
     //missingbox miss_boxes;
    std::vector<missingbox> miss_boxes;//一帧中消失的所有对象

    bool flag_findid=true;
    for(size_t i=0;i<detections1.size();i++){
        flag_findid=false;float dx=0;float dy=0;float dxm=0;float dym=0;
        for(size_t j=0;j<detections2.size();j++){
           if(detections1.at(i).ID_number==detections2.at(j).ID_number)//未消失
           {
               dx=fabs(detections1.at(i).boxings.x-detections2.at(j).boxings.x);
               dy=fabs(detections1.at(i).boxings.y-detections2.at(j).boxings.y);
               if(dx!=0){
                   detections1.at(i).k=dy/dx;//运动方向的比例系数
               }
               else {
                   detections1.at(i).k=infinity;//浮点数表示的正无穷大
               }
               detections1.at(i).dx=detections2.at(j).boxings.x-detections1.at(i).boxings.x;
               detections1.at(i).dy=detections2.at(j).boxings.y-detections1.at(i).boxings.y;
               flag_findid=true;
               break;
           }
        }
        if(!flag_findid){
            //记录ID消失前一帧的信息
             missingbox miss_box;

             miss_box.x=detections1.at(i).boxings.x;
             miss_box.y=detections1.at(i).boxings.y;
             miss_box.vx=detections1.at(i).vx;//有正负
             miss_box.vy=detections1.at(i).vy;
             miss_box.dx=miss_box.vx*delta_T;//有正负
             miss_box.dy=miss_box.vy*delta_T;
             miss_box.wideth=detections1.at(i).boxings.width;
             miss_box.height=detections1.at(i).boxings.height;
             miss_box.missing_area=detections1.at(i).boxings.area();
             miss_box.missing_ID=detections1.at(i).ID_number;
             miss_box.missing_type=detections1.at(i).typenumber;
             miss_box.missing_frame=frame;//消失前一帧

             dxm=fabs(miss_box.vx*delta_T);
             dym=fabs(miss_box.vy*delta_T);
             if(miss_box.dx){
                 miss_box.k=dym/dxm;
             }
             else{
                 miss_box.k=infinity;
             }

             miss_boxes.push_back(miss_box);
        }
    }

    return miss_boxes;
}

/*3.寻找消失ID周围r区域内的对象*/
std::vector<mybox> RXY_findneighbour(missingbox&miss_box,const std::vector<mybox>&detections){
     std::vector<mybox> neighbourboxes;
     std::vector<mybox>::const_iterator it_detections;
     for(it_detections=detections.begin();it_detections<detections.end();++it_detections){
         float dx=fabs((*it_detections).boxings.x-miss_box.x);
         float dy=fabs((*it_detections).boxings.y-miss_box.y);
         float dist=sqrt(dx*dx+dy*dy);
         if(dist<r_threshold){
            neighbourboxes.push_back(*it_detections);
         }
     }
     return neighbourboxes;
}


/*4.对于疑似消失的对象，在后10帧中做检测，检查是否出现，出现坐标和原坐标是否有较大的变动*/
void RXY_morecheck(const std::vector<mybox>&detections,std::vector<missingbox>&like_accidentboxes,std::vector<missingbox>&miss_to_accidentboxes){
//    std::vector<missingbox> miss_to_perframe;
    std::vector<missingbox>::iterator it_like_acc;
    std::vector<mybox>::const_iterator it_det;
    for(it_like_acc=like_accidentboxes.begin();it_like_acc<like_accidentboxes.end();++it_like_acc){
        for(it_det=detections.begin();it_det<detections.end();++it_det){
            if(it_like_acc->missing_ID==it_det->ID_number&&(*it_like_acc).like_check<like_check_thre){
                float DX=fabs(it_det->boxings.x-it_like_acc->x);
                float DY=fabs(it_det->boxings.y-it_like_acc->y);
                float D_sum=sqrt(DX*DX+DY*DY);
 //???可能会出现被遮挡只框出露出的一部分,识别框的中心点移动的情况
 //r_move应大于框的一半宽度
                if(D_sum>(it_like_acc->wideth)*0.7){
                    it_like_acc->flag_accident=false;
                    /*it_like_acc=like_accidentboxes.erase(it_like_acc);*///只挑出出事故的对象，对未出事故的不关心
                    break;
                }
                else{
                    if(it_like_acc->like_check<11){(*it_like_acc).like_check++;}
                    else{miss_to_accidentboxes.push_back(*it_like_acc);}//如果超过检查次数，则判为发生事故
                    break;
                }
            }
        }
    }
//    return miss_to_accidentboxes;
}





void RXY_Judgement(const std::vector<std::string>& dataset_names){
    for (const auto& dataset_name : dataset_names)//读取文件
    {
        // Open label file
        std::string label_path = "/home/ruanxinyao/lastyear_project/Judgement1/data/"+dataset_name+"/det.txt";//输入txt路径
        std::ifstream label_file(label_path);//定义ifstream对象label_file，路径为label_path
        if (!label_file.is_open()) {
            std::cerr << "Could not open or find the label!!!" << std::endl;
//            return -1;
        }
        std::vector<std::vector<mybox>> all_detections = RXY_processLabel(label_file);
        label_file.close();

        std::string output_path = "/home/ruanxinyao/lastyear_project/Judgement1/output/"+dataset_name+".txt";
        std::ofstream output_file(output_path);

        if (output_file.is_open()) {
            std::cout << "Result will be exported to " << output_path << std::endl;
        }
        else {
            std::cerr << "Unable to open output file" << std::endl;
//            return -1;
        }


        //输入视频像素
        //        std::cout<<"INPUT img_wideth: ";
        //        std::cin>>img_wideth;
        //        std::cout<<"INPUT img_height: ";
        //        std::cin>>img_height;
        std::vector<int> id_check;
        int missaccident_count=0;//总计消失对象的事故个数
        int nomissaccident_count=0;//总计未消失对象的事故个数
        size_t total_frame=all_detections.size();
        std::vector<missingbox>miss_to_accidentboxes;//确认为事故的ID（有消失）
        std::vector<mybox>nomiss_to_accidentboxes;//确认为事故的ID（无消失）
        std::vector<missingbox>like_accidentboxes;//对于消失的ID，通过前后两帧对比找出的疑似事故对象
        std::vector<missingbox> miss_boxes;
        std::vector<mybox> neighbourboxes;
        bool flag_ID_missing=true;

        for(size_t i=0;i<total_frame-1;i++){
            const auto &detections=all_detections[i];//一帧
            miss_boxes=RXY_findmissingbox(all_detections[i],all_detections[i+1],i);
//从疑似对象中找出真正碰撞的对象
            RXY_morecheck(detections,like_accidentboxes,miss_to_accidentboxes);
            like_accidentboxes.clear();
//判断有无ID消失
            if(miss_boxes.empty()){
                flag_ID_missing=false;//没有ID消失
               //判断是否有对象的加速度超过阈值
               std::vector<mybox>::iterator it_det_perframe;
               for(it_det_perframe=all_detections[i].begin();it_det_perframe<all_detections[i].end();++it_det_perframe){
                   if((*it_det_perframe).a_sum>it_det_perframe->a_threshold)//不同类型的对象加速度阈值不同
                   {
                       //发生事故
                      it_det_perframe->flag_accident=true;
                      it_det_perframe->accident_frame=i;
                      nomiss_to_accidentboxes.push_back(*it_det_perframe);//放入发生碰撞对象的数组

                   }
               }
            }
            else{
                flag_ID_missing=true;//有ID消失
            }
//判断每一个消失ID是否为驶离图片或驶远
            if(flag_ID_missing){
                std::vector<missingbox>::iterator it_miss_boxes;
                for(it_miss_boxes=miss_boxes.begin();it_miss_boxes<miss_boxes.end();++it_miss_boxes){
                    //判断是否是变小或在边缘处消失
                    if((*it_miss_boxes).missing_area<=(*it_miss_boxes).missbox_areathod((*it_miss_boxes).missing_type))
                    {   (*it_miss_boxes).flag_runaway=true;}//变小
                    else if(((*it_miss_boxes).x<edge_left&&(*it_miss_boxes).vx<0)||((*it_miss_boxes).x>edge_right&&(*it_miss_boxes).vx>0)||((*it_miss_boxes).y<edge_top&&(*it_miss_boxes).vy<0)||((*it_miss_boxes).y>edge_below&&(*it_miss_boxes).vy>0))
                    {   (*it_miss_boxes).flag_runaway=true;}//在边缘消失，且其远离图片行驶
                    else
                    {   (*it_miss_boxes).flag_runaway=false;}
                    //再判断是碰撞还是遮挡
                    if(!((*it_miss_boxes).flag_runaway)){
                        //找出范围r内的相邻物体
                        neighbourboxes=RXY_findneighbour((*it_miss_boxes),detections);
                        //画出当量线段
                        RXY_getline_forneighbour(neighbourboxes);
                        RXY_getline_formissbox(*it_miss_boxes);
                        //计算最近距离
                        float mindistance=RXY_distmin_neighbour(*it_miss_boxes,neighbourboxes);
                        if(mindistance<=crash_distance){
                            //疑似发生事故
                           it_miss_boxes->flag_accident=true;
                           like_accidentboxes.push_back(*it_miss_boxes);//放入疑似事故对象中
                        }
                    }
                }
            }
//输出文档
     std::vector<int>::iterator it_id_check;
     std::cout<<"***nomissing accidentboxes***"<<std::endl;
       for(auto &nomtacci : nomiss_to_accidentboxes){
           std::cout<<nomtacci.accident_frame<<","<<nomtacci.ID_number<<","<<nomtacci.boxings.x<<","<<nomtacci.boxings.y<<std::endl;
           output_file<<nomtacci.accident_frame<<","<<nomtacci.ID_number<<","<<nomtacci.boxings.x<<","<<nomtacci.boxings.y<<std::endl;
           for(it_id_check=id_check.begin();it_id_check<id_check.end();++it_id_check){
               if((*it_id_check)==nomtacci.ID_number){nomtacci.flag_cout=true;break;}
           }
           if(!nomtacci.flag_cout){id_check.push_back(nomtacci.ID_number);nomissaccident_count++;}
       }

     std::cout<<"***missing accidentboxes***"<<std::endl;
       for(auto &mtacci : miss_to_accidentboxes ){
           std::cout<<mtacci.missing_frame<<","<<mtacci.missing_ID<<","<<mtacci.x<<","<<mtacci.y<<std::endl;
           output_file<<mtacci.missing_frame<<","<<mtacci.missing_ID<<","<<mtacci.x<<","<<mtacci.y<<std::endl;
           for(it_id_check=id_check.begin();it_id_check<id_check.end();++it_id_check){
               if((*it_id_check)==mtacci.missing_ID){mtacci.flag_cout=true;break;}
           }
           if(!mtacci.flag_cout){id_check.push_back(mtacci.missing_ID);nomissaccident_count++;}
       }



                    miss_boxes.clear();//不储存每一帧消失的对象
                    miss_to_accidentboxes.clear();
                    nomiss_to_accidentboxes.clear();



       }//end of each frame


        std::cout<<"******事故对象总计******"<<std::endl;
        std::cout<<"missingaccident_count:"<<missaccident_count<<std::endl;
        std::cout<<"nomissingaccident_count:"<<nomissaccident_count<<std::endl;
        std::cout<<"total accidentID_count:"<<missaccident_count+nomissaccident_count<<std::endl;


    }//end of datasets_name
}












int main()
{
    std::vector<std::string> dataset_names{"AAAA-1"};
    RXY_Judgement(dataset_names);

    return 0;
}
