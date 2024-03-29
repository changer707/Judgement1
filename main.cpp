﻿#include "mybox.h"
#include "getline.h"
#include "distance.h"
#include<vector>
#include<map>
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
    int count=0;
    bool flag_unequal=false;
    std::string line;

    while (std::getline(label_file, line))//label_file作为输入流，逐行读取，将读取内容放入line
    {
        count++;
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
            if(count==1){
                flag_unequal=true;
                current_frame_index=static_cast<int>(label[0]);
                mybox bbbox1;
                bbbox1.init(label[4],label[5],label[6],label[7],label[3],label[1],label[8],label[9],label[10],label[11],label[0]);
                bbox_per_frame.emplace_back(bbbox1);
                bbox.push_back(bbox_per_frame);
                bbox_per_frame.clear();
            }
            else if(flag_unequal&&count==2){flag_unequal=false;}
            else{
                if(!flag_unequal){
                    current_frame_index = static_cast<int>(label[0]);
                    bbox.push_back(bbox_per_frame);
                    bbox_per_frame.clear();
                }
            }
        }
        if(!flag_unequal){
            mybox bbbox;
            bbbox.init(label[4],label[5],label[6],label[7],label[3],label[1],label[8],label[9],label[10],label[11],label[0]);

            //输入文档格式：frame,ID,___,class,x,y,W,H,ax,ay,vx,vy
            bbox_per_frame.emplace_back(bbbox);
        }
    }
    // 最后一帧
    bbox.push_back(bbox_per_frame);
    return bbox;
}

/*1.判断前后帧之间是否有ID消失，若有，返回ID，type,frame;*/
std::vector<missingbox> RXY_findmissingbox(std::vector<mybox>& detections1,std::vector<mybox>& detections2){
     //missingbox miss_boxes;
    std::vector<missingbox> miss_boxes;//all_detection[i]中消失的所有对象
    //all_detection[i]与all_detection[i+1]之间的帧数差
    int delta_frame=abs(detections1.at(0).frame-detections2.at(0).frame);
    //帧数差大于1,说明中间有无物体状态
if(delta_frame>1){
    std::vector<mybox>::iterator it_det1;
    for(it_det1=detections1.begin();it_det1<detections1.end();++it_det1){
       missingbox missbox;
       float dxn;float dyn;
       missbox.x=it_det1->boxings.x;
       missbox.y=it_det1->boxings.y;
       missbox.vx=it_det1->vx;
       missbox.vy=it_det1->vy;
       missbox.dx=missbox.vx*delta_T;
       missbox.dy=missbox.vy*delta_T;
       missbox.wideth=it_det1->boxings.width;
       missbox.height=it_det1->boxings.height;
       missbox.missing_area=it_det1->boxings.area();
       missbox.missing_ID=it_det1->ID_number;
       missbox.missing_type=it_det1->typenumber;
       missbox.missing_frame=it_det1->frame;

       dxn=fabs(missbox.vx*delta_T);
       dyn=fabs(missbox.vy*delta_T);
       if(missbox.dx){
           missbox.k=dyn/dxn;
       }
       else{
           missbox.k=infinity;
       }

       miss_boxes.push_back(missbox);
    }
}
else{
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
             miss_box.missing_frame=detections1.at(0).frame;//消失前一帧

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
  }
    return miss_boxes;
}

/*3.寻找消失ID周围r区域内的对象*/
std::vector<mybox> RXY_findneighbour(missingbox&miss_box,const std::vector<mybox>&detections){
    std::vector<mybox> neighbourboxes;
    std::vector<mybox>::const_iterator it_detections;
    for(it_detections=detections.begin();it_detections<detections.end();it_detections++){
        float dx=fabs((*it_detections).boxings.x-miss_box.x);
        float dy=fabs((*it_detections).boxings.y-miss_box.y);
        float dist=sqrt(dx*dx+dy*dy);
        //不把miss_box本身包括进neighbour
        if(miss_box.missing_ID!=it_detections->ID_number){
//            if(it_detections->typenumber<6){
                if(dist<r_threshold) {neighbourboxes.push_back(*it_detections);}
                //如果图面中有货车和客车,增大查找范围1.5倍
                else {
                    if(it_detections->typenumber>=6){
                        if(dist<1.5*r_threshold) {neighbourboxes.push_back(*it_detections);}
                    }
                }
//            }
            //遇到truck,heavytruck,coach时neighbour的搜索范围大一些
//            else {
//                if(dist<r_big_threshold) {neighbourboxes.push_back(*it_detections);}
//            }
        }
    }
    return neighbourboxes;
}


/*4.对于疑似消失的对象，在后10帧中做检测，检查是否出现，出现坐标和原坐标是否有较大的变动*/
void RXY_morecheck(const std::vector<mybox>&detections,std::vector<missingbox>&like_accidentboxes,std::vector<missingbox>&miss_to_accidentboxes){
    //    std::vector<missingbox> miss_to_perframe;
    std::vector<missingbox>::iterator it_like_acc;
    std::vector<mybox>::const_iterator it_det;
    //    bool flag_match=false;
    for(it_like_acc=like_accidentboxes.begin();it_like_acc<like_accidentboxes.end();++it_like_acc){
        it_like_acc->flag_merge=false;
        if(it_like_acc->like_check>like_check_thre){continue;}
        else if(it_like_acc->like_check==like_check_thre){it_like_acc->like_check++;miss_to_accidentboxes.push_back(*it_like_acc);}
        else{
            //超过计数阈值noacci_count_thre的对象认为未出事故,不再执行后续工作
            if(it_like_acc->noacci_count<noacci_count_thre){

                for(it_det=detections.begin();it_det<detections.end();++it_det){
                    //若疑似对象出现
                    if(it_like_acc->missing_ID==it_det->ID_number){
                        it_like_acc->flag_merge=true;
                        float DX=fabs(it_det->boxings.x-it_like_acc->x);
                        float DY=fabs(it_det->boxings.y-it_like_acc->y);
                        float D_sum=sqrt(DX*DX+DY*DY);
                        //???可能会出现被遮挡只框出露出的一部分,识别框的中心点移动的情况
                        //r_move应大于框的一半宽度
                        if((D_sum>(it_like_acc->wideth)*kmove)||it_like_acc->v_sum>20){
                            it_like_acc->flag_accident=false;
                            //计数,防止非事故对象的累积
                            it_like_acc->noacci_count++;
                            /*it_like_acc=like_accidentboxes.erase(it_like_acc);*///只挑出出事故的对象，对未出事故的不关心
                            break;
                        }
                        else{
                            if(it_like_acc->like_check<like_check_thre)//疑似事故对象检测帧
                                //{miss_to_accidentboxes.push_back(*it_like_acc);}
                                //可能速度比较慢,只检测当like_check结束以后的位置
                            {(*it_like_acc).like_check++;}
                            else{miss_to_accidentboxes.push_back(*it_like_acc);}//如果超过检查次数，则判为发生事故
                            break;
                        }
                    }
                }

                if(!it_like_acc->flag_merge){
                    if(it_like_acc->like_check<like_check_thre){(*it_like_acc).like_check++;}
                    else {
                        miss_to_accidentboxes.push_back(*it_like_acc);}
                }
            }
        }
    }
    //    return miss_to_accidentboxes;
}


//再次检查,对于事故车的撞击对象crashone,是否出现并移动,若是,未出事故;若否,出事故
//void RXY_neighcheck_accident(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&fmiss_to_accident){
//    for(int i=0;i<miss_to_accident.size();i++){
//    if(miss_to_accident[i].flag_accident&&(!miss_to_accident[i].neigh_change)){
//        miss_to_accident[i].flag_merge_acci=false;
//        if(miss_to_accident[i].crashone.neigh_check>neigh_check_thre){continue;}
//        //check==neigh_check,*出事故*
//        else if(miss_to_accident[i].crashone.neigh_check==neigh_check_thre){miss_to_accident[i].crashone.neigh_check++;fmiss_to_accident.push_back(miss_to_accident[i]);}
//        else{
//        for(int j=0;j<detections.size();j++){
//            //crashone是否出现
//            if(miss_to_accident[i].crashone.ID_number==detections[j].ID_number){
//                miss_to_accident[i].flag_merge_acci=true;//对象出现
//                float dx=fabs(miss_to_accident[i].crashone.boxings.x-detections[j].boxings.x);
//                float dy=fabs(miss_to_accident[i].crashone.boxings.y-detections[j].boxings.y);
//                float dsum=sqrt(dx*dx+dy*dy);
//              //crashone是否移动
//                if((dsum>(miss_to_accident[i].crashone.boxings.width)*kmove)){
//                    miss_to_accident[i].flag_accident=false;
//                    miss_to_accident[i].neigh_change=1;
//                    break;
//                }
//                 //未移动,未超过检查次数,check++;超过检查次数,*出事故*
//                else{
//                    miss_to_accident[i].crashone.dist_neighcheck=dsum;//刷新
//                    if(miss_to_accident[i].crashone.neigh_check<neigh_check_thre){miss_to_accident[i].crashone.neigh_check++;}
//                    else{fmiss_to_accident.push_back(miss_to_accident[i]);}
//                    break;
//                }
//            }
//        }
//        //该帧内crashone没出现过,未超过检查次数,check++;超过检查次数,*出事故*
//        if(!miss_to_accident[i].flag_merge_acci){
//            if(miss_to_accident[i].crashone.neigh_check<neigh_check_thre){miss_to_accident[i].crashone.neigh_check++;}
//            else{fmiss_to_accident.push_back(miss_to_accident[i]);}
//        }
//      }
//    }
//  }
//}
std::vector<mybox> RXY_findneighbour1(mybox&nomiss_box,const std::vector<mybox>&detections){
    std::vector<mybox> neighbourboxes;
    std::vector<mybox>::const_iterator it_detections;
    for(it_detections=detections.begin();it_detections<detections.end();it_detections++){
        float dx=fabs((*it_detections).boxings.x-nomiss_box.boxings.x);
        float dy=fabs((*it_detections).boxings.y-nomiss_box.boxings.y);
        float dist=sqrt(dx*dx+dy*dy);
        //不把miss_box本身包括进neighbour
        if(nomiss_box.ID_number!=it_detections->ID_number){
//            if(it_detections->typenumber<6){
                if(dist<r_threshold) {neighbourboxes.push_back(*it_detections);}
//            }
            //遇到truck,heavytruck,coach时neighbour的搜索范围大一些
//            else {
//                if(dist<r_big_threshold) {neighbourboxes.push_back(*it_detections);}
//            }
        }
    }
    return neighbourboxes;
}



void RXY_neighcheck_accident(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&fmiss_to_accident){
     std::vector<missingbox>::iterator it_miss;
     for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
         it_miss->crashone.neigh_check++;
         if(it_miss->crashone.neigh_check>=neigh_check_thre){
             if(!it_miss->crashone.flag_neighmove&&(!it_miss->crashone.flag_added)){
                 fmiss_to_accident.push_back(*it_miss);
                 it_miss->crashone.flag_added=true;
             }
             else {
                 continue;
             }
         }
         else{
             if(!it_miss->crashone.flag_neighmove){
                 for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                     if(it_miss->crashone.ID_number==detections[i].ID_number){
                         it_miss->crashone.flag_merge_neighstay1=true;
                         float dx=fabs(detections[i].boxings.x-it_miss->crashone.boxings.x);
                         float dy=fabs(detections[i].boxings.y-it_miss->crashone.boxings.y);
                         float dsum=sqrt(dx*dx+dy*dy);
                         it_miss->crashone.dist_neighcheck=dsum;
                    //移动是否大于0.1*wideth
                         if(dsum>(it_miss->crashone.boxings.width*nmove)){
                            it_miss->crashone.flag_neighmove=true;
                         }
                         break;
                     }
                 }
             }
         }
     }
}


//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其crashone在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_crashonestay(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&stay_to_accident){
    std::vector<missingbox>::iterator it_miss;
    for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
        it_miss->crashone.stay_check++;
        if(it_miss->crashone.stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_miss->crashone.flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_miss->crashone.ID_number==detections[i].ID_number){
                        it_miss->crashone.flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_miss->crashone.boxings.x-detections[i].boxings.x);
                            float dy=fabs(it_miss->crashone.boxings.y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_miss->crashone.dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_miss->crashone.dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<slide_staythre){
                                    it_miss->crashone.flag_stay=true;
                                    stay_to_accident.push_back(*it_miss);
                                }
                            }
                            it_miss->crashone.dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}

//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其miss在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_missstay(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&stay_to_accident){
    std::vector<missingbox>::iterator it_miss;
    for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
        it_miss->stay_check++;
        if(it_miss->stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_miss->flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_miss->missing_ID==detections[i].ID_number){
                        it_miss->flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_miss->x-detections[i].boxings.x);
                            float dy=fabs(it_miss->y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_miss->dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_miss->dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<slide_staythre){
                                    it_miss->flag_stay=true;
                                    stay_to_accident.push_back(*it_miss);
                                }
                            }
                            it_miss->dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}




//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其crashone在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_crashonestay1(const std::vector<mybox>&detections,std::vector<mybox>&nomiss_to_accident,std::vector<mybox>&stay_to_accident){
    std::vector<mybox>::iterator it_nomiss;
    for(it_nomiss=nomiss_to_accident.begin();it_nomiss<nomiss_to_accident.end();++it_nomiss){
        it_nomiss->crashone1.stay_check++;
        if(it_nomiss->crashone1.stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_nomiss->crashone1.flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_nomiss->crashone1.ID_number==detections[i].ID_number){
                        it_nomiss->crashone1.flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_nomiss->crashone1.boxings.x-detections[i].boxings.x);
                            float dy=fabs(it_nomiss->crashone1.boxings.y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_nomiss->crashone1.dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_nomiss->crashone1.dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<slide_staythre){
                                    it_nomiss->crashone1.flag_stay=true;
                                    stay_to_accident.push_back(*it_nomiss);
                                }
                            }
                            it_nomiss->crashone1.dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}

//miss,crashone最高速者vsum_filter小于300
//考虑对于这种类型的miss_to_accident,检测其miss在100帧后是否仍然存在且速度趋近于0,坐标不再移动
void RXY_nomissstay(const std::vector<mybox>&detections,std::vector<mybox>&nomiss_to_accident,std::vector<mybox>&stay_to_accident){
    std::vector<mybox>::iterator it_nomiss;
    for(it_nomiss=nomiss_to_accident.begin();it_nomiss<nomiss_to_accident.end();++it_nomiss){
        it_nomiss->stay_check++;
        if(it_nomiss->stay_check>=stay_checkthre){continue;}//100帧里没有发现停下来,就不认为事故
        else {
            if(!it_nomiss->flag_stay){
                for(int i=0;i<detections.size();i++){
                    //检测crashone是否出现
                    if(it_nomiss->ID_number==detections[i].ID_number){
                        it_nomiss->flag_merge_stay1=true;
                        //速度趋近于0
                        if(detections[i].v_sum<nomiss_vsum_staythre){
                            //记录每一帧已滑行距离
                            float dx=fabs(it_nomiss->boxings.x-detections[i].boxings.x);
                            float dy=fabs(it_nomiss->boxings.y-detections[i].boxings.y);
                            float dsum=sqrt(dx*dx+dy*dy);
                            if(!it_nomiss->dist_slide.empty()){
                                float delta_slide_dist=fabs(dsum-it_nomiss->dist_slide.back());
                                //坐标不再移动
                                if(delta_slide_dist<nomiss_slide_staythre){
                                    it_nomiss->flag_stay=true;
                                    stay_to_accident.push_back(*it_nomiss);
                                }
                            }
                            it_nomiss->dist_slide.push_back(dsum);
                        }
                    }
                }
            }
        }
    }
}

//速度修正,对vy&vx进行修正,vsum_filter
void RXY_Vfix_faccident(std::vector<missingbox>& fmiss_to_accident){
     std::vector<missingbox>::iterator it_fmiss;
     //y方向速度修正比例系数, y=520处l_filter=1
     for(it_fmiss=fmiss_to_accident.begin();it_fmiss<fmiss_to_accident.end();++it_fmiss){
         if(it_fmiss->y<520){
             it_fmiss->k_vfilter=0.1055;
             it_fmiss->b_vfilter=-25.9025;//-53.86;
             it_fmiss->crashone.k_vfilter=0.1055;
             it_fmiss->crashone.b_vfilter=-25.9025;//-53.86;
             //修正系数
             it_fmiss->l_vfilter=(it_fmiss->k_vfilter)*(it_fmiss->y)+it_fmiss->b_vfilter;
             it_fmiss->vy_filter=vyf_gain*(it_fmiss->vy)/(it_fmiss->l_vfilter);
//             it_fmiss->vx_filter=(it_fmiss->vx)/(it_fmiss->l_vfilter);
             it_fmiss->vsum_filter=sqrt(it_fmiss->vy_filter*it_fmiss->vy_filter+it_fmiss->vx*it_fmiss->vx);

             it_fmiss->crashone.l_vfilter=(it_fmiss->crashone.k_vfilter)*(it_fmiss->crashone.boxings.y)+it_fmiss->crashone.b_vfilter;
             it_fmiss->crashone.vy_filter=vyf_gain*(it_fmiss->crashone.vy)/(it_fmiss->crashone.l_vfilter);
//             it_fmiss->crashone.vx_filter=(it_fmiss->crashone.vx)/(it_fmiss->crashone.l_vfilter);
             it_fmiss->crashone.vsum_filter=sqrt(it_fmiss->crashone.vy_filter*it_fmiss->crashone.vy_filter+it_fmiss->crashone.vx*it_fmiss->crashone.vx);
         }
         else {
             it_fmiss->k_vfilter=0.0979;
             it_fmiss->b_vfilter=-72.9635;//-49.908;
             it_fmiss->crashone.k_vfilter=0.0979;
             it_fmiss->crashone.b_vfilter=-72.9635;//-49.908;

             it_fmiss->l_vfilter=(it_fmiss->k_vfilter)*(it_fmiss->y)+it_fmiss->b_vfilter;
             it_fmiss->vy_filter=vyf_gain*(it_fmiss->vy)/(it_fmiss->l_vfilter);
//             it_fmiss->vx_filter=(it_fmiss->vx)/(it_fmiss->l_vfilter);
             it_fmiss->vsum_filter=sqrt(it_fmiss->vy_filter*it_fmiss->vy_filter+it_fmiss->vx*it_fmiss->vx);

             it_fmiss->crashone.l_vfilter=(it_fmiss->crashone.k_vfilter)*(it_fmiss->crashone.boxings.y)+it_fmiss->crashone.b_vfilter;
             it_fmiss->crashone.vy_filter=vyf_gain*(it_fmiss->crashone.vy)/(it_fmiss->crashone.l_vfilter);
//             it_fmiss->crashone.vx_filter=(it_fmiss->crashone.vx)/(it_fmiss->crashone.l_vfilter);
             it_fmiss->crashone.vsum_filter=sqrt(it_fmiss->crashone.vy_filter*it_fmiss->crashone.vy_filter+it_fmiss->crashone.vx*it_fmiss->crashone.vx);
         }

     }

}

//速度滤波,将低速的fmiss_to_accident去掉;
//若miss_to_accident的速度小,而crashone的速度大,也认为是发生了事故(300>速度"小":>100;速度"大":>300)
void RXY_Vfilter_faccident(std::vector<missingbox>& miss_to_accident,std::vector<missingbox>& vfilter_fmiss,std::vector<missingbox>& vfilter_for_stay){
    std::vector<missingbox>::iterator it_miss;
    for(it_miss=miss_to_accident.begin();it_miss<miss_to_accident.end();++it_miss){
        //(1) (miss||crashone) >300
        if((it_miss->vsum_filter>vfilter_thre||it_miss->crashone.vsum_filter>vfilter_thre)&&(!it_miss->flag_fvout)){
            //(2) (miss&&crashone) >20
            if(it_miss->vsum_filter>vfilter_lowthre&&it_miss->crashone.vsum_filter>vfilter_lowthre){
            vfilter_fmiss.push_back(*it_miss);
            it_miss->flag_fvout=true;//防止同一对象重复放入vfilter_fmiss数组
            }
            //(3) miss为y向移动物体:vy_filter本来就小,vx小,vsum_filter小
            //    (i) crashone>20  (ii) |vyf| > vyf_thre
            if((!it_miss->flag_fvout)&&it_miss->crashone.vsum_filter>vfilter_lowthre&&fabs(it_miss->vy_filter)>vy_filter_thre){
            vfilter_fmiss.push_back(*it_miss);
            it_miss->flag_fvout=true;
            }
        }

        //(1)若miss和crashone都小于300,但其实速度还是还是比较大的,这种,放入数组,用stay筛选
        else{
            //(1) (miss||crashone) >30
            if((it_miss->vsum_filter>vsumfilter_forstay_thre||it_miss->crashone.vsum_filter>vsumfilter_forstay_thre)&&(!it_miss->flag_fvout)){
                //(2) (miss&&crashone) >15
                if(it_miss->vsum_filter>vfstay_lowstay&&it_miss->crashone.vsum_filter>vfstay_lowstay){
                vfilter_for_stay.push_back(*it_miss);
                it_miss->flag_fvout=true;//防止同一对象重复放入vfilter_for_stay数组
                }
                //(3) miss为y向移动物体:vy_filter本来就小,vx小,vsum_filter小
                //    (i) crashone>20  (ii) |vyf| > vyf_thre
                if((!it_miss->flag_fvout)&&it_miss->crashone.vsum_filter>vfstay_lowstay&&fabs(it_miss->vy_filter)>vy_filter_thre){
                vfilter_for_stay.push_back(*it_miss);
                it_miss->flag_fvout=true;
                }
            }
        }
    }
}


//寻找a和v都很大的物体
void RXY_highacceleration(const std::vector<mybox>& detections,std::vector<mybox>& high_accelerationboxes,std::vector<motionparasbox>& motionparasboxes){
    std::vector<mybox>::const_iterator it_det;
    std::vector<motionparasbox>::iterator it_paras;
    bool flag=false;
    for(it_det=detections.begin();it_det<detections.end();++it_det){
        if(it_det->a_sum>asum_thre){
            if(it_det->v_sum>vsum_thre){
                if(!motionparasboxes.empty()){
                    flag=false;
                    for(it_paras=motionparasboxes.begin();it_paras<motionparasboxes.end();++it_paras){
                        if(it_det->ID_number==it_paras->id){
                            flag=true;
                            if(it_det->a_sum>it_paras->asum&&it_det->v_sum>it_paras->vsum){
                                it_paras->asum=it_det->a_sum;
                                it_paras->vsum=it_det->v_sum;
                                std::vector<mybox>::iterator it_high;
                                for(it_high=high_accelerationboxes.begin();it_high<high_accelerationboxes.end();++it_high){
                                    if(it_high->ID_number==it_det->ID_number){
                                        *it_high=*it_det;
                                    }
                                }
                            }
                        }
                    }
                    if(!flag){
                        high_accelerationboxes.push_back(*it_det);
                        motionparasbox box1;
                        box1.id=it_det->ID_number;
                        box1.asum=it_det->a_sum;
                        box1.vsum=it_det->v_sum;
                        box1.flagadd_paras=true;
                        motionparasboxes.push_back(box1);
                    }
                }
                else{
                    high_accelerationboxes.push_back(*it_det);
                    motionparasbox box;
                    box.id=it_det->ID_number;
                    box.asum=it_det->a_sum;
                    box.vsum=it_det->v_sum;
                    box.flagadd_paras=true;
                    motionparasboxes.push_back(box);
                }
            }
        }
    }
}






//检查是否已经在miss_to_accidentboxes中
bool RXY_alreadyaccident(missingbox& like_acc,const std::vector<missingbox>& miss_to_accident){
    std::vector<missingbox>::const_iterator it_mtac;
    bool flag=false;
    for(it_mtac=miss_to_accident.begin();it_mtac<miss_to_accident.end();++it_mtac){
        if(like_acc.missing_frame==it_mtac->missing_frame&&like_acc.missing_ID==it_mtac->missing_ID){
          flag=true;
        }
    }
    return flag;
}


void RXY_Judgement(const std::vector<std::string>& dataset_names){
    for (const auto& dataset_name : dataset_names)//读取文件
    {
        // Open label file
std::string label_path = "/home/ruanxinyao/lastyear_project/AAAA-1/samples/11.12selected/24.txt"; //"/home/ruanxinyao/lastyear_project/Judgement1/data/"+dataset_name+"/det.txt";//输入txt路径
        std::ifstream label_file(label_path);//定义ifstream对象label_file，路径为label_path
        if (!label_file.is_open()) {
            std::cerr << "Could not open or find the label!!!" << std::endl;
            //            return -1;
        }
        std::vector<std::vector<mybox>> all_detections = RXY_processLabel(label_file);
        label_file.close();

std::string output_path = "/home/ruanxinyao/lastyear_project/Judgement1/output/11.12selected/"+dataset_name+"24.txt";
        std::ofstream output_file(output_path);

        //        std::string output_path_1="/home/ruanxinyao/lastyear_project/Judgement1/output/1min_num1.txt";
        //        std::ofstream output_file_1(output_path_1);

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
        std::vector<int> like;
        int missaccident_count=0;//总计消失对象的事故个数
        int nomissaccident_count=0;//总计未消失对象的事故个数
        int like_count=0;
        int vfilterfimss_count=0;
        int addvfilterfmiss_count=0;
        int fmissaccident_count=0;
        int like1_count=0;
        int stayaccident_count=0;
        int total_vs_count=0;

        size_t total_frame=all_detections.size();
        std::vector<missingbox>miss_to_accidentboxes;//确认为事故的ID（有消失）
        std::vector<mybox>highaboxes;//高加速度的对象
        std::vector<mybox>nomiss_to_accidentboxes;//确认为事故的ID（无消失）
        std::vector<missingbox>like_accidentboxes;//对于消失的ID，通过前后两帧对比找出的疑似事故对象
        std::vector<mybox>like_accidentboxes1;
        std::vector<missingbox> miss_boxes;
        std::vector<mybox> neighbourboxes;
        std::vector<mybox> neighbourboxes1;
        std::vector<missingbox>fmiss_to_accidentboxes;//经过neighbourcheck后的miss_to_accidentboxes
        std::vector<missingbox> countedlike_boxes;//存放已经统计过的疑似对象
        std::vector<missingbox> vfilter_fmiss;//速度滤波后的fmiss
        std::vector<missingbox> vfilter_for_stay;//速度不小,后续stay筛选
        std::vector<missingbox> stay_to_accidentboxes;//crashone碰撞后静止检测(miss已消失)
        std::vector<mybox> stay_to_accidentboxes1;
        std::vector<mybox> high_accelerationboxes;
        std::vector<missingbox> total_vfilter_stay;//静止检测和速度滤波后的输出事故对象
        std::vector<motionparasbox> motionparasboxes;
        //        std::map<int,motionparas> nomiss_motionparas;

        bool flag_ID_missing=true;
        bool onlyone_mtoacci=false;
        float mindistance=0;
        float mindistance1=0;
        for(size_t im=0;im<total_frame-1;im++)//0为第1帧
        {
            const auto &detections=all_detections[im];//一帧
            miss_boxes=RXY_findmissingbox(all_detections[im],all_detections[im+1]);

            //(1)从所有box中找高加速度,高速度的对象
            //(2)进行静止检测
            //-->nomiss_to_accidentboxes
            RXY_highacceleration(detections,high_accelerationboxes,motionparasboxes);
            RXY_nomissstay(detections,high_accelerationboxes,nomiss_to_accidentboxes);

            //从疑似对象like_to_accidentboxes中找出真正碰撞的对象
            RXY_morecheck(detections,like_accidentboxes,miss_to_accidentboxes);//找出miss_to_accidnetboxes
            RXY_Vfix_faccident(miss_to_accidentboxes);//速度消畸变
            RXY_Vfilter_faccident(miss_to_accidentboxes,vfilter_fmiss,vfilter_for_stay);//根据速度分类:后续做 速度滤波检测 还是 静止检测
            RXY_neighcheck_accident(all_detections[im-11],vfilter_fmiss,fmiss_to_accidentboxes);//x-x向的滤除
            RXY_crashonestay(detections,vfilter_for_stay,stay_to_accidentboxes);//crashone静止检测
            RXY_missstay(detections,vfilter_for_stay,stay_to_accidentboxes);//miss静止检测
//            RXY_crashonestay1(detections,like_accidentboxes1,stay_to_accidentboxes1);
//            RXY_nomissstay(detections,like_accidentboxes1,stay_to_accidentboxes1);

            //判断有无ID消失
            if(miss_boxes.empty()){
                flag_ID_missing=false;//没有ID消失
                //               //判断是否有对象的加速度超过阈值
                //               std::vector<mybox>::iterator it_det_perframe;
                //               for(it_det_perframe=all_detections[i].begin();it_det_perframe<all_detections[i].end();++it_det_perframe){
                //                   if((*it_det_perframe).a_sum>it_det_perframe->a_threshold)//不同类型的对象加速度阈值不同
                //                   {
                //                       //发生事故
                //                      it_det_perframe->flag_accident=true;
                //                      it_det_perframe->accident_frame=i;
                //                      nomiss_to_accidentboxes.push_back(*it_det_perframe);//放入发生碰撞对象的数组

                //                   }
                //               }
            }
            else{
                flag_ID_missing=true;//有ID消失
            }
            //判断每一个消失ID是否为驶离图片或驶远
            if(flag_ID_missing){
                //                std::vector<missingbox>::iterator it_miss_boxes;
                for(int in=0;in<miss_boxes.size();in++){
                    //判断是否是变小或在边缘处消失
                    if(miss_boxes[in].missing_area<=miss_boxes[in].missbox_areathod(miss_boxes[in].missing_type))
                    {   miss_boxes[in].flag_runaway=true;}//变小
                    else if((miss_boxes[in].x<edge_left&&miss_boxes[in].vx<0)||(miss_boxes[in].x>edge_right&&miss_boxes[in].vx>0)||(miss_boxes[in].y<edge_top&&miss_boxes[in].vy<0)||(miss_boxes[in].y>edge_below&&miss_boxes[in].vy>0))
                    {   miss_boxes[in].flag_runaway=true;}//在边缘消失，且其远离图片行驶
                    else
                    {   miss_boxes[in].flag_runaway=false;}
                    //再判断是碰撞还是遮挡
                    if(!(miss_boxes[in].flag_runaway)){
                        //找出范围r内的相邻物体
                        neighbourboxes=RXY_findneighbour(miss_boxes[in],detections);
                        if(!neighbourboxes.empty())//检查neighbor是否为空!
                        {
                            //画出当量线段
                            RXY_getline_forneighbour(neighbourboxes);
                            RXY_getline_formissbox(miss_boxes[in]);
                            //计算最近距离
                            mindistance=RXY_distmin_neighbour(miss_boxes[in],neighbourboxes);
                            if(mindistance<=crash_distance){
                                //疑似发生事故
                                miss_boxes[in].flag_accident=true;
                                like_accidentboxes.push_back(miss_boxes[in]);//放入疑似事故对象中
                            }
                        }
                    }
                }
            }

//            else{
//                //在画面中,有id消失,但是碰撞双方未消失
//                //在整幅画面中查找加速度极大者,查找他的最近邻,并做stay检测
//                  for(int mm=0;mm<detections.size();mm++){
//                      if(detections[mm].a_sum>asum_thre){
//                          highaboxes.push_back(detections[mm]);
//                      }
//                  }
//                  if(!highaboxes.empty()){
//                      for(int mn=0;mn<highaboxes.size();mn++){
//                          //找出附近物体
//                          neighbourboxes1=RXY_findneighbour1(highaboxes[mn],detections);
//                          if(!neighbourboxes1.empty()){
//                          //画出当量线段
//                             RXY_getline_forneighbour(neighbourboxes1);
//                             RXY_getline_fromdirection(highaboxes[mn]);
//                          //计算最近距离
//                             mindistance1=RXY_distmin_neighbour1(highaboxes[mn],neighbourboxes1);
//                             //最近距离小于碰撞距离
//                             if(mindistance1<crash_distance){
//                                 bool flag=false;
//                                 if(!like_accidentboxes1.empty()){
//                                     for(int il=0;il<like_accidentboxes1.size();il++){
//                                         if(highaboxes[mn].ID_number==like_accidentboxes1[il].ID_number){
//                                             flag=true;
//                                         }
//                                     }
//                                 }
//                                 if(!flag){
//                                     if(highaboxes[mn].v_sum>20&&highaboxes[mn].crashone1.v_sum>20){
//                               highaboxes[mn].flag_accident=true;
//                               like_accidentboxes1.push_back(highaboxes[mn]);//疑似事故,留待stay检测
//                                     }
//                               }
//                            }
//                          }
//                      }
//                  }
//            }



            //输出文档
            //     std::vector<int>::iterator it_id_check;
            //     std::cout<<"***nomissing accidentboxes***"<<std::endl;
            //       for(auto &nomtacci : nomiss_to_accidentboxes){
            //           std::cout<<nomtacci.accident_frame<<","<<nomtacci.ID_number<<","<<nomtacci.boxings.x<<","<<nomtacci.boxings.y<<std::endl;
            //           output_file<<nomtacci.accident_frame<<","<<nomtacci.ID_number<<","<<nomtacci.boxings.x<<","<<nomtacci.boxings.y<<std::endl;
            //           for(it_id_check=id_check.begin();it_id_check<id_check.end();++it_id_check){
            //               if((*it_id_check)==nomtacci.ID_number){nomtacci.flag_cout=true;break;}
            //           }
            //           if(!nomtacci.flag_cout){id_check.push_back(nomtacci.ID_number);nomissaccident_count++;}
            //       }

            for(int i=0;i< miss_to_accidentboxes.size();i++){
                if(!miss_to_accidentboxes[i].flag_output){
                    std::cout<<"***missing accidentboxes***"<<std::endl;
                    std::cout<<miss_to_accidentboxes[i].missing_frame<<","<<miss_to_accidentboxes[i].missing_ID<<","<<miss_to_accidentboxes[i].missing_type<<","<<miss_to_accidentboxes[i].x<<","<<miss_to_accidentboxes[i].y<<","<<miss_to_accidentboxes[i].crashone.ID_number <<"," <<miss_to_accidentboxes[i].vsum_filter<<","<<miss_to_accidentboxes[i].crashone.vsum_filter<<std::endl;
                    //           output_file<<miss_to_accidentboxes[i].missing_frame<<","<<miss_to_accidentboxes[i].missing_ID<<","<<miss_to_accidentboxes[i].missing_type<<","<<miss_to_accidentboxes[i].x<<","<<miss_to_accidentboxes[i].y<<std::endl;
                    miss_to_accidentboxes[i].flag_output=true;
                    missaccident_count++;
                    std::cout<<"****crashone boxes****"<<std::endl;
                    std::cout<<miss_to_accidentboxes[i].crashone.ID_number<<","<<miss_to_accidentboxes[i].crashone.typenumber<<","<<miss_to_accidentboxes[i].crashone.boxings.x<<","<<miss_to_accidentboxes[i].crashone.boxings.y<<std::endl;
                }
            }



            for(int ii=0;ii<vfilter_fmiss.size();ii++){
                if(!vfilter_fmiss[ii].flag_vout){
                    std::cout<<"***vfilter_fmiss****"<<std::endl;
                    //输出格式:   frame,id,class,x,y,crashone_id,crashone_class,vsum_filter,crashone_vsumfilter
                    std::cout<<vfilter_fmiss[ii].missing_frame<<","<<vfilter_fmiss[ii].missing_ID<<","<<vfilter_fmiss[ii].missing_type<<","<<vfilter_fmiss[ii].x<<","<<vfilter_fmiss[ii].y<<","
                            <<vfilter_fmiss[ii].crashone.ID_number<<","<<vfilter_fmiss[ii].crashone.typenumber<<","<<vfilter_fmiss[ii].vsum_filter<<","<<vfilter_fmiss[ii].crashone.vsum_filter<<"  "<<vfilter_fmiss[ii].flag_merge_acci<<"," <<vfilter_fmiss[ii].flag_accident<<std::endl;
//                    output_file<<vfilter_fmiss[ii].missing_frame<<","<<vfilter_fmiss[ii].missing_ID<<","<<vfilter_fmiss[ii].missing_type<<","<<vfilter_fmiss[ii].x<<","<<vfilter_fmiss[ii].y<<","
//                              <<vfilter_fmiss[ii].crashone.ID_number<<","<<vfilter_fmiss[ii].crashone.typenumber<<","<<vfilter_fmiss[ii].vsum_filter<<","<<vfilter_fmiss[ii].crashone.vsum_filter<<std::endl;
                    vfilter_fmiss[ii].flag_vout=true;
                    vfilterfimss_count++;
                }
            }


            for(int ic=0;ic<stay_to_accidentboxes.size();ic++){
                if(!stay_to_accidentboxes[ic].flag_sout){
                    std::cout<<"***stay_to_accident***************************"<<std::endl;
                    std::cout<<stay_to_accidentboxes[ic].missing_frame<<","<<stay_to_accidentboxes[ic].missing_ID<<","<<stay_to_accidentboxes[ic].crashone.ID_number<<std::endl;
                    //               output_file_1<<stay_to_accidentboxes[ic].missing_frame<<","<<stay_to_accidentboxes[ic].missing_ID<<","<<stay_to_accidentboxes[ic].crashone.ID_number<<std::endl;
                    stay_to_accidentboxes[ic].flag_sout=true;
                    stayaccident_count++;
                }
            }

//            for(int ii=0;ii<fmiss_to_accidentboxes.size();ii++){
//                if(!fmiss_to_accidentboxes[ii].flag_foutput){
//                    std::cout<<"******************fmiss**************************"<<std::endl;
//                    std::cout<<fmiss_to_accidentboxes[ii].missing_frame<<","<<fmiss_to_accidentboxes[ii].missing_ID<<","<<fmiss_to_accidentboxes[ii].missing_type<<","<<fmiss_to_accidentboxes[ii].x<<","<<fmiss_to_accidentboxes[ii].y<<","
//                            <<fmiss_to_accidentboxes[ii].crashone.ID_number<<","<<fmiss_to_accidentboxes[ii].crashone.typenumber<<","<<fmiss_to_accidentboxes[ii].crashone.dist_neighcheck<<","<<fmiss_to_accidentboxes[ii].flag_merge_stay1<<"," <<fmiss_to_accidentboxes[ii].crashone.boxings.width <<std::endl;
//    //                           output_file<<fmiss_to_accidentboxes[ii].missing_frame<<","<<fmiss_to_accidentboxes[ii].missing_ID<<","<<fmiss_to_accidentboxes[ii].missing_type<<","<<fmiss_to_accidentboxes[ii].x<<","<<fmiss_to_accidentboxes[ii].y<<","
//    //                                     <<fmiss_to_accidentboxes[ii].crashone.ID_number<<","<<fmiss_to_accidentboxes[ii].crashone.typenumber<<std::endl;
//                    fmiss_to_accidentboxes[ii].flag_foutput=true;
//                    fmissaccident_count++;
//                }
//            }


            //统计所有的疑似对象个数
            for(auto &likeab : like_accidentboxes ){
                if(!like_accidentboxes.empty()){
                    if(!likeab.flag_cout){
                        std::cout<<"******like_accidentboxes***"<<std::endl;
                        std::cout<<im<<","<<likeab.missing_frame<<","<<likeab.missing_ID<<","<<likeab.missing_type<<std::endl;
                        likeab.flag_cout=true;
                        like_count++;
                    }
                }
            }

            miss_boxes.clear();//不储存每一帧消失的对象
            neighbourboxes.clear();

        }//end of each frame

        //若速度滤波后无事故,且miss_to_accident仅有一个对象,则认为该对象为事故
//        if(miss_to_accidentboxes.size()==1&&!(vfilter_fmiss.size())){
//            std::cout<<"****add vfilterboxes****"<<std::endl;
//            std::cout<<miss_to_accidentboxes[0].missing_frame<<","<<miss_to_accidentboxes[0].missing_ID<<","<<miss_to_accidentboxes[0].missing_type<<","
//                     <<miss_to_accidentboxes[0].x<<","<<miss_to_accidentboxes[0].y<<","<<miss_to_accidentboxes[0].crashone.ID_number<<","<<miss_to_accidentboxes[0].crashone.typenumber<<","
//                     <<miss_to_accidentboxes[0].vsum_filter<<","<<miss_to_accidentboxes[0].crashone.vsum_filter<<std::endl;
//            output_file<<miss_to_accidentboxes[0].missing_frame<<","<<miss_to_accidentboxes[0].missing_ID<<","<<miss_to_accidentboxes[0].missing_type<<","
//                       <<miss_to_accidentboxes[0].x<<","<<miss_to_accidentboxes[0].y<<","<<miss_to_accidentboxes[0].crashone.ID_number<<","<<miss_to_accidentboxes[0].crashone.typenumber<<","
//                       <<miss_to_accidentboxes[0].vsum_filter<<","<<miss_to_accidentboxes[0].crashone.vsum_filter<<std::endl;
//            addvfilterfmiss_count++;
//            onlyone_mtoacci=true;
//        }


//将vfilter和stay的结果合并输出
        if(!vfilterfimss_count){
            total_vfilter_stay.insert(total_vfilter_stay.begin(),stay_to_accidentboxes.begin(),stay_to_accidentboxes.end());
        }
        else if (!stayaccident_count) {
            total_vfilter_stay.insert(total_vfilter_stay.begin(), vfilter_fmiss.begin(),vfilter_fmiss.end());
        }
        else {
            if((vfilterfimss_count+stayaccident_count)<=6){
               total_vfilter_stay.insert(total_vfilter_stay.begin(), vfilter_fmiss.begin(),vfilter_fmiss.end());
               total_vfilter_stay.insert(total_vfilter_stay.end(),stay_to_accidentboxes.begin(),stay_to_accidentboxes.end());
            }
            else{
                total_vfilter_stay.insert(total_vfilter_stay.begin(), vfilter_fmiss.begin(),vfilter_fmiss.end());
            }
        }

//将nomiss和total的结果合并输出
        if(total_vfilter_stay.empty()){
            for(int ww=0;ww<nomiss_to_accidentboxes.size();ww++){
                if(!nomiss_to_accidentboxes[ww].flag_ooutput){
                    //输出格式:   frame,id,class,x,y,crashone_id,crashone_class,vsum_filter,crashone_vsumfilter
                    output_file<<nomiss_to_accidentboxes[ww].frame+1<<","<<nomiss_to_accidentboxes[ww].ID_number<<","<<nomiss_to_accidentboxes[ww].typenumber<<","<<nomiss_to_accidentboxes[ww].boxings.x<<","<<nomiss_to_accidentboxes[ww].boxings.y<<std::endl;
                    //                              <<nomiss_to_accidentboxes[ww].crashone1.typenumber<<","<<vfilter_fmiss[ii].crashone.typenumber<<","<<vfilter_fmiss[ii].vsum_filter<<","<<vfilter_fmiss[ii].crashone.vsum_filter<<std::endl;
                    nomiss_to_accidentboxes[ww].flag_ooutput=true;
                }
            }
        }
        else if(nomiss_to_accidentboxes.empty()){
            for(int ee=0;ee<total_vfilter_stay.size();ee++){
                if(!total_vfilter_stay[ee].flag_ooutput){
                    output_file<<total_vfilter_stay[ee].missing_frame<<","<<total_vfilter_stay[ee].missing_ID<<","<<total_vfilter_stay[ee].missing_type<<","<<total_vfilter_stay[ee].x<<","<<total_vfilter_stay[ee].y<<","
                              <<total_vfilter_stay[ee].crashone.ID_number<<","<<total_vfilter_stay[ee].crashone.typenumber<<","<<total_vfilter_stay[ee].vsum_filter<<","<<total_vfilter_stay[ee].crashone.vsum_filter<<std::endl;
                    total_vfilter_stay[ee].flag_ooutput=true;
                }
            }
        }
        else{
            if(total_vfilter_stay.size()>6){
                for(int ww=0;ww<nomiss_to_accidentboxes.size();ww++){
                    if(!nomiss_to_accidentboxes[ww].flag_ooutput){
                        //输出格式:   frame,id,class,x,y,crashone_id,crashone_class,vsum_filter,crashone_vsumfilter
                        output_file<<nomiss_to_accidentboxes[ww].frame+1<<","<<nomiss_to_accidentboxes[ww].ID_number<<","<<nomiss_to_accidentboxes[ww].typenumber<<","<<nomiss_to_accidentboxes[ww].boxings.x<<","<<nomiss_to_accidentboxes[ww].boxings.y<<std::endl;
                        //                              <<nomiss_to_accidentboxes[ww].crashone1.typenumber<<","<<vfilter_fmiss[ii].crashone.typenumber<<","<<vfilter_fmiss[ii].vsum_filter<<","<<vfilter_fmiss[ii].crashone.vsum_filter<<std::endl;
                        nomiss_to_accidentboxes[ww].flag_ooutput=true;
                    }
                }
            }
            else{
                if(total_vfilter_stay.size()+nomiss_to_accidentboxes.size()<=6){
                    for(int ww=0;ww<nomiss_to_accidentboxes.size();ww++){
                        if(!nomiss_to_accidentboxes[ww].flag_ooutput){
                            //输出格式:   frame,id,class,x,y,crashone_id,crashone_class,vsum_filter,crashone_vsumfilter
                            output_file<<nomiss_to_accidentboxes[ww].frame+1<<","<<nomiss_to_accidentboxes[ww].ID_number<<","<<nomiss_to_accidentboxes[ww].typenumber<<","<<nomiss_to_accidentboxes[ww].boxings.x<<","<<nomiss_to_accidentboxes[ww].boxings.y<<std::endl;
                            //                              <<nomiss_to_accidentboxes[ww].crashone1.typenumber<<","<<vfilter_fmiss[ii].crashone.typenumber<<","<<vfilter_fmiss[ii].vsum_filter<<","<<vfilter_fmiss[ii].crashone.vsum_filter<<std::endl;
                            nomiss_to_accidentboxes[ww].flag_ooutput=true;
                        }
                    }
                    for(int ee=0;ee<total_vfilter_stay.size();ee++){
                        if(!total_vfilter_stay[ee].flag_ooutput){
                            output_file<<total_vfilter_stay[ee].missing_frame<<","<<total_vfilter_stay[ee].missing_ID<<","<<total_vfilter_stay[ee].missing_type<<","<<total_vfilter_stay[ee].x<<","<<total_vfilter_stay[ee].y<<","
                                      <<total_vfilter_stay[ee].crashone.ID_number<<","<<total_vfilter_stay[ee].crashone.typenumber<<","<<total_vfilter_stay[ee].vsum_filter<<","<<total_vfilter_stay[ee].crashone.vsum_filter<<std::endl;
                            total_vfilter_stay[ee].flag_ooutput=true;
                        }
                    }
                }
                else{
                    for(int ee=0;ee<total_vfilter_stay.size();ee++){
                        if(!total_vfilter_stay[ee].flag_ooutput){
                            output_file<<total_vfilter_stay[ee].missing_frame<<","<<total_vfilter_stay[ee].missing_ID<<","<<total_vfilter_stay[ee].missing_type<<","<<total_vfilter_stay[ee].x<<","<<total_vfilter_stay[ee].y<<","
                                      <<total_vfilter_stay[ee].crashone.ID_number<<","<<total_vfilter_stay[ee].crashone.typenumber<<","<<total_vfilter_stay[ee].vsum_filter<<","<<total_vfilter_stay[ee].crashone.vsum_filter<<std::endl;
                            total_vfilter_stay[ee].flag_ooutput=true;
                        }
                    }
                }
            }
        }



        std::cout<<"******事故对象总计******"<<std::endl;

        std::cout<<"missingaccident_count:"<<missaccident_count<<std::endl;
        std::cout<<"total accidentID_count:"<<missaccident_count+nomissaccident_count<<std::endl;
        std::cout<<"like_count:"<<like_count<<std::endl;
        std::cout<<"vfilter_count:"<<vfilterfimss_count<<std::endl;
        std::cout<<"addvfilter_count:"<<addvfilterfmiss_count<<std::endl;
        std::cout<<"fmissaccident_count:"<<fmiss_to_accidentboxes.size()<<std::endl;
        std::cout<<"stayaccident_count:"<<stayaccident_count<<std::endl;
//        std::cout<<"higha_count:"<<like_accidentboxes1.size()<<std::endl;
        std::cout<<"total vfilter&stay count:"<<total_vfilter_stay.size()<<std::endl;

        std::cout<<"highaccelerationboxes count:"<<high_accelerationboxes.size()<<std::endl;
        std::cout<<"nomissingaccident_count:"<<nomiss_to_accidentboxes.size()<<std::endl;



//        std::cout<<"***stay_to_accident***************************"<<std::endl;
//        for(int ic=0;ic<stay_to_accidentboxes.size();ic++){
//            if(!stay_to_accidentboxes[ic].flag_sout){

//                std::cout<<stay_to_accidentboxes[ic].missing_frame<<","<<stay_to_accidentboxes[ic].missing_ID<<","<<stay_to_accidentboxes[ic].crashone.ID_number<<std::endl;
//                //               output_file_1<<stay_to_accidentboxes[ic].missing_frame<<","<<stay_to_accidentboxes[ic].missing_ID<<","<<stay_to_accidentboxes[ic].crashone.ID_number<<std::endl;
//                stay_to_accidentboxes[ic].flag_sout=true;
//                stayaccident_count++;
//            }
//        }

        std::cout<<"****higha*****"<<std::endl;
        for(int qq=0;qq<high_accelerationboxes.size();qq++){
            if(!high_accelerationboxes[qq].flag_loutput){
                std::cout<<high_accelerationboxes[qq].frame+1<<","<<high_accelerationboxes[qq].ID_number<<","<<high_accelerationboxes[qq].a_sum<<","<<high_accelerationboxes[qq].v_sum<<std::endl;
                high_accelerationboxes[qq].flag_loutput=true;
            }
        }




        std::cout<<"***nomiss**************************"<<std::endl;

        for(int ii=0;ii<nomiss_to_accidentboxes.size();ii++){
            if(!nomiss_to_accidentboxes[ii].flag_noutput){
                std::cout<<nomiss_to_accidentboxes[ii].frame+1<<","<<nomiss_to_accidentboxes[ii].ID_number<<","<<nomiss_to_accidentboxes[ii].a_sum<<","<<nomiss_to_accidentboxes[ii].v_sum<<std::endl;
                nomiss_to_accidentboxes[ii].flag_noutput=true;

            }
        }



        std::cout<<"***total vfilter&stay***"<<std::endl;
        for(int ie=0;ie<total_vfilter_stay.size();ie++){
            if(!total_vfilter_stay[ie].flag_tout){
                std::cout<<total_vfilter_stay[ie].missing_frame<<","<<total_vfilter_stay[ie].missing_ID<<","<<total_vfilter_stay[ie].crashone.ID_number<<std::endl;
                //               output_file_1<<stay_to_accidentboxes[ic].missing_frame<<","<<stay_to_accidentboxes[ic].missing_ID<<","<<stay_to_accidentboxes[ic].crashone.ID_number<<std::endl;
                total_vfilter_stay[ie].flag_tout=true;
                total_vs_count++;
            }
        }



//        for(int ii=0;ii<fmiss_to_accidentboxes.size();ii++){
//            if(!fmiss_to_accidentboxes[ii].flag_foutput){
//                std::cout<<"***fmiss**************************"<<std::endl;
//                std::cout<<fmiss_to_accidentboxes[ii].missing_frame<<","<<fmiss_to_accidentboxes[ii].missing_ID<<","<<fmiss_to_accidentboxes[ii].missing_type<<","<<fmiss_to_accidentboxes[ii].x<<","<<fmiss_to_accidentboxes[ii].y<<","
//                        <<fmiss_to_accidentboxes[ii].crashone.ID_number<<","<<fmiss_to_accidentboxes[ii].crashone.typenumber<<","<<fmiss_to_accidentboxes[ii].crashone.dist_neighcheck<<","<<fmiss_to_accidentboxes[ii].flag_merge_stay1<<"," <<fmiss_to_accidentboxes[ii].crashone.boxings.width <<std::endl;
//  //                         output_file<<fmiss_to_accidentboxes[ii].missing_frame<<","<<fmiss_to_accidentboxes[ii].missing_ID<<","<<fmiss_to_accidentboxes[ii].missing_type<<","<<fmiss_to_accidentboxes[ii].x<<","<<fmiss_to_accidentboxes[ii].y<<","
//  //                                    <<fmiss_to_accidentboxes[ii].crashone.ID_number<<","<<fmiss_to_accidentboxes[ii].crashone.typenumber<<std::endl;
//                fmiss_to_accidentboxes[ii].flag_foutput=true;
//                fmissaccident_count++;
//            }
//        }



//        for(int ik=0;ik<like_accidentboxes1.size();ik++){
//            if(!like_accidentboxes1[ik].flag_loutput){
//                std::cout<<"***like1**************************"<<std::endl;
//                std::cout<<like_accidentboxes1[ik].accident_frame<<","<<like_accidentboxes1[ik].ID_number<<","
//                        <<like_accidentboxes1[ik].crashone1.ID_number<<","<<like_accidentboxes1[ik].a_sum <<std::endl;
// //                           output_file<<fmiss_to_accidentboxes[ii].missing_frame<<","<<fmiss_to_accidentboxes[ii].missing_ID<<","<<fmiss_to_accidentboxes[ii].missing_type<<","<<fmiss_to_accidentboxes[ii].x<<","<<fmiss_to_accidentboxes[ii].y<<","
// //                                     <<fmiss_to_accidentboxes[ii].crashone.ID_number<<","<<fmiss_to_accidentboxes[ii].crashone.typenumber<<std::endl;
//                like_accidentboxes1[ik].flag_loutput=true;
//                like1_count++;
//            }
//        }


    }//end of datasets_name
}












int main()
{
    float img_wideth=1920;
    float img_height=1080;
    RXY_threinit(img_wideth,img_height);
    std::cout<<r_threshold<<std::endl;
//Rnode A1;Rnode A2;Rnode B1;Rnode B2;
//A1.x=1;A1.y=1;
//A2.x=2;A2.y=1;
//B1.x=1;B1.y=2;
//B2.x=2,B2.y=2;

//float dist=RXY_distmin_dotline(A1,A2,B1);
//float dist2=RXY_distmin_twoline(A1,A2,B1,B2);

//std::cout<<dist<<"," <<dist2<<std::endl;

    std::vector<std::string> dataset_names{"AAAA-1"};
    RXY_Judgement(dataset_names);


    return 0;
}
