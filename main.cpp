#include "mybox.h"
#include "getline.h"
#include "distance.h"
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
        bbbox.init(label[4],label[5],label[6],label[7],label[3],label[1],label[8],label[9],label[10],label[11],label[0]);
//输入文档格式：frame,ID,___,class,x,y,W,H,ax,ay,vx,vy
            bbox_per_frame.emplace_back(bbbox);

    }
    // 最后一帧
    bbox.push_back(bbox_per_frame);
    return bbox;
}

/*1.判断前后帧之间是否有ID消失，若有，返回ID，type,frame;*/
std::vector<missingbox> RXY_findmissingbox(std::vector<mybox>& detections1,std::vector<mybox>& detections2){
     //missingbox miss_boxes;
    std::vector<missingbox> miss_boxes;//一帧中消失的所有对象
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
     if(miss_box.missing_ID!=it_detections->ID_number){
         if(dist<r_threshold){
            neighbourboxes.push_back(*it_detections);
         }
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
        else if(it_like_acc->like_check==like_check_thre){it_like_acc->like_check++;miss_to_accidentboxes.push_back(*it_like_acc);it_like_acc->flag_accident=true;}//
        else{
            for(it_like_acc=like_accidentboxes.begin();it_like_acc<like_accidentboxes.end();++it_like_acc){
                    it_like_acc->flag_merge=false;
                    if(it_like_acc->like_check>like_check_thre){continue;}
                    else if(it_like_acc->like_check==like_check_thre){it_like_acc->like_check++;miss_to_accidentboxes.push_back(*it_like_acc);it_like_acc->flag_accident=true;}
                    else{
                    for(it_det=detections.begin();it_det<detections.end();++it_det){
                        if(it_like_acc->missing_ID==it_det->ID_number){
                            it_like_acc->flag_merge=true;
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
                                if(it_like_acc->like_check<like_check_thre)//疑似事故对象检测帧
                                  {(*it_like_acc).like_check++;}
                                else{miss_to_accidentboxes.push_back(*it_like_acc);it_like_acc->flag_accident=true;}//如果超过检查次数，则判为发生事故
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
    }
}

//再次检查,对于事故车的撞击对象,是否出现并移动,若是,未出事故;若否,出事故
void RXY_neighcheck_accident(const std::vector<mybox>&detections,std::vector<missingbox>&miss_to_accident,std::vector<missingbox>&fmiss_to_accident){
    for(int i=0;i<miss_to_accident.size();i++){
    if(miss_to_accident[i].flag_accident&&(!miss_to_accident[i].neigh_change)){
        miss_to_accident[i].flag_merge_acci=false;
        if(miss_to_accident[i].crashone.neigh_check>neigh_check_thre){continue;}
        //check==neigh_check,*出事故*
        else if(miss_to_accident[i].crashone.neigh_check==neigh_check_thre){miss_to_accident[i].crashone.neigh_check++;fmiss_to_accident.push_back(miss_to_accident[i]);}
        else{
        for(int j=0;j<detections.size();j++){
            //crashone是否出现
            if(miss_to_accident[i].crashone.ID_number==detections[j].ID_number){
                miss_to_accident[i].flag_merge_acci=true;//对象出现
                float dx=fabs(miss_to_accident[i].crashone.boxings.x-detections[j].boxings.x);
                float dy=fabs(miss_to_accident[i].crashone.boxings.y-detections[j].boxings.y);
                float dsum=fabs(dx*dx+dy*dy);
              //crashone是否移动
                if(dsum>(miss_to_accident[i].crashone.boxings.width)*0.7){miss_to_accident[i].flag_accident=false;miss_to_accident[i].neigh_change=1;break;}
                 //未移动,未超过检查次数,check++;超过检查次数,*出事故*
                else{
                    if(miss_to_accident[i].crashone.neigh_check<neigh_check_thre){miss_to_accident[i].crashone.neigh_check++;}
                    else{fmiss_to_accident.push_back(miss_to_accident[i]);}
                    break;
                }
            }
        }
        //该帧内crashone没出现过,未超过检查次数,check++;超过检查次数,*出事故*
        if(!miss_to_accident[i].flag_merge_acci){
            if(miss_to_accident[i].crashone.neigh_check<neigh_check_thre){miss_to_accident[i].crashone.neigh_check++;}
            else{fmiss_to_accident.push_back(miss_to_accident[i]);}
        }
      }
    }
  }
}

//速度修正,对vy进行修正,vsum_filter
void RXY_Vfix_faccident(std::vector<missingbox>& fmiss_to_accident){
     std::vector<missingbox>::iterator it_fmiss;
     //y方向速度修正比例系数, y=520处l_filter=1
     for(it_fmiss=fmiss_to_accident.begin();it_fmiss<fmiss_to_accident.end();++it_fmiss){
         if(it_fmiss->y<520){
             it_fmiss->k_vfilter=0.1055;
             it_fmiss->b_vfilter=-25.9025;
             //修正系数
             it_fmiss->l_vfilter=(it_fmiss->k_vfilter)*(it_fmiss->y)+it_fmiss->b_vfilter;
             it_fmiss->vy_filter=(it_fmiss->vy)/(it_fmiss->l_vfilter);
             it_fmiss->vsum_filter=sqrt(it_fmiss->vy_filter*it_fmiss->vy_filter+it_fmiss->vx*it_fmiss->vx);
         }
         else {
             it_fmiss->k_vfilter=0.0979;
             it_fmiss->b_vfilter=-72.9635;
             it_fmiss->l_vfilter=(it_fmiss->k_vfilter)*(it_fmiss->y)+it_fmiss->b_vfilter;
             it_fmiss->vy_filter=(it_fmiss->vy)/(it_fmiss->l_vfilter);
             it_fmiss->vsum_filter=sqrt(it_fmiss->vy_filter*it_fmiss->vy_filter+it_fmiss->vx*it_fmiss->vx);
         }

     }

}

//速度滤波,将低速的fmiss_to_accident去掉
std::vector<missingbox> RXY_Vfilter_faccident(const std::vector<missingbox>& fmiss_to_accident){
    std::vector<missingbox>::const_iterator it_fmiss;
    std::vector<missingbox> vfilter_fmiss;
    for(it_fmiss=fmiss_to_accident.begin();it_fmiss<fmiss_to_accident.end();++it_fmiss){
        if(it_fmiss->vsum_filter>20){
            vfilter_fmiss.push_back(*it_fmiss);
        }
    }
    return vfilter_fmiss;
}




//    return miss_to_accidentboxes;

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
        std::string label_path = "/home/ruanxinyao/lastyear_project/AAAA-1/samples/bcarcar1.txt"; //"/home/ruanxinyao/lastyear_project/Judgement1/data/"+dataset_name+"/det.txt";//输入txt路径
        std::ifstream label_file(label_path);//定义ifstream对象label_file，路径为label_path
        if (!label_file.is_open()) {
            std::cerr << "Could not open or find the label!!!" << std::endl;
//            return -1;
        }
        std::vector<std::vector<mybox>> all_detections = RXY_processLabel(label_file);
        label_file.close();

        std::string output_path = "/home/ruanxinyao/lastyear_project/Judgement1/output/"+dataset_name+"carcar1.txt";
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
        std::vector<int> like;
        int missaccident_count=0;//总计消失对象的事故个数
        int fmissaccident_count=0;
        int nomissaccident_count=0;//总计未消失对象的事故个数
        int like_count=0;
        size_t total_frame=all_detections.size();
        std::vector<missingbox>miss_to_accidentboxes;//确认为事故的ID（有消失）
        std::vector<missingbox>fmiss_to_accidentboxes;//经过neighbourcheck后的miss_to_accidentboxes
        std::vector<mybox>nomiss_to_accidentboxes;//确认为事故的ID（无消失）
        std::vector<missingbox>like_accidentboxes;//对于消失的ID，通过前后两帧对比找出的疑似事故对象
        std::vector<missingbox> miss_boxes;
        std::vector<mybox> neighbourboxes;
        std::vector<missingbox> countedlike_boxes;//存放已经统计过的疑似对象
        bool flag_ID_missing=true;
        float mindistance=0;
        for(size_t im=0;im<total_frame-1;im++)//0为第1帧
        {
            const auto &detections=all_detections[im];//一帧
            miss_boxes=RXY_findmissingbox(all_detections[im],all_detections[im+1]);
//疑似对象like_accidentboxes,根据本身是否出现并移动,第一次筛选-->miss_to_accidentboxes
            RXY_morecheck(detections,like_accidentboxes,miss_to_accidentboxes);
//miss_to_accidentboxes,根据相撞对象是否出现并移动,第二次筛选-->fmiss_to_accidentboxes
            //问题??????miss_box--(im-check)帧;crashone检测从im开始
            RXY_neighcheck_accident(all_detections[im],miss_to_accidentboxes,fmiss_to_accidentboxes);

//            like_accidentboxes.clear();
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
                        //计算最近距离,记录相撞对象
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
//输出文档
//     std::vector<int>::iterator it_id_check;
//     std::cout<<"***nomissing accidentboxes***"<<std::endl;
//       for(auto &nomtacci : nomiss_to_acciden1031tboxes){
//           std::cout<<nomtacci.accident_frame<<","<<nomtacci.ID_number<<","<<nomtacci.boxings.x<<","<<nomtacci.boxings.y<<std::endl;
//           output_file<<nomtacci.accident_frame<<","<<nomtacci.ID_number<<","<<nomtacci.boxings.x<<","<<nomtacci.boxings.y<<std::endl;
//           for(it_id_check=id_check.begin();it_id_check<id_check.end();++it_id_check){
//               if((*it_id_check)==nomtacci.ID_number){nomtacci.flag_cout=true;break;}
//           }
//           if(!nomtacci.flag_cout){id_check.push_back(nomtacci.ID_number);nomissaccident_count++;}
//       }

       for(int ib=0;ib< miss_to_accidentboxes.size();ib++){
           if(!miss_to_accidentboxes[ib].flag_output){
           std::cout<<"***missing accidentboxes***"<<std::endl;
           std::cout<<miss_to_accidentboxes[ib].missing_frame<<","<<miss_to_accidentboxes[ib].missing_ID<<","<<miss_to_accidentboxes[ib].missing_type<<","<<miss_to_accidentboxes[ib].x<<","<<miss_to_accidentboxes[ib].y<<std::endl;
          //输出格式:frame,ID,class,x,y
//           output_file<<miss_to_accidentboxes[i].missing_frame<<","<<miss_to_accidentboxes[i].missing_ID<<","<<miss_to_accidentboxes[i].missing_type<<","<<miss_to_accidentboxes[i].x<<","<<miss_to_accidentboxes[i].y<<std::endl;
           miss_to_accidentboxes[ib].flag_output=true;
           missaccident_count++;
           std::cout<<"****crashone boxes****"<<std::endl;
           std::cout<<miss_to_accidentboxes[ib].crashone.ID_number<<","<<miss_to_accidentboxes[ib].crashone.typenumber<<","<<miss_to_accidentboxes[ib].crashone.boxings.x<<","<<miss_to_accidentboxes[ib].crashone.boxings.y<<std::endl;
           }
//           for(it_id_check=id_check.begin();it_id_check<id_check.end();++it_id_check){
//               if((*it_id_check)==miss_to_accidentboxes[i].missing_ID){miss_to_accidentboxes[i].flag_cout=true;break;}
//           }
//           if(!miss_to_accidentboxes[i].flag_cout){id_check.push_back(miss_to_accidentboxes[i].missing_ID);nomissaccident_count++;}
       }
//       std::vector<int>::iterator it_like;


       for(int ii=0;ii<fmiss_to_accidentboxes.size();ii++){
           if(!fmiss_to_accidentboxes[ii].flag_foutput){
               std::cout<<"***fmiss****"<<std::endl;
               std::cout<<fmiss_to_accidentboxes[ii].missing_frame<<","<<fmiss_to_accidentboxes[ii].missing_ID<<","<<fmiss_to_accidentboxes[ii].missing_type<<","<<fmiss_to_accidentboxes[ii].x<<","<<fmiss_to_accidentboxes[ii].y<<","
                       <<fmiss_to_accidentboxes[ii].crashone.ID_number<<","<<fmiss_to_accidentboxes[ii].crashone.typenumber<<std::endl;
               output_file<<fmiss_to_accidentboxes[ii].missing_frame<<","<<fmiss_to_accidentboxes[ii].missing_ID<<","<<fmiss_to_accidentboxes[ii].missing_type<<","<<fmiss_to_accidentboxes[ii].x<<","<<fmiss_to_accidentboxes[ii].y<<","
                         <<fmiss_to_accidentboxes[ii].crashone.ID_number<<","<<fmiss_to_accidentboxes[ii].crashone.typenumber<<std::endl;
               fmiss_to_accidentboxes[ii].flag_foutput=true;
               fmissaccident_count++;
           }
       }

//统计所有的疑似对象个数
       for(auto &likeab : like_accidentboxes ){
           if(!like_accidentboxes.empty()){
               if(!likeab.flag_cout){
                 std::cout<<"******like_accidentboxes***"<<std::endl;
                 std::cout<<im<<","<<likeab.missing_frame<<","<<likeab.missing_ID<<","<<likeab.missing_type<<std::endl;
                 likeab.flag_cout=true;
//                 countedlike_boxes.push_back(likeab);
                 like_count++;
             }

//           for(it_like=like.begin();it_like<like.end();++it_like){
//               if((*it_like)==likeab.missing_ID){likeab.flag_cout=true;break;}
//           }
//           if(!likeab.flag_cout){like.push_back(likeab.missing_ID);like_count++;}
       }
}

                    miss_boxes.clear();//不储存每一帧消失的对象
                    neighbourboxes.clear();
//                  miss_to_accidentboxes.clear();
//                  nomiss_to_accidentboxes.clear();



       }//end of each frame




        std::cout<<"******事故对象总计******"<<std::endl;
        std::cout<<"nomissingaccident_count:"<<nomissaccident_count<<std::endl;
        std::cout<<"missingaccident_count:"<<missaccident_count<<std::endl;
        std::cout<<"fmissaccident_count:"<<fmissaccident_count<<std::endl;
        std::cout<<"total accidentID_count:"<<missaccident_count+nomissaccident_count<<std::endl;
        std::cout<<"like_count:"<<like_count<<std::endl;


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
