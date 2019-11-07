# 视频组----事故判断部分
重复以下过程：<br>
１）判断前后帧之间是否有ID消失，若有，返回ID，类别和帧数;并记录消失ID的体积大小，和位置（x,y）;<br>
### 注意<br>
　　该ID消失是在卡尔曼滤波以后的消失,也就是说,当视频中出现某一帧后面该ID消失,输出的txt中该ID仍存在并有一个预测位置,若X帧以后该ID仍不出现,输出文档中该ID才消失.<br>
                                    　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　                                   
２）若有消失，获得消失前一帧的物体类别和大小，判断是否小于对应体积阈值;消失前一帧物体的位置是否在图片边缘且行驶方向为远离图片行驶;<br>
３）若是，无事故;<br>
　　若否，寻找出在消失前一帧以ID中心点（x，y）为中心 r 距离区域内的所有ID，对每个ID寻找其当量直线，计算每个当量直线和消失ID的当量直线之间的最小距离;<br>
４）比较最小距离和设定的阈值，若小于阈值，疑似碰撞;<br>
　　　　　　　　　　　　　　　若大于阈值，无事故;<br>
                                                                                                                                    
　　若无消失，寻找各物体是否有加速度突变，若有，将加速度突变值与加速度阈值比较，若大于，认为发生碰撞;<br>
　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　若小于，无事故;<br>
　　　　　　　　　　　　　　　　　　　　　若否，无事故;<br>
５）对于疑似碰撞的对象，在后10帧中做检测，检查是否出现，若出现,出现坐标和原坐标是否有较大的变动;<br>
　　若变动超过阈值,认为正常<br>
　　若不超过阈值,认为出事故;<br>
　　　　　　　　　　　　　　　　　　　　　　　　　　　　若10帧都不出现,则认为出事故;<br>
                                                                                          
