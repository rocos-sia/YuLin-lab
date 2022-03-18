该思路等待研究中，目前失败
涉及库的安装教程网址
http://www.javashuo.com/article/p-kaqhawpe-kt.html




build 里面放图片
orb_cv 以及orb_self 为特征匹配函数
pose_es..为求相机的运动
调用格式
举例，1.jpg 2.jpg 特征匹配
cd build 
./orb_cv 1.jpg 2.jpg
或者
./orb_self #在里面更改图片
通过图片判断相机旋转角度
./pose_estimation_2d2d 1,jpg 2.jpg
