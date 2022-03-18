
import cv2 as cv
import numpy as np
import math
def huofu_circle(img,out):
    img0 = cv.imread(img)
    # rows,cols=planets.shape
    # print(cols)
    # print(rows)
    gay_img = cv.cvtColor(img0, cv.COLOR_BGRA2GRAY)
    img = cv.medianBlur(gay_img, 7)  # 进行中值模糊，去噪点
    cimg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    # cv.imshow("cimg", cimg)
    #11 12
    # circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 2, 300, param1=300, param2=80, minRadius=200,maxRadius=225)
    #13 14
    # circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 2, 300, param1=300, param2=80, minRadius=350,maxRadius=400)
    #205
    # 
    #30
    circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT,2, 200,param1=250, param2=110, minRadius=0,maxRadius=100)
#参数1：输入图像，参数二：方法，只有这个，参数三：累加器图像的分辨率。参数三：两个不同圆之间的最小距离。参数四：param1为用于Canny的
    #返回值为圆心，半径
    circles1 = np.uint16(np.around(circles))
    # print("1")
    # print(circles1)
    # print("2")
    for i in circles1[0, :]:  # 遍历矩阵每一行的数据
        #画圆指令
        #参数1：图像，参数2：圆心，参数3：半径，参数4：颜色，参数5：粗细
        cv.circle(img0, (i[0], i[1]), i[2], (0, 255, 0), 2)
        cv.circle(img0, (i[0], i[1]), 2, (0, 0, 255), 3)

    cv.imshow("gay_img", img0)
    cv.imwrite(out, img0)
    return circles1
#求取欧式距离
def math_dist(point,point1):
    a=len(point)
    b=len(point1)
    if a==b:
        dist_0=0
        for i in range(2):
            dist_0=dist_0+(int(point[i])-int(point1[i]))*(int(point[i])-int(point1[i]))
           
        dist_0=math.sqrt(dist_0)
        return dist_0
    else:
        print("参数错误")
def theta_solve(min_circle_point,min_circle_point1,max_circle_point):
    dist=math_dist(min_circle_point,max_circle_point)
    
    dist1=math_dist(min_circle_point1, max_circle_point)
    
    r=(dist+dist1)/2
    print("r:",r)
    dist3=math_dist(min_circle_point, min_circle_point1)
    print("dist3",dist3)
    d=dist3/2
    print("d:",d)
    theta=math.asin(d/r)
    
    theta=theta*180/math.pi
    return theta*2
if __name__ == '__main__':
    #0图和2图
    circles=huofu_circle("32.jpg", "32_1.jpg")
    circles1=huofu_circle("31.jpg", "31_1.jpg")
    print(circles)
    
    min_circle_point=np.array(circles[0,1,:])
    min_circle_point1=np.array(circles1[0,1,:])
    max_circle_point=(np.array(circles[0,0,:])+np.array(circles1[0,0,:]))/2

    theta=theta_solve(min_circle_point, min_circle_point1, max_circle_point)
    print(theta)
#1图和0图
    # circles=huofu_circle("00.jpg", "0.jpg")
    # circles1=huofu_circle("01.jpg", "1.jpg")

    # print(circles)
    # print(circles1)
    # min_circle_point=np.array(circles[0,1,:])
    # min_circle_point1=np.array(circles1[0,1,:])
    # max_circle_point=(np.array(circles[0,0,:])+np.array(circles1[0,0,:]))/2
    # print(max_circle_point)
    # theta=theta_solve(min_circle_point, min_circle_point1, max_circle_point)
    
    # print(theta)

#2图和1图
    # circles=huofu_circle("02.jpg", "2.jpg")
    # circles1=huofu_circle("01.jpg", "1.jpg")

    # print(circles)
    # print(circles1)
    # min_circle_point=np.array(circles[0,1,:])
    # min_circle_point1=np.array(circles1[0,1,:])
    # max_circle_point=(np.array(circles[0,0,:])+np.array(circles1[0,0,:]))/2
    # print(max_circle_point)
    # theta=theta_solve(min_circle_point, min_circle_point1, max_circle_point)
    
   
    # print(theta)