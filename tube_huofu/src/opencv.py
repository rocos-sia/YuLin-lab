
import cv2 as cv
import numpy as np
img0 = cv.imread("30.jpg")
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
#参数1：输入图像，参数二：方法，只有这个，参数三：累加器图像的分辨率。参数三：两个不同圆之间的最小距离。参数四：param1为用于Canny的边缘阀值上限，下限被置为上限的一半。
#参数五：累加器的阀值。 参数六：最小圆半径。参数七：最大圆半径
#返回值为圆心，半径
circles1 = np.uint16(np.around(circles))
print("1")
print(circles1)
print("2")
for i in circles1[0, :]:  # 遍历矩阵每一行的数据
    #画圆指令
    #参数1：图像，参数2：圆心，参数3：半径，参数4：颜色，参数5：粗细
    cv.circle(img0, (i[0], i[1]), i[2], (0, 255, 0), 2)
    cv.circle(img0, (i[0], i[1]), 2, (0, 0, 255), 3)

cv.imshow("gay_img", img0)
cv.imwrite('30_1.jpg', img0)
cv.waitKey(0)
cv.destroyAllWindows()

