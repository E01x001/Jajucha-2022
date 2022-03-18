# Copyright 2020 Sanggyu Lee. All rights reserved.
# sanggyu.developer@gmail.com
import cv2
import numpy as np
from scipy.signal import argrelextrema
import traceback

if 'jajucha' in __name__:
    from jajucha.control import mtx, mtx2
else:
    from control import mtx, mtx2


class Vars:
    pass


class Camera:
    def __init__(self, mtx, h, trans, trans_inv, flip=False):
        self.f_u = f_u = mtx[0, 0]
        self.f_v = f_v = mtx[1, 1]
        if not flip:
            self.c_u = c_u = mtx[0, 2]
            self.c_v = c_v = mtx[1, 2]
        else:
            self.c_u = c_u = 639 - mtx[0, 2]
            self.c_v = c_v = 479 - mtx[1, 2]
        self.h = h
        self.M = trans @ np.array([[-h / f_u, 0., h * c_u / f_u],
                                   [0., 0., -h],
                                   [0., -1 / f_v, c_v / f_v]], dtype=np.float32)
        # if flip:
        #     self.M_inv = np.array([[-1, 0, 639],
        #                            [0., 1, 0],
        #                            [0., 0, 1]], dtype=np.float32) @ \
        #                  np.array([[f_u, c_u, 0],
        #                            [0., c_v, h * f_v],
        #                            [0., 1, 0]], dtype=np.float32) @ trans_inv
        # else:
        #     self.M_inv = np.array([[f_u, c_u, 0],
        #                            [0., c_v, h * f_v],
        #                            [0., 1, 0]], dtype=np.float32) @ trans_inv

        self.M_inv = np.array([[f_u, c_u, 0],
                               [0., c_v, h * f_v],
                               [0., 1, 0]], dtype=np.float32) @ trans_inv

    def warpImg(self, img):
        return cv2.warpPerspective(img, self.M, (500, 300))

    def unWarpPts(self, pts):
        return cv2.perspectiveTransform(np.array([pts], dtype=np.float32), self.M_inv)[0]


class LaneDetector:
    def __init__(self, cam, name=''):
        self.cam = cam
        self.explored = []
        self.name = name

    def imshow(self, name, img):
        return
        cv2.imshow(self.name + name, img)

    def findLines(self, img):
        """undistorted image => lines"""
        # Image Transformation
        l = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)[:, :, 1]
        blur = cv2.bilateralFilter(l, 7, 10, 20)
        # edge = cv2.Canny(blur, 300, 500)
        edge = cv2.Canny(blur, 200, 400)
        warp = self.cam.warpImg(edge)
        self.imshow('warp', warp)

        # Histogram Search
        histogram = np.sum(warp, axis=0)
        histogram = self.smooth(histogram, 20)
        histogram_near = np.sum(warp[270:], axis=0)
        histogram_near = self.smooth(histogram_near, 20)
        maxima, = argrelextrema(histogram, np.greater)
        maxima_near, = argrelextrema(histogram_near, np.greater)
        maxima = sorted(np.concatenate((maxima, maxima_near)))
        maxima = np.delete(maxima, np.argwhere(np.ediff1d(maxima) < 30) + 1)
        maxima = np.delete(maxima, np.where(np.isin(maxima, maxima_near)))
        maxima = sorted(maxima_near, key=lambda x: abs(x - 250)) + sorted(maxima, key=lambda x: abs(x - 250))
        # print(maxima_near, maxima)
        # Sliding Windows
        height = warp.shape[0]
        pts = warp.nonzero()
        self.explored = []
        result = []
        aux = warp.copy()
        for start_x in maxima:
            line_points = self.follow_line(height, pts, start_x, aux=aux)
            # print(line_points)
            if line_points is not None:
                line_points, centers = line_points
                line = self.cam.unWarpPts(line_points)
                centers = self.cam.unWarpPts(np.array(centers, dtype=np.float32))
                result.append((line_points, line, centers))
        self.imshow('aux', aux)
        result.sort(key=lambda x: x[0][0, 0])
        result = [u[2] for u in result]
        return result

    def follow_line(self, height, pts, start_x, windows=20, half_width=25, thresh=30, aux=None):
        # Idea from https://www.haidynmcleod.com/driving-lane-detection
        for x_range in self.explored:
            if x_range[0] < start_x < x_range[1]:
                return
        h = height // windows
        pts_y = pts[0]
        pts_x = pts[1]
        cur_x = start_x
        point_ids = []
        dx = 0
        cnt = 0
        last_x = None
        min_x = start_x
        max_x = start_x
        min_y = height
        max_y = -1
        centers = []
        skip = -1
        for window in range(windows):
            y0 = height - (window + 1) * h
            y1 = height - window * h
            x0 = cur_x - half_width
            x1 = cur_x + half_width
            if aux is not None:
                cv2.rectangle(aux, (int(x0), int(y0)), (int(x1), int(y1)),
                              (255 * (window / windows), 255 * (windows - window) / windows, 0), 2)
            pts_in_window, = ((y0 <= pts_y) & (pts_y < y1) & (x0 <= pts_x) & (pts_x < x1)).nonzero()
            point_ids.append(pts_in_window)
            if len(pts_in_window) > thresh:
                cur_x = np.mean(pts_x[pts_in_window])
                for x_range in self.explored:
                    if x_range[0] < cur_x < x_range[1]:
                        break
                centers.append((cur_x, (y0 + y1) / 2))
                if last_x is not None:
                    dx = cur_x - last_x
                last_x = cur_x
                cnt += 1
                if min_y > y0:
                    min_y = y0
                if max_y < y1:
                    max_y = y1
                if min_x > cur_x:
                    min_x = cur_x
                if max_x < cur_x:
                    max_x = cur_x
                skip = 0
            else:
                last_x = None
                cur_x += dx
                if skip >= 0:
                    skip += 1
                if skip > 2:
                    break

        point_ids = np.concatenate(point_ids)
        if len(point_ids) < 100 or cnt < 5:
            return
        x = pts_x[point_ids]
        y = pts_y[point_ids]
        try:
            fit = np.polyfit(y, x, 2)
            f = np.poly1d(fit)
            line_y = np.arange(min_y, max_y + 15, 15)
            line_x = f(line_y)
            # print(line_x)
            self.explored.append((min_x - half_width / 2, max_x + half_width / 2))
            return np.column_stack((np.array(line_x, dtype=np.int), np.array(line_y, dtype=np.int))), centers
        except:
            traceback.print_exc()
            pass

    @staticmethod
    def smooth(x, window_len=11, window='hanning'):
        # https://github.com/scipy/scipy-cookbook/blob/master/ipython/SignalSmooth.ipynb
        """smooth the data using a window with requested size.

        This method is based on the convolution of a scaled window with the signal.
        The signal is prepared by introducing reflected copies of the signal
        (with the window size) in both ends so that transient parts are minimized
        in the begining and end part of the output signal.

        input:
            x: the input signal
            window_len: the dimension of the smoothing window; should be an odd integer
            window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
                flat window will produce a moving average smoothing.

        output:
            the smoothed signal

        example:

        t=linspace(-2,2,0.1)
        x=sin(t)+randn(len(t))*0.1
        y=smooth(x)

        see also:

        numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
        scipy.signal.lfilter

        TODO: the window parameter could be the window itself if an array instead of a string
        NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
        """

        if x.ndim != 1:
            raise ValueError("smooth only accepts 1 dimension arrays.")

        if x.size < window_len:
            raise ValueError("Input vector needs to be bigger than window size.")

        if window_len < 3:
            return x

        if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
            raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")

        s = np.r_[x[window_len - 1:0:-1], x, x[-2:-window_len - 1:-1]]
        # print(len(s))
        if window == 'flat':  # moving average
            w = np.ones(window_len, 'd')
        else:
            w = eval('np.' + window + '(window_len)')

        y = np.convolve(w / w.sum(), s, mode='valid')
        return y[(window_len // 2 - 1):-(window_len // 2)]


class LightDetector:
    # idea from hevlhayt@foxmail.com
    # Date: 2016/1/15
    # Time: 19:20
    # https://github.com/HevLfreis/TrafficLight-Detector/blob/master/src/main.py
    def __init__(self, cutoff_h, name=''):
        self.cutoff_h = cutoff_h
        self.name = name

    def imshow(self, name, img):
        return
        cv2.imshow(self.name + name, img)

    def detect(self, img, aux_img):
        img = img[:self.cutoff_h]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        r1_low = np.array([0, 100, 190])
        r1_high = np.array([30, 255, 255])
        r1 = cv2.inRange(hsv, r1_low, r1_high)
        r2_low = np.array([160, 100, 190])
        r2_high = np.array([179, 255, 255])
        r2 = cv2.inRange(hsv, r2_low, r2_high)
        r_range = cv2.bitwise_or(r1, r2)

        g_low = np.array([40, 100, 160])
        g_high = np.array([100, 255, 255])
        g_range = cv2.inRange(hsv, g_low, g_high)

        kernel = np.ones((5, 5), np.uint8)
        r_range = cv2.dilate(r_range, kernel)
        r_range = cv2.bitwise_and(v, v, mask=r_range)
        kernel = np.ones((3, 3), np.uint8)
        g_range = cv2.dilate(g_range, kernel)
        g_range = cv2.bitwise_and(v, v, mask=g_range)

        red = cv2.HoughCircles(r_range, cv2.HOUGH_GRADIENT, 1, 50, param1=200, param2=12, minRadius=3, maxRadius=10)
        green = cv2.HoughCircles(g_range, cv2.HOUGH_GRADIENT, 1, 50, param1=200, param2=14, minRadius=3, maxRadius=10)

        reds, greens = [], []

        if red is not None:
            for circle in red[0]:
                x0, y0, r = np.uint16(np.around(circle))
                center = (x0, y0)
                reds.append(circle)
                cv2.circle(aux_img, center, r + 10, (0, 0, 255), 2)

        if green is not None:
            for circle in green[0]:
                x0, y0, r = np.uint16(np.around(circle))
                center = (x0, y0)
                greens.append(circle)
                cv2.circle(aux_img, center, r + 10, (0, 255, 0), 2)
        return aux_img, (reds, greens)


class BasePlanning:
    FrontCam = Camera(mtx, 81,
                      np.array([[0.5, 0., 250.],
                                [0., -0.5, 350.],
                                [0., 0., 1.]], dtype=np.float32),
                      np.array([[2., 0., -500.],
                                [0., -2., 700.],
                                [0., 0., 1.]], dtype=np.float32))
    FrontLaneDetector = LaneDetector(FrontCam, 'front')
    RearCam = Camera(mtx2, 88,
                     np.array([[0.5, 0., 250.],
                               [0, -0.5, 370.],
                               [0., 0., 1.]], dtype=np.float32),
                     np.array([[2., 0., -500.],
                               [0, -2., 740.],
                               [0., 0., 1.]], dtype=np.float32)
                     , flip=True)
    RearLaneDetector = LaneDetector(RearCam, 'rear')
    FrontLightDetector = LightDetector(int(mtx[1, 2]), 'front')
    RearLightDetector = LightDetector(int(mtx2[1, 2]), 'rear')
    # https://colorswall.com/palette/102/
    colors = [
        (0, 0, 255),
        (0, 165, 255),
        (0, 255, 255),
        (0, 128, 0),
        (255, 0, 0),
        (130, 0, 75),
        (238, 130, 238),
    ]

    def __init__(self, graphics):
        self.graphics = graphics
        self.vars = Vars()

    def linesFront(self, img):
        raise NotImplementedError()

    def linesRear(self, img):
        raise NotImplementedError()

    def objectsFront(self, img):
        raise NotImplementedError()

    def objectsRear(self, img):
        raise NotImplementedError()

    def processFront(self, img):
        lines = self.FrontLaneDetector.findLines(img)
        img2 = img.copy()
        for i, line in enumerate(lines):
            for point in line:
                cv2.circle(img2, tuple(point), 3, self.colors[i % 7], -1)
        img2, lights = self.FrontLightDetector.detect(img, img2)
        self.graphics.setFrontImage2(img2)
        return lines, lights

    def processRear(self, img):
        img2 = img.copy()
        # img = cv2.flip(img, 1)
        lines = self.RearLaneDetector.findLines(img)
        for i, line in enumerate(lines):
            for point in line:
                cv2.circle(img2, tuple(point), 3, self.colors[i % 7], -1)
        # img2, lights = self.RearLightDetector.detect(img, img2)
        self.graphics.setRearImage2(img2)
        return lines

    def process(self, time, frontImage, rearImage, frontLidar, rearLidar):
        frontLines, frontObject = self.processFront(frontImage)
        rearLines = self.processRear(rearImage)

        steer = 0
        velocity = 0
        return steer, velocity

    def canny(self, img, par1=200, par2=400):
        l = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)[:, :, 1]
        blur = cv2.bilateralFilter(l, 7, 10, 20)
        edge = cv2.Canny(blur, par1, par2)
        return edge
    
    def canny2(self, img, low_threshold, high_threshold): # Canny 알고리즘
        return cv2.Canny(img, low_threshold, high_threshold)
    
    def grayscale(self, img): # 흑백이미지로 변환
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    def gaussian_blur(self, img, kernel_size): # 가우시안 필터
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    
    def region_of_interest(self,img, vertices, color3=(255,255,255), color1=255): # ROI 셋팅
        mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
        if len(img.shape) > 2: # Color 이미지(3채널)라면 :
            color = color3
        else: # 흑백 이미지(1채널)라면 :
            color = color1
        # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
        cv2.fillConvexPoly(mask, vertices, color)
        #cv2.polylines(mask, vertices, True, (255,0,0),3)
        
        #cv2.polylines(mask, vertices, True, (0,255,0),3 )
        # 이미지와 color로 채워진 ROI를 합침
        ROI_image = cv2.bitwise_and(img, mask)
        return ROI_image

    def draw_lines(self,img, lines, color=[0, 0, 255], thickness=3): # 선 그리기
        #point_r=[]
        #point_l = []
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)


    # def give_lines(self,img,lines):
    #     point_r = []
    #     point_l = []
    #     point_all = []
    #     for line in lines:
    #         for x1,y1,x2,y2 in line:
    #     #         point_r.append(x1,y1)
    #     #         point_l.append(x2,y2)
    #     # point_all = [point_r,point_r]
    #     # return point_all
    #             print(line)
                

    def hough_lines(self,img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines)

        return line_img

    def weighted_img(self,img, initial_img, α=1, β=1., λ=0.): # 두 이미지 operlap 하기
        return cv2.addWeighted(initial_img, α, img, β, λ)


    def for_ROI(self,img):
        gray_img = self. grayscale(img) # 흑백이미지로 변환
        blur_img = self.gaussian_blur(gray_img,7 ) # Blur 효과 #원래 7정도에서도 나름 잘 됐음.

        canny_img = self.canny2(blur_img, 70, 210) # Canny edge 알고리즘
        vertices = np.array([[(0,479),(0,365), (230,270),(410,270),(639,365),(639,479)]], dtype=np.int32)

        ROI_img = self.region_of_interest(canny_img, vertices) # ROI 설정
        lines = cv2.HoughLinesP(ROI_img, 1, 1 * np.pi/180, 30, 10, 20)
        #line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        return lines
    
    #이거 조정한 거ROI_img, 1, 1 * np.pi/180, 30, 10, 20
    def wemade(self, img):
        #height = 480 / width = 640 / channel = 3
        gray_img = self. grayscale(img) # 흑백이미지로 변환
        blur_img = self.gaussian_blur(gray_img,7 ) # Blur 효과 #원래 7정도에서도 나름 잘 됐음.
        # cv2.imshow('blur', blur_img)
        # cv2.waitKey(0)
        canny_img = self.canny2(blur_img, 70, 210) # Canny edge 알고리즘
        # cv2.imshow('canny', canny_img)
        # cv2.imshow('1', self.image)
        # cv2.waitKey(0)

        # vertices = np.array([[(0,270),(639,270), (639,479),(0,479) ]], dtype=np.int32)
        vertices = np.array([[(0,479),(0,365), (230,270),(410,270),(639,365),(639,479)]], dtype=np.int32)
        #240, 310
        #(0,0) / (639,0)
        #(0,240) / (639,240) / (0,479) / (639,479)
        #(480 620)
        #print(vertices)
        ROI_img = self.region_of_interest(canny_img, vertices) # ROI 설정
        # cv2.imshow('ROI',ROI_img) # 결과 이미지 출력
        # cv2.waitKey(0) 

        hough_img = self.hough_lines(ROI_img, 1, 1 * np.pi/180, 30, 10, 20) # 허프 변환

        result = self.weighted_img(hough_img, img) # 원본 이미지에 검출된 선 overlap
        # cv2.imshow('result',result) # 결과 이미지 출력
        # cv2.waitKey(0) 
        return result

    def imshow(self, title, img):
        cv2.imshow(title, img)
        cv2.waitKey(1)