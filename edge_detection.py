import cv2 # opencv 사용
import numpy as np

def grayscale(img): # 흑백이미지로 변환
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img, low_threshold, high_threshold): # Canny 알고리즘
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size): # 가우시안 필터
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices, color3=(255,255,255), color1=255): # ROI 셋팅

    mask = np.zeros_like(img) # mask = img와 같은 크기의 빈 이미지
    
    if len(img.shape) > 2: # Color 이미지(3채널)라면 :
        color = color3
    else: # 흑백 이미지(1채널)라면 :
        color = color1
        
    # vertices에 정한 점들로 이뤄진 다각형부분(ROI 설정부분)을 color로 채움 
    cv2.fillConvexPoly(mask, vertices, color)
    
    # 이미지와 color로 채워진 ROI를 합침
    ROI_image = cv2.bitwise_and(img, mask)
    return ROI_image

def draw_lines(img, lines, color=[0, 0, 255], thickness=2): # 선 그리기
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap): # 허프 변환
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)

    return line_img

def weighted_img(img, initial_img, α=1, β=1., λ=0.): # 두 이미지 operlap 하기
    return cv2.addWeighted(initial_img, α, img, β, λ)

image = cv2.imread('E:\jajucha\image\straight.jpg') # 이미지 읽기 =============== 자신의 사진 주소 삽입 ===============

height, width = image.shape[:2] # 이미지 높이, 너비
#height = 480 / width = 640 / channel = 3
gray_img = grayscale(image) # 흑백이미지로 변환


    
blur_img = gaussian_blur(gray_img, 7) # Blur 효과 #원래 7정도에서도 나름 잘 됐음.
cv2.imshow('blur', blur_img)
# cv2.waitKey(0)
canny_img = canny(blur_img, 70, 210) # Canny edge 알고리즘
cv2.imshow('canny', canny_img)
cv2.imshow('1', image)
# cv2.waitKey(0)

vertices = np.array([(0,365),(0,479), (639,479),(639,365), (410, 270), (230, 270) ], dtype=np.int32)
#240, 310

#(0,0) / (639,0)
#(0,240) / (639,240) / (0,479) / (639,479)
#(480 620)
print(vertices)
ROI_img = region_of_interest(canny_img, vertices) # ROI 설정
cv2.imshow('ROI',ROI_img) # 결과 이미지 출력
cv2.waitKey(0) 

hough_img = hough_lines(canny_img, 1, 1 * np.pi/180, 30, 10, 20) # 허프 변환

result = weighted_img(hough_img, image) # 원본 이미지에 검출된 선 overlap
cv2.imshow('result',result) # 결과 이미지 출력
cv2.waitKey(0) # 창 열린 채 유지
cv2.destroyAllWindows # 열린 모든 창 닫기