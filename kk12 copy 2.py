from re import L
from jajucha.planning import BasePlanning
from jajucha.graphics import Graphics
from jajucha.control import mtx
import cv2
import numpy as np
import time

#아래 주석은 무시
#만약에 카운트가 두번 돼서 두번 80으로 돌게 되면 초를 줄이는 코드

''' ===============================
이 코딩은 다음과 같은 특징을 가졌습니다. 코딩 완성의 우선순위는 다음과 같습니다.
참고로 이 코딩이 정답은 아닙니다!!! 여러분은 더 좋은 코딩을 만들 수 있습니다.
여기서 제시하는 코딩을 배우면서 더 적합한 알고리즘을 만드는 것이 목표입니다!!!       

(1) "차선 선택"에서
    예) if frontLines[i][0, 1] < 460: # [No 460]은 적절한 값이 아닙니다!!
    이 의미는 460이란 값이 적절한 값이 아님을 의미합니다. 이때 표시를 [No 460]이라고 하겠습니다.
    즉 [No 460]이라는 뜻은 그 행의 460이라는 값을 적절한 값으로 넣으라는 의미입니다.
    (1) 460으로 등장하는 [No 460]은 적절한 값으로 고쳐주세요.
        이 값을 적절한 값으로 찾으면 추세선의 점선이 보입니다.
    (2) 380으로 등장하는 [No 360]은 적절한 값으로 고쳐주세요.
        이 값은 가장 잘 등장하는 값으로 중앙에서 치우친 정도를 파악합니다.
    (3) 143으로 등장하는 [No 143]은 적절한 값으로 고쳐주세요.
        우차선으로 기준으로 중앙값에 해당하는 픽셀차이는 얼마일까요?
    (4) 143으로 등장하는 [No 143]은 적절한 값으로 고쳐주세요.
        좌차선을 기준으로 중앙값에 해당하는 픽셀차이는 얼마일까요?
    (5) 2으로 등장하는 [No 2]은 적절한 값으로 고쳐주세요.
        픽셀차이와 조향값 차이를 해결하는 1차식의 기울기에 해당하는 값은?
    (6) 3으로 등장하는 [No 3]은 적절한 값으로 고쳐주세요.
        픽셀차이와 조향값 차이를 해결하는 1차식의 상수항에 해당하는 값은?
    (7) 10으로 등장하는 [No 10]은 적절한 값으로 고쳐주세요.
        자주차 속력의 적절한 값은? 상수가 좋을까? 변수가 좋을까?

(2) "라이다 처리" 는 직접 여러분들이 코딩을 완성해야 합니다.

(3) "신호등 처리?"라고 표현된 곳은
    i) [No 1]은 적절한 값이 아닙니다! => 적절한 값 찾아야 합니다.
    ii) 차가 멈추어 있을 때의 코딩이 아직 없습니다. => 적절한 코딩을 완성해야 합니다.


'''

class Planning(BasePlanning):
    def __init__(self, graphics):
        super().__init__(graphics)
        # --------------------------- # 초기 변수를 설정하는 함수
        self.vars.redCnt = 0  # 빨강불 카운트 변수 설정
        self.vars.greenCnt = 5  # 녹색불 카운트 변수 설정
        self.vars.stop = True # 차가 처음에 정지해 있음을 정의한 변수 설정
        self.vars.steer = 0  # 조향값은 0인 상태로 변수 설정
        self.vars.velocity = 0  # 속도도 0인 상태로 변수 설정
        self.vars.noLine = 0
        self.vars.controls = 1
        self.vars.control_in_noline=True
        self.vars.fixedSteer=0
        self.vars.countNoLine = 0 #사용안함

    def process(self, t, frontImage, rearImage, frontLidar, rearLidar):
        """
        cv2.show
        자주차의 센서 정보를 바탕으로 조향과 속도를 결정하는 함수
        t: 주행 시점으로부터의 시간 (초)
        frontImage: 전면 카메라 캘리된 이미지(640X480)
        rearImage: 후면 카메라 캘리된 이미지
        frontLidar: 전면 거리 센서 (mm), 0은 오류를 의미함, 0<x<2000 (2m 최대거리)
        rearLidar: 후면 거리 센서 (mm), 0은 오류를 의미함
        """
        frontLines, frontObject = self.processFront(frontImage)  # 전면 카메라 이미지 처리
        # frontLines = [[x1,y1],[x2,y2],[x3,y3], … ] , [[x1,y1], [x2,y2], … ]
        # frontLines[0]이 가장 왼쪽 차선, 빨(0), 주(1), 노(2), 초(3), 파(4), 남(5), 보(6)
        rearLines= self.processRear(rearImage) # 후면 카메라 이미지 처리

        # # canny image 출력 
        canny = self.canny(frontImage)
        # self.imshow('Canny Image', canny)
       
        #hough image 출력
        #ROI=self.for_ROI(frontImage)
        ##print(ROI)
        myline = self.for_ROI(frontImage)
        #print("myline: ",myline) 
        proper_myline = []
        proper_info = []
        #0을 왼쪽, 1을 오른쪽
        for i in myline:
            #print("i : ",i)
            for j in i:
                if(j[1]<360 or j[3]<360):
                    if(j[0]<319):  
                        proper_info.append([np.abs((j[3]-j[1])/(j[2]-j[0])),0])
                    else:
                        proper_info.append([np.abs((j[3]-j[1])/(j[2]-j[0])),1])
        print(proper_info)
        
        
        
        
        
        
        
        hough = self.wemade(frontImage)
        self.imshow('hough Image', hough)
        # hough = cv2.HoughLines(canny, rho=1, theta=np.pi/180, threshold=100)
        # cv2.imshow('Hough Image', hough)
        # hough = cv2.HoughLines(canny,1,np.pi/180,100)
        # cv2.imshow('Hough Image', hough)
        #분석을 위한 y값 설정 
        x = 360
        y = 479
        while y >= 0:
            if canny[y, x] > 0:
                break
            y -= 1
        # #print('479-y=', 479-y)

        global starttime
        #신호등 처리?
        reds, greens = frontObject # reds : n*3의 크기
        # reds: numpy array([[x1,y1,반지름], [x2,y2,반지름], ...])
        if reds: # 빨간불이면 
            self.vars.redCnt += 1
        else:
            self.vars.redCnt = 0
        if greens:
            self.vars.greenCnt += 1
        else:
            self.vars.greenCnt = 0

        if self.vars.redCnt >= 6:    # 5는 임의로 넣어둠 / 직진 주행이 완성되면 그 속도에 맞춰 경험적으로 구하기 (2021.02.03)
            self.vars.greenCnt = 0     # 빨강불이면 녹색불 카운트는 0
            self.vars.stop = True  
            return 0,0
        if self.vars.greenCnt >= 2: #얘도 임의 (02.03)
            self.vars.redCnt = 0
            self.vars.stop = False
        
        # # 라이다 처리
        if 0 < frontLidar < 200:
            #print('lidar stop')
            return 0, 0      

        # 차선 선택
        center_x = mtx[0, 2]       # 이미지의 가운데, center_x=360 부근의 값을 가지게 됩니다.
        # #print ('center_x=', center_x)
        line = None
        frontLines.sort(key=lambda x:x[0, 1], reverse=True)
        #print("frontLine: ", frontLines)
        # frontLines를 정렬(x[0,1]을 기준으로 내림차순으로 정렬)
        # frontLines = [[x1,y1],[x2,y2],[x3,y3], … ] , [[x1,y1], [x2,y2], … ]
        # y1>y2>y3... : 밑의 점부터 표현
        # #print ('frontLines=', frontLines)
        for i in range(len(frontLines)):
            if frontLines[i][0, 1] < 360: # (1) [No 460]은 적절한 값이 아닙니다!! / y1, y2, y3... [No 460]보다 작은가? 
                continue              # y값이 [No 460]보다 작7으면 추세선 안하고 넘어가라(479~[No 460]까지만 추세선 찾기)
            x = frontLines[i][:, 0]    # 차선의 x값들   x=[x1, x2, x3, ...]
            y = frontLines[i][:, 1]    # 차선의 y값들   y=[y1, y2, y3, ...]
            coefficient = np.polyfit(y,x,1)    # coefficient = [a, b] (단, x = ay + b)
            ##print(coefficient) 
            line = np.poly1d(coefficient)        # line = ay + b (가장 밑에 닿은 차선의 추세선 식)
            ##print(line)
            ##print("Line = ",line)
            ##print('x=', x,'y=',y,'line=',line)
        
            if line(360) < center_x:   # (2) [No 360]은 적절한 값이 아닙니다!! / 
                      # 만약 line = a*[No 360] + b : y=[No 360]에서의 x값이 중앙보다 왼쪽이면
                line = 'left', i            # line = ('left', i) 인 class, tuple
                break
            else:                           # 만약 line = a*400 + b : y=400에서의 x값이 중앙보다 왼쪽이 아니면
                line = 'right', i           # line = ('right', i) 인 class, tuple
                break
        # #print('line', line)
        laneImage = frontImage.copy()
        if line == None:                    # line 이 없으면
            # No line
            
            self.vars.noLine += 1
            

            #print('No line found') 
            if(self.vars.noLine>=20): 
                self.vars.velocity =30
                ##print('만약')          #만약 노라인이 8번 넘게 뜨게되면
                if(self.vars.controls ==1):     #8번 넘게 뜬 후 처음 한 번만 starttime에 현재시각을 저장해줌
                    starttime = round(time.time(),2)
                    ##print("여기")    
                    ##print("starttime이 설정됨.      starttime = ", starttime)
                    self.vars.controls = 0
                if(self.vars.steer>=0):
                    ##print("까지")         #가장 최근 스티어가 왼쪽일 때 무한 루프
                    self.vars.steer = 80
                    
                    while(self.vars.control_in_noline==True): 
                        ##print("온다면")  
                        #print('right')
                        # #print(int(time.time()))
                        # #print("time.time() = ",int(time.time()), end="")
                        # #print("         starttime= " ,(starttime))
                        if(self.vars.controls == 0):

                            if(round(time.time(),2)<(starttime+1.5)):          # 노라인이 11번 뜨고 난 이후 3초가 지날때까지 스티어 고정    #1) time을 더 늘리는 방법.      #2) 다 프린트 찍어보다가 값이 확 변할 때 이상하다는 걸 눈치채고 무시하도록... ->이건 방법을 어케해야할까?
                                #print("스티어 고정중...")
                                self.vars.fixedSteer = 2
                                return self.vars.steer, self.vars.velocity  
                                
                            else:
                                #print("시간 지남. 고정 해제")               #3초가 지나면 고정 해제
                                self.vars.control_in_noline=False
                                self.vars.noLine = 0
                                self.vars.control =1 
                                return self.vars.steer, self.vars.velocity
                    return self.vars.steer, self.vars.velocity
                                
                                





                if(self.vars.steer<0):
                    self.vars.steer = -80
                    while(self.vars.control_in_noline==True):
                        #print("left")
                        ##print(int(time.time()))
                        # #print("time.time() = ",int(time.time()), end="")
                        #print("         starttime= " ,(starttime))
                        if(self.vars.controls ==0):
                            ##print("으악 왼쪽 컨트롤 들어감")
                            if(round(time.time(),2)<(starttime+1.5)):
                                #print(round(time.time(),2))
                                #print("스티어 고정중...")
                                self.vars.fixedSteer = 1
                                return self.vars.steer, self.vars.velocity    
                                
                            else:
                                #print("시간 지남. 고정 해제")
                                self.vars.control_in_noline=False
                                self.vars.noLine = 0
                                self.vars.control = 1
                                return self.vars.steer, self.vars.velocity
                    return self.vars.steer, self.vars.velocity 
                                
                                
                         
                    
            else:
                try:
                    self.vars.controls = 1
                    self.vars.fixedSteer = 0
                    
                    #print("controls이 초기화됨")
                    

                    self.vars.control_in_noline = True
                except Exception as e:
                    print("오류 발생: ",e)
              
                return self.vars.steer, self.vars.velocity





        if line[0] == 'right':              # line 이 우차선이면
            # follow right
            # left = frontLines[line[1]-1]
            line = frontLines[line[1]]    # 다시 line 재정의 
            x = line[:, 0]
            y = line[:, 1]
            coefficient = np.polyfit(y, x, 1)
            line = np.poly1d(coefficient)
            #print('Line:', line)


            differential = np.polyder(line, m = 1)
            line +=differential
            #미분 한 값을 더해주기
            self.vars.noLine = 0

            e = line(360) - center_x - 200  # (2) [No 360]과 (3) [No 143]은 적절한 값이 아닙니다!!
            # 추세선 line = a*[No 360] + b 에서 line([No 360])은 추세선에서 y=[No 360]일 때의 x의 값을 의미하며
            # line([No 360])값이 중앙값(center_x)의 값보다 몇 픽셀 커야 자주차는 중앙에 위치한 것일까요?
            # 이 몇 픽셀이 바로 [No 143]에 해당하며 적절한 값을 찾아주세요.
            ##print('line(360)', line(360)) # (2) [No 360]의 적절한 값을 넣어주세요.
            ##print('center_x=', center_x, 'e=', e)
            for i in range(200, 480, 20):
                cv2.circle(laneImage, (int(line(i)), i), 5, (255, 0, 0), -1)
            # cv2.circle(frontImage, (int(f(400)), 400), 5, (255, 0, 0), -1)
            #cv2.imshow('Front Lane Image', laneImage)
            
        else:
            # follow left
            line = frontLines[line[1]]

            x = line[:, 0]
            y = line[:, 1]
            coefficient = np.polyfit(y, x, 1)
            line = np.poly1d(coefficient)

            differential = np.polyder(line, m = 1)
            line -=differential
            #미분 한 값을 더해주기


            e = line(360) - center_x + 150  # (2) [No 360]과 (4) [No 143]은 적절한 값이 아닙니다!!
            self.vars.noLine = 0
            # 추세선 line = a*[No 360] + b 에서 line([No 360])은 추세선에서 y=[No 360]일 때의 x의 값을 의미하며
            # line([No 360])값이 중앙값(center_x)의 값보다 몇 픽셀 작아야 자주차는 중앙에 위치한 것일까요?
            # 이 몇 픽셀이 바로 [No 143]에 해당하며 적절한 값을 찾아주세요.
            ##print('line(360)', line(360)) # (2) [No 360]의 적절한 값을 넣어주세요.
            ##print('center_x=', center_x, 'e=', e)
            for i in range(200, 480, 20):
                cv2.circle(laneImage, (int(line(i)), i), 5, (255, 0, 0), -1)
            # cv2.circle(frontImage, (int(f(400)), 400), 5, (255, 0, 0), -1)
            #cv2.imshow('Front Lane Image', laneImage)
        
        steer =  0.5 * e  -3
        if self.vars.fixedSteer==1:
            steer=-80
        elif self.vars.fixedSteer==2:
            steer=80

        ##print('steer=', steer)
            
        
        # (5) [No 2]와 (6) [No 3]은 적절한 값이 아닙니다. steer는 조향값이며, e는 픽셀의 차이입니다.
        # 픽셀과 조향값과는 어떤 관계가 있을까요? 이 관계를 가장 적절한 1차식으로 만들어 봅시다.
        # 참고로 조향값 steer는 -80~80 을 추천합니다. [-143~143] 사이값은 이론적인 값이며 실제 이 값을 넣으면
        # 기계에 무리가 옵니다. 절대 넣지 않기를 조언합니다.
        velocity = 50  # [No 10]은 적절한 값이 아닐 수 있습니다. velocity는 속도입니다.
        # -50 ~ 50의 값을 추천합니다. [-360 ~ 360] 사이값은 이론적인 값이며
        # 실제 이 값을 넣으면 기계에 무리가 옵니다. 절대 넣지 않기를 조언합니다.
        # 이 velocity를 e값의 1차 함수로 넣을 수도 있습니다.

        # cv2.waitKey(1)

        if (self.vars.steer < 90 or self.vars.steer > -90):
            self.vars.steer = steer
            self.vars.velocity = velocity
            return self.vars.steer, self.vars.velocity
        else:
            return self.vars.steer, self.vars.velocity

        



if __name__ == "__main__":
    g = Graphics(Planning) # 자주차 컨트롤러 실행
    g.root.mainloop() # 클릭 이벤트 처리
    g.exit() # 자주차 컨트롤러 종료