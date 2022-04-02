# Copyright 2020 Sanggyu Lee. All rights reserved.
# sanggyu.developer@gmail.com
if 'jajucha' in __name__:
    from jajucha import communication
    from jajucha import config
else:
    import communication
    import config
import threading
import traceback
import cv2
import numpy as np
import tkinter
import tkinter.messagebox
import time
import os
import logging
import glob
import math

logger = logging.getLogger("control")
logger.setLevel(logging.DEBUG)
log_handler = logging.StreamHandler()
logger.addHandler(log_handler)

#
# imgpoints = []
# objpoints = []
# objp = np.zeros((9*7, 3), np.float32)
# objp[:,:2] = np.mgrid[0:9, 0:7].T.reshape(-1, 2)
# objp *= 20

mtx = np.array([[309.07332417, 0., 324.4646727],
                [0., 309.49421445, 225.88178544],
                [0., 0., 1.]], dtype=np.float32)
dist = np.array([[-0.28743189, 0.07504807, -0.00050962, 0.00069096, -0.00815296]], dtype=np.float32)
# newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))

# imgpoints2 = []
# objpoints2 = []
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

mtx2 = np.array([[395.52784006, 0., 239.21418142],
                 [0., 395.130504, 230.42492257],
                 [0., 0., 1.]])
dist2 = np.array([[-0.31976223, 0.10018421, -0.00051188, -0.00091223, -0.01348978]])


# newcameramtx2, roi = cv2.getOptimalNewCameraMatrix(mtx2, dist2, (640, 480), 0, (640, 480))

class BaseControl:
    def __init__(self, graphics):
        self.graphics = graphics
        self.done = False

    def quit(self):
        self.done = True
        # clear All Ongoing Connection and stop

    def hang(self):
        pass


class DriveControl(BaseControl):
    def __init__(self, graphics, address):
        super().__init__(graphics)
        self.address = address
        self.receiver = communication.VideoStreamSubscriber(*config.image_address)
        self._halt = False
        self._fImg = None
        self._rImg = None
        self._fLdr = None
        self._rLdr = None
        self._live = False
        self._save = False
        self._saveDir = None
        self._startTime = None
        self._thread = threading.Thread(target=self._run, args=())
        self._thread.daemon = True
        self._thread.start()

    def _run(self):
        self.client = None
        try:
            self.client = communication.Client('tcp://%s:%d' % config.address)
            msg, front, rear = self.receiver.receive()
            if self.client.id is None:
                self.graphics.setCommandText('자주차가 사용중입니다.')
                if tkinter.messagebox.askyesno('연결 오류', '자주차가 사용중입니다.\n강제로 연결할까요?'):
                    if self.client.override():
                        self.graphics.btnStartStop['state'] = tkinter.NORMAL
                        self.graphics.setCommandText('자주차에 연결되었습니다.')
                    else:
                        tkinter.messagebox.showinfo('연결 오류', '강제 연결에 실패했습니다.\n 자주차를 재부팅 해주세요.')

            else:
                self.graphics.btnStartStop['state'] = tkinter.NORMAL
                self.graphics.setCommandText('자주차에 연결되었습니다.')
            while not self._halt:
                self._fImg = cv2.imdecode(np.frombuffer(front, dtype='uint8'), -1)
                self._rImg = cv2.imdecode(np.frombuffer(rear, dtype='uint8'), -1)
                self._rImg = cv2.rotate(self._rImg, cv2.ROTATE_180)
                _t, self._fLdr, self._rLdr = msg.split()
                _t, self._fLdr, self._rLdr = float(_t), int(self._fLdr), int(self._rLdr)

                # logger.debug("latency: %.3f"%(time.time()-_t))
                self.graphics.setFrontLidar(self._fLdr)
                self.graphics.setRearLidar(self._rLdr)
                if self._live:
                    t = time.time() - self._startTime
                else:
                    t = 0
                self.t = t
                if self._live and self._save:
                    prefix = "%06.2f %d %d" % (t, self._fLdr, self._rLdr)
                    cv2.imwrite(prefix + ',front.jpg', self._fImg)  # Replace Unix Time with time since run
                    cv2.imwrite(prefix + ',rear.jpg', self._rImg)
                    logging.debug(prefix)

                self._fImg = cv2.undistort(self._fImg, mtx, dist, None, None)
                self._rImg = cv2.undistort(self._rImg, mtx2, dist2, None, None)
                self._rImg = cv2.flip(self._rImg, 1)
                self.graphics.setFrontImage1(self._fImg)
                self.graphics.setRearImage1(self._rImg)
                try:
                    command = self.graphics.plan.process(self.t, self._fImg, self._rImg, self._fLdr, self._rLdr)
                except Exception as e:
                    self.graphics.setCommandText(f'런타임 에러: {str(e)}')
                    print(str(e))
                    traceback.print_exc()
                    command = 0, 0
                try:
                    steer, velocity = command
                    assert math.isfinite(steer)
                    assert math.isfinite(velocity)
                except:
                    logger.warning("Traceback:\n  " + str(command) + " is not a valid return for process()")
                    command = 0, 0
                self.graphics.setCommand(*command)
                self.graphics.setTime(t)
                if self._live:
                    rtn = self.client.sendCommand(*command)
                    if not rtn:
                        tkinter.messagebox.showerror('연결 오류', '자주차 연결이 강제로 해제되었습니다.')
                        raise RuntimeError('Connection Reset')
                elif self.client.id is not None:
                    rtn = self.client.sendCommand(0, 0)
                    if not rtn:
                        tkinter.messagebox.showerror('연결 오류', '자주차 연결이 강제로 해제되었습니다.')
                        raise RuntimeError('Connection Reset')

                msg, front, rear = self.receiver.receive()
            # print("Out of While Loop")
            self.client.disconnect()
            # self.client.quit()
        except TimeoutError:
            if not self._halt:
                self.graphics.setCommandText("자주차 연결에 실패했습니다.")
        except RuntimeError:
            if not self._halt:
                self.graphics.setCommandText("자주차 연결이 강제로 해제되었습니다.")
        except Exception as e:
            print("Error in Driving:", e)
            traceback.print_exc()
        finally:
            if not self._halt:
                self.graphics.btnStartStop['state'] = tkinter.DISABLED
                self.graphics.btnDriveSave['state'] = tkinter.NORMAL
                self.graphics.callback_change_save()
                self.graphics.btnModeDrive['state'] = tkinter.NORMAL
                self.graphics.btnModeImage['state'] = tkinter.NORMAL
            self.receiver.close()

    def start(self):
        if self.graphics.runSave.get() and self.graphics.saveLocation is not None:
            self._save = True
            os.chdir(self.graphics.saveLocation)
            if len(glob.glob("*.jpg")) > 0:
                if not tkinter.messagebox.askyesno("주행 저장 실패", "저장 경로에 이미지 파일이 있어 저장할 수 없습니다.\n주행을 계속할까요?"):
                    return False
                else:
                    tkinter.messagebox.showinfo("주행 안내", "주행을 시작합니다.\n이미지를 저장하지 않습니다.")
                    self._save = False
        else:
            self._save = False
        self.graphics.btnDriveSave['state'] = tkinter.DISABLED
        self.graphics.btnFindSavePath['state'] = tkinter.DISABLED
        self.graphics.btnModeDrive['state'] = tkinter.DISABLED
        self.graphics.btnModeImage['state'] = tkinter.DISABLED
        self._startTime = time.time()
        self._live = True

    def stop(self):
        self._live = False
        time.sleep(0.1)
        self.client.sendCommand(0, 0)
        # 2) gracefully disconnect
        self.graphics.btnDriveSave['state'] = tkinter.NORMAL
        self.graphics.callback_change_save()
        self.graphics.btnModeDrive['state'] = tkinter.NORMAL
        self.graphics.btnModeImage['state'] = tkinter.NORMAL

    def quit(self):
        super().quit()
        self._halt = True
        # self.hang()

    def hang(self):
        self._thread.join(1)


class ImageControl(BaseControl):
    def __init__(self, graphics, path):
        super().__init__(graphics)
        self.path = path
        os.chdir(path)
        self.imList = glob.glob('*.jpg')
        self.len = len(self.imList) // 2
        if self.len < 1:
            self.graphics.setCommandText('경로에 이미지가 없습니다. 경로를 확인해주세요.')
            return
        self.graphics.setCommandText('프레임 %d장을 확인했습니다.' % self.len)
        self.index = 0
        self._fImg = None
        self._rImg = None
        self._fLdr = None
        self._rLdr = None
        self.t = 0
        self._play = False
        self._thread = None
        self.update(self.index)
        self.graphics.btnStartStop['state'] = tkinter.NORMAL
        self.setBtnStatus(tkinter.NORMAL)

    def play(self):
        self._play = True
        self._thread = threading.Thread(target=self._run, args=())
        self._thread.daemon = True
        self._thread.start()

    def pause(self):
        self._play = False

    def _run(self):
        self.setBtnStatus(tkinter.DISABLED)
        while self._play and self.index < self.len - 1:
            start = time.time()
            self.index += 1
            self.update(self.index)
            nextT = self.getTime(self.index + 1)
            end = time.time()
            if nextT is not None:
                time.sleep(max(nextT - self.t - end + start, 0))
        if self._play:
            self.graphics.callback_start_stop()
        self.setBtnStatus(tkinter.NORMAL)

    def next(self):
        self.index = min(self.index + 1, self.len - 1)
        self.update(self.index)

    def before(self):
        self.index = max(self.index - 1, 0)
        self.update(self.index)

    def next10(self):
        self.index = min(self.index + 10, self.len - 1)
        self.update(self.index)

    def before10(self):
        self.index = max(self.index - 10, 0)
        self.update(self.index)

    def update(self, index):
        try:
            self._fImg = cv2.imread(self.imList[2 * index])
            self._rImg = cv2.imread(self.imList[2 * index + 1])
            msg = self.imList[2 * index].split(',')[0]
            self.t, self._fLdr, self._rLdr = msg.split()
            self.t, self._fLdr, self._rLdr = float(self.t), int(self._fLdr), int(self._rLdr)

            self._fImg = cv2.undistort(self._fImg, mtx, dist, None, None)
            self._rImg = cv2.undistort(self._rImg, mtx2, dist2, None, None)
            self._rImg = cv2.flip(self._rImg, 1)
            self.graphics.setFrontImage1(self._fImg)
            self.graphics.setRearImage1(self._rImg)

            self.graphics.setFrontLidar(self._fLdr)
            self.graphics.setRearLidar(self._rLdr)

            self.graphics.setTime(self.t)
            try:
                command = self.graphics.plan.process(self.t, self._fImg, self._rImg, self._fLdr, self._rLdr)
            except Exception as e:
                self.graphics.setCommandText(f'런타임 에러: {repr(e)}')
                traceback.print_exc()
                command = 0, 0
            try:
                steer, velocity = command
                assert math.isfinite(steer)
                assert math.isfinite(velocity)
            except:
                error = ValueError(repr(command) + ' is not a valid return for process()')
                self.graphics.setCommandText(f'런타임 에러: {repr(error)}')
                traceback.print_exception(ValueError, error, None)
                command = 0, 0
            self.graphics.setCommand(*command)
        except Exception as e:
            print('Error in Imageshow:', e)
            traceback.print_exc()

    def getTime(self, index):
        try:
            if index >= self.len:
                return None
            msg = self.imList[2 * index].split(',')[0]
            t, _, _ = msg.split()
            return float(t)
        except Exception as e:
            print('Error in Imageshow:', e)
            traceback.print_exc()

    def setBtnStatus(self, state):
        self.graphics.btnBefore['state'] = state
        self.graphics.btnBefore10['state'] = state
        self.graphics.btnNext['state'] = state
        self.graphics.btnNext10['state'] = state
