# Copyright 2020 Sanggyu Lee. All rights reserved.
# sanggyu.developer@gmail.com
if 'jajucha' in __name__:
    from jajucha import *
else:
    import config
    import control
    import planning
import tkinter
import tkinter.filedialog
import cv2
from PIL import Image, ImageTk


class Graphics:
    def __init__(self, classPlanning):
        self.root = tkinter.Tk()
        self.root.title("자주차 컨트롤러 v.2.1")
        self.root.geometry("1000x620-100+100")
        self.root.resizable(False, False)
        self.view_frame = tkinter.Frame(self.root, width=700, height=620)
        self.control_frame = tkinter.Frame(self.root, width=300, height=620)
        self.view_frame.grid(row=1, column=1)
        self.control_frame.grid(row=1, column=2)
        self.view_frame.grid_propagate(False)
        self.control_frame.grid_propagate(False)

        self.front_frame = tkinter.Frame(self.view_frame, width=700, height=280)
        self.back_frame = tkinter.Frame(self.view_frame, width=700, height=280)
        self.text_frame = tkinter.Frame(self.view_frame, width=700, height=60)
        self.front_frame.grid(row=1, column=1)
        self.back_frame.grid(row=2, column=1)
        self.text_frame.grid(row=3, column=1)
        self.front_frame.grid_propagate(False)
        self.back_frame.grid_propagate(False)
        self.text_frame.grid_propagate(False)

        self.imgFront1 = tkinter.Label(self.front_frame, width=320, height=240, bg='white', bitmap='question')
        self.imgFront2 = tkinter.Label(self.front_frame, width=320, height=240, bg='white', bitmap='question')
        self.imgFront1.grid(row=1, column=1, padx=(15, 10), pady=(5, 5))
        self.imgFront2.grid(row=1, column=2, padx=(10, 15), pady=(5, 5))
        self.imgFront1.grid_propagate(False)
        self.imgFront2.grid_propagate(False)

        self.varTxtFront = tkinter.StringVar()
        self.varTxtFront.set("전면 LiDAR: ????mm")
        self.txtFront1 = tkinter.Label(self.front_frame, text="전면 카메라")
        self.txtFront2 = tkinter.Label(self.front_frame, textvariable=self.varTxtFront)
        self.txtFront1.grid(row=2, column=1)
        self.txtFront2.grid(row=2, column=2)
        self.txtFront1.grid_propagate(False)
        self.txtFront2.grid_propagate(False)

        self.imgBack1 = tkinter.Label(self.back_frame, width=320, height=240, bg='white', bitmap='question')
        self.imgBack2 = tkinter.Label(self.back_frame, width=320, height=240, bg='white', bitmap='question')
        self.imgBack1.grid(row=1, column=1, padx=(15, 10), pady=(5, 5))
        self.imgBack2.grid(row=1, column=2, padx=(10, 15), pady=(5, 5))
        self.imgBack1.grid_propagate(False)
        self.imgBack2.grid_propagate(False)

        self.varTxtBack = tkinter.StringVar()
        self.varTxtBack.set("후면 LiDAR: ????mm")
        self.txtBack1 = tkinter.Label(self.back_frame, text="후면 카메라")
        self.txtBack2 = tkinter.Label(self.back_frame, textvariable=self.varTxtBack)
        self.txtBack1.grid(row=2, column=1)
        self.txtBack2.grid(row=2, column=2)
        self.txtBack1.grid_propagate(False)
        self.txtBack2.grid_propagate(False)

        self.varTxt = tkinter.StringVar()
        self.varTxt.set("연결 시도중...")
        self.txt = tkinter.Label(self.text_frame, textvariable=self.varTxt)
        self.txt.pack(pady=(3, 3))

        self.varTxtCommand = tkinter.StringVar()
        self.varTxtCommand.set("조향: ???, 속도: ???")
        self.txtCommand = tkinter.Label(self.text_frame, textvariable=self.varTxtCommand)
        self.txtCommand.pack(pady=0)

        self.varTxtTime = tkinter.StringVar()
        self.setTime(0)
        self.txtTime = tkinter.Label(self.control_frame, textvariable=self.varTxtTime)
        self.txtTime.grid(row=7, column=1, sticky='w', pady=(120, 0))
        self.txtTime.grid_propagate(False)

        self.runMode = tkinter.IntVar()
        self.runMode.set(0)
        self.runMode.trace("w", lambda *args: self.callback_change_mode())
        self.btnModeDrive = tkinter.Radiobutton(self.control_frame, text="주행 모드", \
                                                value=0, variable=self.runMode)
        self.btnModeImage = tkinter.Radiobutton(self.control_frame, text="재생 모드", \
                                                value=1, variable=self.runMode)
        self.btnModeDrive.grid(row=1, column=1, sticky='w')
        self.btnModeImage.grid(row=4, column=1, sticky='w', pady=(130, 0))

        self.runSave = tkinter.IntVar()
        self.runSave.set(0)
        self.runSave.trace("w", lambda *args: self.callback_change_save())
        self.btnDriveSave = tkinter.Checkbutton(self.control_frame, text="주행 저장", \
                                                onvalue=1, offvalue=0, variable=self.runSave)
        self.btnDriveSave.grid(row=2, column=1, sticky='w')
        self.varTxtSavePath = tkinter.StringVar()
        self.varTxtSavePath.set("저장할 폴더를 선택하세요.")
        self.lblDriveSave = tkinter.Label(self.control_frame, textvariable=self.varTxtSavePath)
        self.btnFindSavePath = tkinter.Button(self.control_frame, text="폴더 찾기", command=self.callback_save_dir)
        self.lblDriveSave.grid(row=3, column=1, sticky='w')
        self.btnFindSavePath.grid(row=3, column=2, sticky='w')
        self.saveLocation = None

        self.varTxtLoadPath = tkinter.StringVar()
        self.varTxtLoadPath.set("불러올 폴더를 선택하세요.")
        self.lblImageLoad = tkinter.Label(self.control_frame, textvariable=self.varTxtLoadPath)
        self.btnFindLoadPath = tkinter.Button(self.control_frame, text="폴더 찾기", command=self.callback_load_dir)
        self.lblImageLoad.grid(row=5, column=1, sticky='w')
        self.btnFindLoadPath.grid(row=5, column=2, sticky='w')
        self.loadLocation = None

        self.ImageControlFrame = tkinter.Frame(self.control_frame)
        self.btnBefore = tkinter.Button(self.ImageControlFrame, text="-1", command=self.callback_before)
        self.btnBefore10 = tkinter.Button(self.ImageControlFrame, text="-10", command=self.callback_before10)
        self.btnNext = tkinter.Button(self.ImageControlFrame, text="+1", command=self.callback_next)
        self.btnNext10 = tkinter.Button(self.ImageControlFrame, text="+10", command=self.callback_next10)
        self.ImageControlFrame.grid(row=6, column=1, sticky='w')
        self.btnBefore10.grid(row=1, column=1, padx=(0, 1))
        self.btnBefore.grid(row=1, column=2, padx=(1, 1))
        self.btnNext.grid(row=1, column=3, padx=(1, 1))
        self.btnNext10.grid(row=1, column=4, padx=(1, 0))

        self.varTxtStartStop = tkinter.StringVar()
        self.varTxtStartStop.set("준비")
        self.btnStartStop = tkinter.Button(self.control_frame, textvariable=self.varTxtStartStop, \
                                           command=self.callback_start_stop)
        self.btnStartStop.grid(row=8, column=1, sticky='nwse')

        self.plan = classPlanning(self)
        self.control = control.BaseControl(self)
        self.control.quit()
        self.root.protocol("WM_DELETE_WINDOW", self.callback_close)
        self.callback_change_mode()

    def callback_close(self):
        self.control.quit()
        # self.control.hang()
        self.root.after(300, self.root.destroy)

    def setFrontLidar(self, value):  # value: Positive Integer < 10000
        self.varTxtFront.set("전면 LiDAR: %04dmm" % value)

    def setRearLidar(self, value):  # value: Positive Integer < 10000
        self.varTxtBack.set("후면 LiDAR: %04dmm" % value)

    def setCommandText(self, value):  # value: String
        self.varTxt.set(value)

    def setCommand(self, steer, velocity):  # angle, velocity: float
        self.varTxtCommand.set("조향: %+04d, 속도: %+06.1fmm/s" % (steer, velocity))

    def setTime(self, value):  # value: Float
        self.varTxtTime.set("시간: %06.2fs" % value)
        # self.varTxtTime.set("Latency: %dms"%(int(value*1000)))

    def setFrontImage1(self, array, **kwargs):
        tkImage = self.getTkImage(array, **kwargs)
        self.imgFront1.configure(image=tkImage)
        self.imgFront1.image = tkImage

    def setFrontImage2(self, array, **kwargs):
        tkImage = self.getTkImage(array, **kwargs)
        self.imgFront2.configure(image=tkImage)
        self.imgFront2.image = tkImage

    def setRearImage1(self, array, **kwargs):
        tkImage = self.getTkImage(array, **kwargs)
        self.imgBack1.configure(image=tkImage)
        self.imgBack1.image = tkImage

    def setRearImage2(self, array, **kwargs):
        tkImage = self.getTkImage(array, **kwargs)
        self.imgBack2.configure(image=tkImage)
        self.imgBack2.image = tkImage

    @staticmethod
    def getTkImage(arrayImg, isBGR=True):
        arrayImg = cv2.resize(arrayImg, (320, 240))
        if isBGR:
            arrayImg = cv2.cvtColor(arrayImg, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(arrayImg)
        tkImg = ImageTk.PhotoImage(image=img)
        return tkImg

    def callback_start_stop(self):
        status = self.varTxtStartStop.get()
        if status == '준비':
            if self.control.start() is None:
                self.varTxtStartStop.set('중지')
        elif status == '중지':
            self.control.stop()
            self.varTxtStartStop.set('준비')
        elif status == '재생':
            self.control.play()
            self.varTxtStartStop.set('일시정지')
        elif status == '일시정지':
            self.control.pause()
            self.varTxtStartStop.set('재생')

    def callback_save_dir(self):
        location = tkinter.filedialog.askdirectory(title="저장할 경로 선택")
        if not location:
            return
        self.saveLocation = location
        location = location.split('/')
        self.varTxtSavePath.set('.../' + '/'.join(location[-2:]))
        # Setting accessed at time of 'start'

    def callback_load_dir(self):
        location = tkinter.filedialog.askdirectory(title="불러올 경로 선택")
        if not location:
            return
        self.loadLocation = location
        location = location.split('/')
        self.varTxtLoadPath.set('.../' + '/'.join(location[-2:]))
        self.control.quit()
        self.control = control.ImageControl(self, self.loadLocation)

    def callback_change_mode(self):
        mode = self.runMode.get()
        self.btnBefore['state'] = tkinter.DISABLED
        self.btnBefore10['state'] = tkinter.DISABLED
        self.btnNext['state'] = tkinter.DISABLED
        self.btnNext10['state'] = tkinter.DISABLED
        if mode == 0:  # Drive Mode
            self.btnDriveSave['state'] = tkinter.NORMAL
            self.callback_change_save()
            self.lblImageLoad['state'] = tkinter.DISABLED
            self.btnFindLoadPath['state'] = tkinter.DISABLED

            self.varTxtStartStop.set("준비")
            self.btnStartStop['state'] = tkinter.DISABLED
            self.setCommandText('연결 시도중...')

            self.control.quit()
            self.control = control.DriveControl(self, config.address)
        elif mode == 1:  # Image Mode
            self.btnDriveSave['state'] = tkinter.DISABLED
            self.btnFindSavePath['state'] = tkinter.DISABLED
            self.lblDriveSave['state'] = tkinter.DISABLED
            self.lblImageLoad['state'] = tkinter.NORMAL
            self.btnFindLoadPath['state'] = tkinter.NORMAL

            self.varTxtStartStop.set("재생")
            self.btnStartStop['state'] = tkinter.DISABLED

            self.loadLocation = None
            self.varTxtLoadPath.set('불러올 폴더를 선택하세요.')
            self.setCommandText('불러올 폴더를 선택하세요.')
            self.control.quit()

    def callback_change_save(self):
        isSave = self.runSave.get()
        # Setting accessed at time of 'start'
        if isSave == 0:
            self.btnFindSavePath['state'] = tkinter.DISABLED
            self.lblDriveSave['state'] = tkinter.DISABLED
        else:
            self.btnFindSavePath['state'] = tkinter.NORMAL
            self.lblDriveSave['state'] = tkinter.NORMAL

    def callback_before(self):
        self.control.before()

    def callback_before10(self):
        self.control.before10()

    def callback_next(self):
        self.control.next()

    def callback_next10(self):
        self.control.next10()

    def exit(self):
        self.control.quit()


# foo = tkinter.filedialog.askopenfilename(title="실행할 코드 선택", filetypes=(("Python files", "*.py"), ("All files", "*.*")))

# Handle Connection (DISCONNECT)
if __name__ == "__main__":
    g = Graphics(planning.BasePlanning)
    g.root.mainloop()
    g.exit()
