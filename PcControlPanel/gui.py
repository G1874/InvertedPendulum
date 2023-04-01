from tkinter import *
import threading
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
from matplotlib.figure import Figure
from SerialCOM import SerialCommunication
from PIL import Image, ImageTk
import time


class Gui():
    def __init__(self) -> None:
        self.root = Tk()
        self.save_to_file:bool = False
        self.paused:bool = False

    def quitApp(self):
        self.serialcom.stopReceiving()
        plt.close()
        self.root.destroy()

    def guiInit(self):
        self.createSerialInstance()
        self.defineLabels()
        self.defineGraph()
        self.defineAngleReading()
        self.defineDistanceReading()
        self.anim = animation.FuncAnimation(fig=self.fig,func=self.animate,frames=10)
        self.defineControls()
        self.defineTextBox()
        self.root.mainloop()
        
    def defineLabels(self):
        self.root.title("Pendulum Control Panel")
        self.root.iconbitmap(".\images\stop_player_multimedia_control_icon_232870.ico")
        self.root.attributes("-fullscreen", True)
        self.root.configure(background='#3D5A80')

    def defineControls(self):
        quitButton = Button(master=self.root,text='Quit',font=('sans-serif',13),command=self.quitApp)
        quitButton.place(x=1481,y=820)

        frm = Frame(master=self.root)
        frm.place(x=1196, y=525)
        frm.configure(background='#3D5A80')

        Button(master=frm,text='Start Receiving',font=('sans-serif',13),command=self.startReceiving).grid(row=0,column=0)
        Button(master=frm,text='Stop Receiving',font=('sans-serif',13),command=self.stopReceiving).grid(row=0,column=1)
        Button(master=frm,text='Automatic Control',font=('sans-serif',13),command=self.automaticMode).grid(row=1,column=1)
        Button(master=frm,text='Stop motor',font=('sans-serif',13),command=self.stopMotor).grid(row=2,column=1)
        self.manControlButton = Button(master=frm,text='Manual Control',font=('sans-serif',13),command=self.manualControl)
        self.manControlButton.grid(row=1,column=0)

        for widget in frm.winfo_children():
            widget.grid(padx=20,pady=10,sticky='e')

        Pause = Button(master=self.root,text='Pause',padx=15,font=('sans-serif',8),command=self.toggleAnimationPause)
        Pause.place(x=1104,y=90)

        clearText = Button(master=self.root,text='Clear',padx=15,font=('sans-serif',8),command=self.clearTextBoxes)
        clearText.place(x=1440,y=450)

        im = ImageTk.PhotoImage(Image.open(r'.\images\three_dots_icon_159804.ico').
                                resize((25,25),Image.ANTIALIAS))
        self.misc = Button(master=self.root,image=im,command=self.aditionalOptionsWindow)
        self.misc.place(x=1220,y=641)
        self.misc.im = im

    def automaticMode(self):
        self.serialcom.sendInstruction(instructionNrr=77)

    def manualControl(self):
        self.manControlButton.configure(state='disabled')

        self.top2 = Toplevel()
        self.top2.title('Manual Control')
        self.top2.iconbitmap(".\images\stop_player_multimedia_control_icon_232870.ico")
        self.top2.configure(background='#3D5A80')
        self.top2.wm_attributes("-topmost", 1)
        self.top2.protocol("WM_DELETE_WINDOW",self.closeManControlWindow)
        #self.top2.grid_rowconfigure(1,weight=1)

        im1 = ImageTk.PhotoImage(Image.open(r'.\images\Icons8-Windows-8-Arrows-Left-Circular.ico').
                                resize((30,30),Image.ANTIALIAS))
        im2 = ImageTk.PhotoImage(Image.open(r'.\images\Icons8-Windows-8-Arrows-Right-Circular.ico').
                                resize((30,30),Image.ANTIALIAS))

        leftButton = Button(master=self.top2,text='Quit',font=('sans-serif',13),image=im1)
        rightButton = Button(master=self.top2,text='Quit',font=('sans-serif',13),image=im2)

        leftButton.im1 = im1
        rightButton.im2 = im2

        leftButton.grid(row=0,column=0)
        rightButton.grid(row=0,column=1)

        leftButton.bind("<ButtonPress>", self.leftInstr)
        leftButton.bind("<ButtonRelease>", self.pauseMotor)

        rightButton.bind("<ButtonPress>", self.rightInstr)
        rightButton.bind("<ButtonRelease>", self.pauseMotor)

        for widget in self.top2.winfo_children():
            widget.grid(padx=20,pady=10)

        self.serialcom.sendInstruction(instructionNrr=65)

    def closeManControlWindow(self):
        self.manControlButton.configure(state='normal')
        self.top2.destroy()

    def rightInstr(self,event):
        self.serialcom.sendInstruction(instructionNrr=82,additionalArgument=60)

    def leftInstr(self,event):
        self.serialcom.sendInstruction(instructionNrr=76,additionalArgument=60)

    def pauseMotor(self,event):
        self.serialcom.sendInstruction(instructionNrr=83)

    def stopMotor(self):
        self.serialcom.sendInstruction(instructionNrr=72)

    def zeroAS5600Position(self):
        self.serialcom.sendInstruction(instructionNrr=90)

    def getAGCregister(self):
        self.serialcom.sendInstruction(instructionNrr=71)

    def getMagnetStatus(self):
        self.serialcom.sendInstruction(instructionNrr=84)

    def aditionalOptionsWindow(self):
        self.misc.configure(state="disabled")

        self.top1 = Toplevel()
        self.top1.title('Aditional Options')
        self.top1.iconbitmap("images\stop_player_multimedia_control_icon_232870.ico")
        self.top1.configure(background='#3D5A80')
        self.top1.wm_attributes("-topmost", 1)
        self.top1.protocol("WM_DELETE_WINDOW",self.closeAdditionalWindow)

        self.defineAdditionalGraph(window=self.top1)
        self.anim2 = animation.FuncAnimation(fig=self.fig2,func=self.animateSpeed,frames=10)
        self.defineExtraReading()

        for widget in self.top1.winfo_children():
            widget.grid(padx=10)

        frm = Frame(master=self.top1)
        frm.grid(row=1,column=0)
        frm.configure(background='#3D5A80')

        Button(master=frm,text='Set AS5600 zero position',font=('sans-serif',13),command=self.zeroAS5600Position).grid(row=1,column=0)
        Button(master=frm,text='Get AS5600 agc register',font=('sans-serif',13),command=self.getAGCregister).grid(row=1,column=1)
        Button(master=frm,text='Get AS5600 magnet status',font=('sans-serif',13),command=self.getMagnetStatus).grid(row=1,column=2)
        Checkbutton(master=frm,text='Save to csv file',font=('sans-serif',13),variable=self.save_to_file).grid(row=1,column=3)

        for widget in frm.winfo_children():
            widget.grid(padx=20,pady=10,sticky='w')

    def closeAdditionalWindow(self):
        self.misc.configure(state="normal")
        self.top1.destroy()

    def defineGraph(self):
        frm = Frame(master=self.root)
        frm.place(x=0,y=25)

        plt.style.use("serious_style")

        self.fig = Figure(figsize=(13,8), dpi=100)
        self.a = self.fig.add_subplot(211)
        self.b = self.fig.add_subplot(212)

        self.a_text = self.a.text(x=0.5,y=0.5,s='')
        self.b_text = self.b.text(x=0.5,y=0.5,s='')

        canvas = FigureCanvasTkAgg(figure=self.fig,master=frm)
        canvas.draw()
        NavigationToolbar2Tk(canvas=canvas,window=frm)

        canvas.get_tk_widget().pack()

    def defineAdditionalGraph(self,window):
        frm = Frame(master=window)
        frm.grid(row=0,column=0)

        plt.style.use("serious_style")

        self.fig2 = Figure(figsize=(9,3), dpi=100)
        self.c = self.fig2.add_subplot(111)

        canvas = FigureCanvasTkAgg(figure=self.fig2,master=frm)
        canvas.draw()
        NavigationToolbar2Tk(canvas=canvas,window=frm)

        canvas.get_tk_widget().pack()

    def defineDistanceReading(self):
        self.textDist = Text(master=self.root,height=17,width=10,background='#293241',fg='#E0FBFC')
        self.textDist.configure(state='disabled')
        self.textDist.place(x=15,y=120)

    def defineAngleReading(self):
        self.textAng = Text(master=self.root,height=17,width=10,background='#293241',fg='#E0FBFC')
        self.textAng.configure(state='disabled')
        self.textAng.place(x=15,y=457)

    def defineExtraReading(self):
        self.textExtr = Text(master=self.top1,height=17,width=11,background='#293241',fg='#E0FBFC')
        self.textExtr.configure(state='disabled')
        self.textExtr.grid(row=0,column=1)

    def writeReadingData(self):
        self.textDist.configure(state='normal')
        self.textAng.configure(state='normal')
        self.textDist.insert(1.0, str(self.serialcom.dist) + '\n')
        self.textAng.insert(1.0, str(self.serialcom.ang) + '\n')
        self.textDist.configure(state='disabled')
        self.textAng.configure(state='disabled')

    
    def writeExtraReadingData(self):
        self.textExtr.configure(state='normal')
        self.textExtr.insert(1.0, str(self.serialcom.spd) + ' - ' + str(self.serialcom.extra_data) + '\n')
        self.textExtr.configure(state='disabled')

    def animate(self,i):
        self.a.clear()
        self.b.clear()
        
        self.a.plot(self.serialcom.distance)
        self.b.plot(self.serialcom.angle)

        self.a.set_ylim(ymin=-0.21,ymax=0.21)
        self.b.set_ylim(ymin=-180,ymax=180)

        self.writeReadingData()

    def animateSpeed(self,i):
        self.c.clear()
        self.c.plot(self.serialcom.speed)

        self.writeExtraReadingData()
        #self.c.set_ylim(ymin=0,ymax=100000)

    def toggleAnimationPause(self):
        if self.paused:
            self.anim.resume()
            self.paused = False
        elif not self.paused:
            self.anim.pause()
            self.paused = True

    def defineTextBox(self):
        self.text = Text(master=self.root,height=20,width=35,background='#293241',fg='#E0FBFC')
        self.text.configure(state='disabled')
        self.text.place(x=1238,y=120)
        self.text.tag_config('error',foreground="#EE6C4D")
        self.errorDict = {
            1 : "error: AS5600 - Invalid low power mode specified",
            2 : "error: AS5600 - Invalid hysteresis mode specified",
            3 : "error: AS5600 - Invalid output mode specified",
            4 : "error: AS5600 - Invalid PWM frequency specified",
            5 : "error: AS5600 - Invalid slow filter mode specified",
            6 : "error: AS5600 - Invalid fast filter mode specified",
            7 : "error: AS5600 - Invalid watchdog state specified",
            8 : "error: AS5600 - I2C Write register error",
            9 : "error: AS5600 - I2C Magnet status read error",
            10 : "error: AS5600 - Magnet not detected",
            11 : "error: AS5600 - B-field is too strong",
            12 : "error: AS5600 - B-field is too weak",
            13 : "error: Rising edge timer input capture start error",
            14 : "error: Falling edge timer input capture start error",
            15 : "error: Sensor triger start error",
            16 : "error: Synchronization timer start error",
            17 : "error: AS5600 - I2C Read register error",
            18 : "error: Port not available",
            19 : "error: Failed to keep up"
        }
        self.comDict = {
            1 : "Command acknowledged",
            2 : "Data transmision stopped",
            3 : "Data transmision started",
            4 : "Manual control mode",
            5 : "Automatic control mode",
            6 : "Stopped",
            7 : "Right",
            8 : "Left",
            9 : "AS5600 Magnet status: ",
            10 : "AS5600 AGC register: ",
            11 : "AS5600 position zeroed"
        }
        t2 = threading.Thread(target=self.updateTextBox,daemon=True)
        t2.start()

    def writeToTextBox(self,text,tag=None):
        self.text.configure(state='normal')
        self.text.insert(1.0, text + '\n',tag)
        self.text.configure(state='disabled')

    def clearTextBoxes(self):
        self.text.configure(state='normal')
        self.text.delete(1.0,END)
        self.text.configure(state='disabled')

    def createSerialInstance(self):
        self.serialcom = SerialCommunication()

    def startReceiving(self):
        if self.paused:
            self.anim.resume()
            self.paused = False
        if self.serialcom.getPortState():
            t1 = threading.Thread(target=self.serialcom.startReceiving,args=(self.save_to_file,),daemon=True)
            t1.start()
        else:
            self.writeToTextBox(self.errorDict[18],'error')

    def stopReceiving(self):
        self.serialcom.stopReceiving()
        if not self.paused:
            self.anim.pause()
            self.paused = True

    def updateTextBox(self):
        while True:
            if self.serialcom.error_number != 0:
                self.writeToTextBox(self.errorDict[self.serialcom.error_number],'error')
                self.serialcom.error_number = 0
            if self.serialcom.com_number != 0:
                if self.serialcom.com_number == 10:
                    self.writeToTextBox(self.comDict[self.serialcom.com_number] + str(self.serialcom.register_data))
                elif self.serialcom.com_number == 9:
                    match self.serialcom.register_data:
                        case 16:
                            self.writeToTextBox(self.comDict[self.serialcom.com_number] + "Magnet not detected")
                        case 24:
                            self.writeToTextBox(self.comDict[self.serialcom.com_number] + "Magnet too weak")
                        case 40:
                            self.writeToTextBox(self.comDict[self.serialcom.com_number] + "Magnet too strong")
                        case 32:
                            self.writeToTextBox(self.comDict[self.serialcom.com_number] + "Magnet OK")
                        case _:
                            self.writeToTextBox(self.comDict[self.serialcom.com_number] + "Invalid register value")
                else:
                    self.writeToTextBox(self.comDict[self.serialcom.com_number])
                self.serialcom.com_number = 0
            time.sleep(0.5)
