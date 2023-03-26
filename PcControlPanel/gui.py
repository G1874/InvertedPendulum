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
        self.anim = animation.FuncAnimation(fig=self.fig, func=self.animate, frames=10)
        self.defineControls()
        self.defineTextBox()
        self.root.mainloop()
        
    def defineLabels(self):
        self.root.title("Pendulum Control Panel")
        self.root.iconbitmap("images\stop_player_multimedia_control_icon_232870.ico")
        self.root.attributes("-fullscreen", True)
        self.root.configure(background='#3D5A80')

    def defineControls(self):
        quitButton = Button(master=self.root,text='Quit',font=('sans-serif',13),command=self.quitApp)
        quitButton.place(x=1484,y=820)

        frm = Frame(master=self.root)
        frm.place(x=1200, y=525)
        frm.configure(background='#3D5A80')

        Button(master=frm,text='Start Receiving',font=('sans-serif',13),command=self.startReceiving).grid(row=0,column=0)
        Button(master=frm,text='Manual Control',font=('sans-serif',13),command=self.manualControl).grid(row=1,column=0)
        Button(master=frm,text='Stop Receiving',font=('sans-serif',13),command=self.stopReceiving).grid(row=0,column=1)
        Button(master=frm,text='Automatic Control',font=('sans-serif',13),command=self.automaticMode).grid(row=1,column=1)
        Button(master=frm,text='Stop motor',font=('sans-serif',13),command=self.StopMotor).grid(row=2,column=1)

        for widget in frm.winfo_children():
            widget.grid(padx=20,pady=10,sticky='e')

        Pause = Button(master=self.root,text='Pause',padx=15,font=('sans-serif',8),command=self.toggleAnimationPause)
        Pause.place(x=1104,y=90)

        clearText = Button(master=self.root,text='Clear',padx=15,font=('sans-serif',8),command=self.clearTextBox)
        clearText.place(x=1440,y=450)

        im = ImageTk.PhotoImage(Image.open(r'images\three_dots_icon_159804.ico').
                                resize((25,25),Image.ANTIALIAS))
        misc = Button(master=self.root,image=im,command=self.aditionalOptionsWindow)
        misc.place(x=1220,y=641)
        misc.im = im

    def automaticMode(self):
        self.serialcom.sendInstruction(instructionNrr=77)

    def manualControl(self):
        top = Toplevel()
        top.title('Manual Control')
        top.iconbitmap("images\stop_player_multimedia_control_icon_232870.ico")
        top.configure(background='#3D5A80')
        top.focus_set()
        top.grab_set()

        im1 = ImageTk.PhotoImage(Image.open(r'images\Icons8-Windows-8-Arrows-Left-Circular.ico').
                                resize((25,25),Image.ANTIALIAS))
        im2 = ImageTk.PhotoImage(Image.open(r'images\Icons8-Windows-8-Arrows-Right-Circular.ico').
                                resize((25,25),Image.ANTIALIAS))

        leftButton = Button(master=top,text='Quit',font=('sans-serif',13),image=im1)
        rightButton = Button(master=top,text='Quit',font=('sans-serif',13),image=im2)
        Entry(master=top,width=5,bg='#293241',fg='#E0FBFC').grid(row=0,column=1)

        leftButton.grid(row=0,column=0)
        rightButton.grid(row=0,column=2)

        leftButton.bind("<ButtonPress>", self.leftInstr)
        leftButton.bind("<ButtonRelease>", self.pauseMotor)

        rightButton.bind("<ButtonPress>", self.rightInstr)
        rightButton.bind("<ButtonRelease>", self.pauseMotor)

        leftButton.im1 = im1
        rightButton.im2 = im2

        for widget in top.winfo_children():
            widget.grid(padx=20,pady=10)

        self.serialcom.sendInstruction(instructionNrr=65)

    def rightInstr(self,event):
        self.serialcom.sendInstruction(instructionNrr=82,additionalArgument=15)

    def leftInstr(self,event):
        self.serialcom.sendInstruction(instructionNrr=76,additionalArgument=15)

    def pauseMotor(self,event):
        self.serialcom.sendInstruction(instructionNrr=83)

    def StopMotor(self):
        self.serialcom.sendInstruction(instructionNrr=72)

    def aditionalOptionsWindow(self):
        top = Toplevel()
        top.title('Aditional Options')
        top.iconbitmap("images\stop_player_multimedia_control_icon_232870.ico")
        top.configure(background='#3D5A80')
        top.focus_set()                                                        
        top.grab_set()
        Checkbutton(master=top,text='Save to csv file',variable=self.save_to_file).pack()

    def defineGraph(self):
        frm = Frame(master=self.root)
        frm.place(x=0,y=25)

        plt.style.use("serious_style")

        self.fig = Figure(figsize=(13,8), dpi=100)
        self.a = self.fig.add_subplot(211)
        self.b = self.fig.add_subplot(212)

        self.a_text = self.a.text(x=0.5,y=0.5,s='')
        self.b_text = self.b.text(x=0.5,y=0.5,s='')

        canvas = FigureCanvasTkAgg(figure=self.fig, master=frm)
        canvas.draw()
        NavigationToolbar2Tk(canvas=canvas, window=frm)

        canvas.get_tk_widget().pack()

    def animate(self,i):
        self.a.clear()
        self.b.clear()
        
        self.a.plot(self.serialcom.distance)
        self.b.plot(self.serialcom.angle)

        self.a.set_ylim(ymin=0,ymax=45)

    def toggleAnimationPause(self):
        if self.paused:
            self.anim.resume()
            self.paused = False
        elif not self.paused:
            self.anim.pause()
            self.paused = True

    def defineTextBox(self):
        self.text = Text(master=self.root,height=20,width=32,background='#293241',fg='#E0FBFC')
        self.text.configure(state='disabled')
        self.text.place(x=1250,y=120)
        self.text.tag_config('error',foreground="#EE6C4D")
        self.errorDict = {
            1 : "error: AS5600 - Invalid low power mode specified",
            2 : "error: AS5600 - Invalid hysteresis mode specified",
            3 : "error: AS5600 - Invalid output mode specified",
            4 : "error: AS5600 - Invalid PWM frequency specified",
            5 : "error: AS5600 - Invalid slow filter mode specified",
            6 : "error: AS5600 - Invalid fast filter mode specified",
            7 : "error: AS5600 - Invalid watchdog state specified",
            8 : "error: AS5600 - I2C write register error",
            9 : "error: AS5600 - I2C magnet status read error",
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
            8 : "Left"
        }
        t2 = threading.Thread(target=self.updateTextBox,daemon=True)
        t2.start()

    def writeToTextBox(self,text,tag=None):
        self.text.configure(state='normal')
        self.text.insert(1.0, text + '\n',tag)
        self.text.configure(state='disabled')

    def clearTextBox(self):
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
            if self.serialcom.errorNumber != 0:
                self.writeToTextBox(self.errorDict[self.serialcom.errorNumber],'error')
                self.serialcom.errorNumber = 0
            if self.serialcom.comNumber != 0:
                self.writeToTextBox(self.comDict[self.serialcom.comNumber])
                self.serialcom.comNumber = 0
            time.sleep(0.5)