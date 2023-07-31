from tkinter import *
import translators as ts
from tkinter import messagebox
from PIL import ImageTk, Image
# Importing OpenCV Library for basic image processing functions
import cv2
import mediapipe
# Numpy for array related functions
import numpy as np
# Dlib for deep learning based Modules and face landmark detection
import dlib
# face_utils for basic operations of conversion
from imutils import face_utils
from pygame import mixer


def strt():
    mixer.init()
    mixer.music.load("music.wav")

# Initializing the camera and taking the instance
    cap = cv2.VideoCapture(0)

# Initializing the face detector and landmark detector
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor("models/shape_predictor_68_face_landmarks.dat")

# status marking for current state
    sleep = 0
    drowsy = 0
    active = 0
    status = ""
    color = (0, 0, 0)


    def compute(ptA, ptB):
        dist = np.linalg.norm(ptA - ptB)
        return dist


    def blinked(a, b, c, d, e, f):
        up = compute(b, d) + compute(c, e)
        down = compute(a, f)
        ratio = up / (2.0 * down)

    # Checking if it is blinked
        if (ratio > 0.25):
            return 2
        elif (ratio > 0.21 and ratio <= 0.25):
            return 1
        else:
            return 0


    while True:
        _, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = detector(gray)
    # detected face in faces array
        for face in faces:
            x1 = face.left()
            y1 = face.top()
            x2 = face.right()
            y2 = face.bottom()

            face_frame = frame.copy()
            cv2.rectangle(face_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            landmarks = predictor(gray, face)
            landmarks = face_utils.shape_to_np(landmarks)

        # The numbers are actually the landmarks which will show eye
            left_blink = blinked(landmarks[36], landmarks[37],
                             landmarks[38], landmarks[41], landmarks[40], landmarks[39])
            right_blink = blinked(landmarks[42], landmarks[43],
                              landmarks[44], landmarks[47], landmarks[46], landmarks[45])

        # Now judge what to do for the eye blinks
            if (left_blink == 0 or right_blink == 0):
                sleep += 1
                drowsy = 0
                active = 0
                if (sleep > 6):
                    status = "SLEEPING !!!"
                    mixer.music.play()
                    color = (255, 0, 0)

            elif (left_blink == 1 or right_blink == 1):
                sleep = 0
                active = 0
                drowsy += 1
                if (drowsy > 6):
                    status = "Drowsy !"
                    mixer.music.play()
                    color = (0, 0, 255)

            else:
                drowsy = 0
                sleep = 0
                active += 1
                if (active > 6):
                    status = "Active :)"
                    mixer.music.stop()
                    color = (0, 255, 0)

            cv2.putText(frame, status, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

            for n in range(0, 68):
                (x, y) = landmarks[n]
                cv2.circle(face_frame, (x, y), 1, (255, 255, 255), -1)

        cv2.imshow("Frame", frame)
        cv2.imshow("Result of detector", face_frame)
        key = cv2.waitKey(1)
        if key == 27:
           break


    




    
root = Tk()
root.geometry('600x500')
root.title('Translater')
root.configure(bg='gold2')
C = Canvas(root, bg="black",height=300,width=350)
C.place(x=50,y=120)
global our_images, count
count=0

our_images = [
               ImageTk.PhotoImage((Image.open("1.jpeg")).resize((350,300))),
               ImageTk.PhotoImage((Image.open("2.jpeg")).resize((350,300))),
               ImageTk.PhotoImage((Image.open("3.jpeg")).resize((350,300))),
               ImageTk.PhotoImage((Image.open("4.jpeg")).resize((350, 300))),
               ImageTk.PhotoImage((Image.open("5.jpeg")).resize((350, 300)))
              
    ]

C.create_image(0, 0, anchor=NW, image=our_images[0])
def next():
    global count
    if count == 4 :
        C.create_image(0, 0, anchor=NW, image=our_images[0])
        count = 0

    else :
        C.create_image(0, 0, anchor=NW, image=our_images[count+1])
        count+=1

    root.after(1000, next)
next()
global title
title=Label(root,text=''' Driver Drowsiness  \nDetction ''',font=('stencil', 24, 'italic bold  '),fg='white',bg="gold2")
title.place(x=70,y=25)

def on_enterlogin_start(e):
    btn_start['fg'] = 'white'
    btn_start['bg'] = 'red'


def on_leavelogin_start(e):
    btn_start['fg'] = 'gold2'
    btn_start['bg'] = 'white'

btn_start = Button(root, text="START", fg="gold2",bg="white", activebackground='green',width=7, height=2,font=('stencil', 10, 'italic  ') , bd=5, command=strt)
btn_start.place(x=500, y=50)
btn_start.bind('<Enter>', on_enterlogin_start)
btn_start.bind('<Leave>', on_leavelogin_start)


def main_exit():
    rr = messagebox.askyesnocancel('EXIT', 'Are you want to exit !', parent=root)
    if (rr == True):
     root.destroy()



def on_enterexit_btn(e):
    exit_btn['bg'] = 'red'
    exit_btn['fg'] = 'white'


def on_leaveexit_btn(e):
    exit_btn['bg'] = 'white'
    exit_btn['fg'] = 'gold2'



exit_btn = Button(root, text='Exit', bd=5, fg="gold2",bg="white", activebackground='green', width=7, height=2,font=('stencil',10 , 'italic  '), compound=BOTTOM,command=main_exit)
exit_btn.place(x=500, y=360)
exit_btn.bind('<Enter>', on_enterexit_btn)
exit_btn.bind('<Leave>', on_leaveexit_btn)


lab_cong=Label(root,text="",font=('arial', 10, 'italic bold  '),fg='red',bg="white")
lab_cong.place(x=30,y=450)

root.mainloop()
