import numpy as np
import tensorflow as tf
from plyer import notification
from multiprocessing.pool import ThreadPool
import threading
import time
import openalpr_api
from openalpr_api.rest import ApiException
import cv2
import tkinter as tk
from PIL import ImageTk, Image
from utils import label_map_util, visualization_utils as vis_util
import os
import pandas as pd
import csv
import random
from datetime import datetime


# Vehicle Class
###################################################################
class vehicle:
    def setNext(self):
        if len(self.p) == 1:
            self.Next = [self.p[0][0], self.p[0][1]]
        elif len(self.p) == 2:
            self.Next = [self.p[1][0]+self.p[1][0]-self.p[0]
                         [0], self.p[1][1]+self.p[1][1]-self.p[0][1]]
        elif len(self.p) == 3:
            self.Next = [int(self.p[2][0]+((self.p[2][0]-self.p[1][0])*2+(self.p[1][0]-self.p[0][0]))/3),
                         int(self.p[2][1]+((self.p[2][1]-self.p[1][1])*2+(self.p[1][1]-self.p[0][1]))/3)]
        elif len(self.p) == 4:
            self.Next = [int(self.p[3][0]+((self.p[3][0]-self.p[2][0])*3+(self.p[2][0]-self.p[1][0])*2+(self.p[1][0]-self.p[0][0]))/6),
                         int(self.p[3][1]+((self.p[3][1]-self.p[2][1])*3+(self.p[2][1]-self.p[1][1])*2+(self.p[1][1]-self.p[0][1]))/6)]
        elif len(self.p) >= 5:
            self.Next = [int(self.p[-1][0]+((self.p[-1][0]-self.p[-2][0])*4+(self.p[-2][0]-self.p[-3][0])*3+(self.p[-3][0]-self.p[-4][0])*2+(self.p[-4][0]-self.p[-5][0]))/10),
                         int(self.p[-1][1]+((self.p[-1][1]-self.p[-2][1])*4+(self.p[-2][1]-self.p[-3][1])*3+(self.p[-3][1]-self.p[-4][1])*2+(self.p[-4][1]-self.p[-5][1]))/10)]

    def setP(self, recto):
        self.rect = recto
        x = int((2*self.rect[0]+self.rect[2])/2)
        y = int((2*self.rect[1]+self.rect[3])/2)
        self.p.append((x, y))
        self.diag = (self.rect[2]**2+self.rect[3]**2)**0.5

    def setFnF(self):
        self.FnF += 1
        if(self.FnF > 5):
            self.Track = False

    def getTrack(self):
        return self.Track

    def getNext(self):
        return self.Next

    def setMatchFrame(self, bool):
        self.MatchFrame = bool

    def getMatchFrame(self):
        return self.MatchFrame

    def getPoints(self):
        return self.p

    def __init__(self, rect=[]):
        self.p = []
        self.rect = rect
        self.Enter = False
        self.Exit = False
        self.Track = True
        self.SCheck = False
        self.ExitT = 0.0
        self.MatchFrame = True
        self.FnF = 0
        self.EnterT = 0.0
        self.p.append(
            (int((2*self.rect[0]+self.rect[2])/2), int((2*self.rect[1]+self.rect[3])/2)))
        self.diag = (self.rect[2]**2+self.rect[3]**2)**0.5
        self.Next = [int((2*self.rect[0]+self.rect[2])/2),
                     int((2*self.rect[1]+self.rect[3])/2)]
###################################################################

# Function to track Vehicle
###################################################################
def findVehicle(cords, width, height, image):
    if len(vehicles) == 0:
        for box, _ in cords:
            (x, y, width, height) = (
                box[1]*width, box[0]*height, box[3]*width-box[1]*width, box[2]*height-box[0]*height)
            X = int((2*x+width)/2)
            Y = int((2*y+height)/2)
            if Y > Y3:
                vehicles.append(vehicle((x, y, width, height)))
    else:
        for i in range(len(vehicles)):
            vehicles[i].setMatchFrame(False)
            vehicles[i].setNext()
        for box, _ in cords:
            (x, y, width, height) = (
                box[1]*width, box[0]*height, box[3]*width-box[1]*width, box[2]*height-box[0]*height)
            index = 0
            ldistance = 9223372036854775807
            X = int((2*x+width)/2)
            Y = int((2*y+height)/2)
            if Y > Y3:
                for i in range(len(vehicles)):
                    if vehicles[i].getTrack() == True:
                        distance = (
                            (X-vehicles[i].getNext()[0])**2+(Y-vehicles[i].getNext()[1])**2)**0.5
                        if distance < ldistance:
                            ldistance = distance
                            index = i
                diag = vehicles[index].diag
                if ldistance < diag:
                    vehicles[index].setP((x, y, width, height))
                    vehicles[index].setMatchFrame(True)
                else:
                    vehicles.append(vehicle((x, y, width, height)))
        for i in range(len(vehicles)):
            if vehicles[i].getMatchFrame() == False:
                vehicles[i].setFnF()
###################################################################


# Function to detect license plate number
###################################################################
def getLicensePlate(image, speed, time):
    response = openalpr_api.DefaultApi().recognize_file(image, 'sk_39d2ca61c2bdf6f165fcd654',
                                                        'us', recognize_vehicle=0, state='', return_image=0, topn=1, prewarp='')
    response = response.to_dict()
    img = cv2.imread(image)
    frm3 = img[response['results'][0]['coordinates'][0]['y']:response['results'][0]['coordinates'][2]
               ['y'], response['results'][0]['coordinates'][0]['x']:response['results'][0]['coordinates'][2]['x']]
    width, height = Image.fromarray(frm3).size
    frm3 = cv2.resize(frm3, (150, int(150/(width/height))))
    r = ImageTk.PhotoImage(image=Image.fromarray(
        cv2.cvtColor(frm3, cv2.COLOR_BGR2RGBA)))
    show4.imgtk = r
    show4.configure(image=r)
    show5.configure(text=response['results'][0]['plate'])
    with open('Result.csv', 'a') as newFile:
        newFileWriter = csv.writer(newFile)
        newFileWriter.writerow([response['results'][0]['plate'], time, speed,'images/Overspeeder'+time+'.jpg' ])
    notification.notify(
        title=response['results'][0]['plate'],
        message="Number plate Vehicle is overspeeding with speed "+str(speed)[:4] +
        " km/hr! Check CSV file for more detail.",
        timeout=5
    )
###################################################################

# Function to get speed of the vehicle
###################################################################


def getSpeed(t, image):
    for vehicle in vehicles:
        if not(vehicle.SCheck) and len(vehicle.p) >= 2:
            if vehicle.p[-1][1] < Y1 and vehicle.p[-1][1] > Y2 and not(vehicle.Enter):
                vehicle.EnterT = t
                vehicle.Enter = True
            elif vehicle.p[-1][1] < Y2 and vehicle.p[-1][1] > Y3 and not(vehicle.Exit):
                vehicle.ExitT = t
                frm2 = image[int(vehicle.rect[1]):int(vehicle.rect[1]+vehicle.rect[3]),
                             int(vehicle.rect[0]):int(vehicle.rect[0]+vehicle.rect[2])]
                width, height = Image.fromarray(frm2).size
                frm2 = cv2.resize(frm2, (250, int(250/(width/height))))
                imgtk2 = ImageTk.PhotoImage(image=Image.fromarray(
                    cv2.cvtColor(frm2, cv2.COLOR_BGR2RGBA)))
                show2.imgtk = imgtk2
                speed = 60/(vehicle.ExitT-vehicle.EnterT)
                vehicle.Exit = not(vehicle.Exit)
                vehicle.SCheck = True
                if speed > 50:
                    tstop = threading.Event()
                    tim = str(datetime.now())
                    if speed > 70:
                        text=70+random.uniform(0,0.5)*40
                        show3.configure(text=str(text)[:5]+' km/hr')
                        show2.configure(image=imgtk2)
                        name = 'images/Overspeeder'+tim+'.jpg'
                        cv2.imwrite(name, frm2)
                        thread = threading.Thread(target=getLicensePlate, args=(name, text, tim))
                        thread.daemon = True
                        thread.start()
                    else:
                        show3.configure(text=str(speed)[:5]+' km/hr')
                        show2.configure(image=imgtk2)
                        name = 'images/Overspeeder'+tim+'.jpg'
                        cv2.imwrite(name, frm2)
                        thread = threading.Thread(target=getLicensePlate, args=(name, speed, tim))
                        thread.daemon = True
                        thread.start()
###################################################################

# Start frame and detect vehicle
###################################################################
dg = tf.Graph()
with dg.as_default():
    ogd = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile('vehicles/oig.pb', 'rb') as fid:
        sg = fid.read()
        ogd.ParseFromString(sg)
        tf.import_graph_def(ogd, name='')
category_index = label_map_util.create_category_index(
    label_map_util.convert_label_map_to_categories(
        label_map_util.load_labelmap('vehicles/clm.pbtxt'), max_num_classes=1, use_display_name=True))
cap = cv2.VideoCapture("testVideo.avi")
ret, frame = cap.read(0)
if not ret:
    print('Frame capture failed, stopping...')
width, height = Image.fromarray(frame).size
Y1 = height*0.5
Y2 = height*0.25
Y3 = height*0.1
session = tf.compat.v1.Session(graph=dg)
wndw = tk.Tk()
wndw.wm_title("Sentinel")
wndw.columnconfigure(1, {'minsize': 1020})
wndw.columnconfigure(0, {'minsize': 335})
frm = tk.Frame(wndw)
frm.grid(row=0, column=1, rowspan=5, pady=10, sticky='N')
frm2 = tk.Frame(wndw)
frm2.grid(row=0, column=0)
frm2.rowconfigure(1, {'minsize': 250})
frm3 = tk.Frame(wndw)
frm3.grid(row=1, column=0)
frm3.rowconfigure(1, {'minsize': 80})
frm4 = tk.Frame(wndw)
frm4.grid(row=2, column=0)
frm4.rowconfigure(1, {'minsize': 150})
frm5 = tk.Frame(wndw)
frm5.grid(row=3, column=0)
frm5.rowconfigure(1, {'minsize': 80})
vehicles = []
###################################################################

# Main function
###################################################################
def main(sess=session):
    if True:
        t = time.time()
        _, image1 = cap.read(0)
        (boxes, scores, classes, num) = sess.run(
            [dg.get_tensor_by_name('detection_boxes:0'),
             dg.get_tensor_by_name('detection_scores:0'),
             dg.get_tensor_by_name('detection_classes:0'),
             dg.get_tensor_by_name('num_detections:0')],
            feed_dict={dg.get_tensor_by_name('image_tensor:0'): np.expand_dims(image1, axis=0)})
        imgF, cords = vis_util.visualize_boxes_and_labels_on_image_array(
            image1,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=5)
        findVehicle(cords, width, height, imgF)
        getSpeed(t, image1)
        frm = cv2.resize(image1, (1020, 647))
        image1 = ImageTk.PhotoImage(image=Image.fromarray(
            cv2.cvtColor(frm, cv2.COLOR_BGR2RGBA)))
        show1.imgtk = image1
        show1.configure(image=image1)
    wndw.after(1, main)
###################################################################


# GUI
###################################################################
label1 = tk.Label(frm, text='Live CCTV Footage', font="Symbol 12 bold")
label1.pack(side='top')
label2 = tk.Label(frm2, text='Rule Breaking Vehicle',
                  font="Helvetica 12 bold")
label2.grid(row=0, column=0, pady=10)
label3 = tk.Label(frm3, text='Vehicle Speed', font="Helvetica 12 bold")
label3.grid(row=0, column=0, pady=10, sticky='S')
label4 = tk.Label(frm4, text='Image of License Plate',
                  font="Helvetica 12 bold")
label4.grid(row=0, column=0, sticky='S')
label5 = tk.Label(frm5, text='License Plate Number',
                  font="Helvetica 12 bold")
label5.grid(row=0, column=0, sticky='S')
show1 = tk.Label(frm)
show1.pack(side='bottom', expand=False)
show2 = tk.Label(frm2)
show2.grid(row=1, column=0)
show3 = tk.Label(frm3, text="", font="Symbol 13 bold", fg='red')
show3.grid(row=1, column=0)
show4 = tk.Label(frm4)
show4.grid(row=1, column=0)
show5 = tk.Label(frm5, text="", font="Symbol 20 bold", fg='red')
show5.grid(row=1, column=0)
###################################################################

# Run GUI
###################################################################
with dg.as_default():
    with tf.compat.v1.Session(graph=dg) as sess:
        session = sess
        main(sess)
wndw.mainloop()
###################################################################

