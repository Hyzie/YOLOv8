import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import numpy as np
import cv2
from ultralytics import YOLO
import serial_connect as serial
import time
import threading
import statistics
import logging
logging.getLogger('ultralytics').setLevel(logging.WARNING)
x_des = 0.53
y_des = 0.5

x_current = 0.0
y_current = 0.0

error_chophep = 0.01

line_length = 20
color_red = (0, 0, 255)
color_green = (0, 255, 0)

window = tk.Tk()
window.title('May gap tom')
window.geometry('1280x720')
window.resizable(width=None, height=None)

arduino_vid = 0x2341
arduino_pid = 0x0042
model = YOLO('weights/Final_report/best-segment.pt')
# model = YOLO('yolov8n.pt')
com_port = serial.AutoCOMPort(vid=arduino_vid, pid=arduino_pid)

def button01():
    if com_port.serial_connection:
        # com_port.write_data("Nut nhan 01")
        print("start luồng pid")
        pid_yaw = RobotController(target_x=x_des, target_y=y_des)
        pid_thread_yaw = threading.Thread(target=pid_yaw.PID)
        pid_thread_yaw.start()
def button02():
    if com_port.serial_connection:
        com_port.write_data("Nut nhan 02")
def button03():
    if com_port.serial_connection:
        com_port.write_data("Nut nhan 03")
def button04():
    if com_port.serial_connection:
        com_port.write_data("Nut nhan 04")

def update_frame():
    while(cap.isOpened):
        ret, frame = cap.read()
        if ret:
            start, end = 0, 0
            start = time.time()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = model(frame, conf=0.75)
            end = time.time()

            x_located, y_located = 0, 0
            if results[0].masks is None:
                # print("No masks found in the results.")
                continue
            mask = results[0].masks.xy[0]
            x_list = []
            y_list = []
            for location in mask:
                x_list.append(location[0])
                y_list.append(location[1])
            x_located = int(statistics.median(x_list))
            y_located = int(statistics.median(y_list))

            center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2

            # Center
            cv2.line(frame, (center_x - line_length, center_y), (center_x + line_length, center_y), color_red, 2)
            cv2.line(frame, (center_x, center_y - line_length), (center_x, center_y + line_length), color_red, 2)

            #Shrimp
            cv2.line(frame, (x_located - line_length, y_located), (x_located + line_length, y_located), color_green, 2)
            cv2.line(frame, (x_located, y_located - line_length), (x_located, y_located + line_length), color_green, 2)

            # Speed
            frame = cv2.putText(frame, f'FPS: {round(1 / (end - start),2)}', (440, 470), cv2.FONT_HERSHEY_SIMPLEX, 1,color_red, 1, cv2.LINE_AA)

            annotated_frame = results[0].plot()

            image = Image.fromarray(annotated_frame)

            # from pixel_location to 0-1 location
            x_located = x_located/frame.shape[1]
            y_located = y_located/frame.shape[0]
            distance_x = 0.5 - x_located
            distance_y = 0.5 - y_located
            print(f"x: {x_located} - e_x: {distance_x} ; y: {y_located} - e_y: {distance_y}")
            photo = ImageTk.PhotoImage(image)

            image_label.config(image=photo)
            image_label.image = photo

            window.after(15, update_frame)
        else:
            cap.release()
            window.quit()
        window.mainloop()


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    def compute_control(self, error):
        # PID control algorithm
        self.integral += error
        derivative = error - self.prev_error
        max_khaui = self.ki * self.integral
        control = self.kp * error + max_khaui + self.kd * derivative
        self.prev_error = error
        return control

class RobotController:
    def __init__(self, target_x, target_y):
        self.pid_linear_x = PIDController(kp=1, ki=0, kd=0)
        self.pid_linear_y = PIDController(kp=1, ki=0, kd=0)
        # Target position
        self.target_x = float(target_x)
        self.target_y = float(target_y)

        self.Dir_X = 0
        self.Dir_Y = 0
        # print(f" {x_current} {y_current} ")
        # nếu e_y < 0 -> trục X +
        # nếu e_y > 0 -> trục X -
        # nếu e_x > 0 -> trục Y +
        # nếu e_x < 0 -> trục Y -

    def map_value(self, a, in_min, in_max, out_min, out_max):
        return (a - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    def PID(self):
        global x_current, y_current, error_chophep
        while True:
            error_x = x_current - self.target_x
            u_x = self.pid_linear_x.compute_control(error_x)
            # B = self.map_value(abs(error_x), 0, 0.5, 7999, 1999)
            B = round(self.map_value(abs(error_x), 0, 0.5, 500, 100))

            print(" giá trị",B)
            if (u_x > 0):
                self.Dir_Y = 1
            elif (u_x < 0):
                self.Dir_Y = -1
            else:
                self.Dir_Y = 0

            error_y = y_current - self.target_y
            u_y = self.pid_linear_y.compute_control(error_y)
            if (u_y > 0):
                self.Dir_X = -1
            elif (u_y < 0):
                self.Dir_X = 1
            else:
                self.Dir_X = 0

            print(f"u_control: {round(B, 4)} x:{round(x_current, 4)} --- y:{round(y_current, 4)} ---[e_x:{round(error_x, 4)}, u_x:{round(u_x, 4)}, dir_y: {self.Dir_Y}] [e_y:{round(error_y, 4)}, u_y: {round(error_y, 4)}, dir_x: {self.Dir_X}]")
            # if abs(error_x) < error_chophep and abs(error_y) < error_chophep:
            #     self.Dir_Y = 0
            #     self.Dir_X = 0
            #     print("đã nằm trong khoảng cho phép")
            #     break
            # arduino.send_data(f'{self.Dir_X} {-self.Dir_Y} {round(B, 4)}\n')
            if com_port.serial_connection:
                com_port.write_data(f'{self.Dir_X} {-self.Dir_Y} {round(B, 4)}\n')
            if abs(error_y) < error_chophep and abs(error_x) < error_chophep:
                self.Dir_Y = 0
                self.Dir_X = 0
                print("đã nằm trong khoảng cho phép")
                if com_port.serial_connection:
                    com_port.write_data('0 0 0\n')
                # print(arduino.recv_data())
                break
                # break
            time.sleep(0.1)

# Hien thi hinh anh
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
title_label = ttk.Label(master=window, text='Frame:', font='Calibri 20')
title_label.pack()
image_label = tk.Label(window)
image_label.pack()
update_frame()

Style_r = ttk.Style()
Style_r.configure('r_custom.TButton', background='red', foreground='red', font=('Arial', 20))
Style_g = ttk.Style()
Style_g.configure('g_custom.TButton', background='green', foreground='green', font=('Arial', 20))
Style_b = ttk.Style()
Style_b.configure('b_custom.TButton', background='blue', foreground='blue', font=('Arial', 20))
Style_o = ttk.Style()
Style_o.configure('o_custom.TButton', background='orange', foreground='orange', font=('Arial', 20))

input_frame = ttk.Frame(master=window)
button = ttk.Button(master=input_frame, text='01', style='r_custom.TButton', command=button01)
button.pack(side='left', padx=10)
button2 = ttk.Button(master=input_frame, text='02', style='g_custom.TButton', command=button02)
button2.pack(side='left', padx=10)
button3 = ttk.Button(master=input_frame, text='03', style='b_custom.TButton', command=button03)
button3.pack(side='left', padx=10)
button4 = ttk.Button(master=input_frame, text='04', style='o_custom.TButton', command=button04)
button4.pack(side='left', padx=10)
input_frame.pack(pady=10)

com_port.close()
