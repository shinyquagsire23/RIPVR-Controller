import math
import cv2
import cv2.cv as cv
import numpy as np
import numpy
import itertools
from scipy.ndimage import label
import serial
import mmap
import os
import struct
import re
import sys
import threading
import glob
import time
import subprocess

ovrd_mem = "";
CONTROL_VALUES_LEN = 34

def flatten(listOfLists):
    return chain.from_iterable(listOfLists)
    
pi_4 = 4*math.pi

from subprocess import check_output
def get_pid(name):
    return int(check_output(["pidof",name]))

def read_ovrd_mem(pid):
    global ovrd_mem
    memory_permissions = 'rw'
    ovrd_mem = ""
    good = 0
    with open("/proc/%d/maps" % pid, 'r') as maps_file:
        with open("/proc/%d/mem" % pid, 'r', 0) as mem_file:
            for line in maps_file.readlines():  # for each mapped region             
                if "heap" in line:
                    good = 1;
                    continue;
                if good == 0:
                    continue;
                if good < 10:
                    good += 1
                    continue
                    
                m = re.match(r'([0-9A-Fa-f]+)-([0-9A-Fa-f]+) ([-r][-w])', line)
                if m.group(3) == memory_permissions: 
                    start = int(m.group(1), 16)
                    if start > 0xFFFFFFFFFFFF:
                        continue
                    end = int(m.group(2), 16)
                    mem_file.seek(start)  # seek to region start
                    chunk = mem_file.read(end - start)  # read region contents
                    if len(chunk) >= 0x50000:
                        sys.stderr.write( "start = " + hex(start) + "\n")
                        ovrd_mem += chunk
                        good = 0
                        

def segment_on_dt(img):
    border = img - cv2.erode(img, None)

    dt = cv2.distanceTransform(255 - img, 2, 3)
    dt = ((dt - dt.min()) / (dt.max() - dt.min()) * 255).astype(numpy.uint8)
    _, dt = cv2.threshold(dt, 100, 255, cv2.THRESH_BINARY)

    lbl, ncc = label(dt)
    lbl[border == 255] = ncc + 1

    lbl = lbl.astype(numpy.int32)
    cv2.watershed(cv2.cvtColor(img, cv2.COLOR_GRAY2RGB), lbl)
    lbl[lbl < 1] = 0
    lbl[lbl > ncc] = 0

    lbl = lbl.astype(numpy.uint8)
    lbl = cv2.erode(lbl, None)
    lbl[lbl != 0] = 255
    return lbl


def find_circles(frame):
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.GaussianBlur(frame_gray, (7, 7), 1)

    edges = frame_gray - cv2.erode(frame_gray, None)
    _, bin_edge = cv2.threshold(edges, 0, 255, cv2.THRESH_OTSU)
    height, width = bin_edge.shape
    mask = numpy.zeros((height+2, width+2), dtype=numpy.uint8)
    cv2.floodFill(bin_edge, mask, (0, 0), 255)

    circles, obj_center = [], []
    contours, _ = cv2.findContours(frame_gray,
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #print len(contours)

    biggest_area = 200
    for c in contours:
        c = c.astype(numpy.int64) # XXX OpenCV bug.
        area = cv2.contourArea(c)
        if (area >= biggest_area or len(circles) < 2) and area > 200:
            arclen = cv2.arcLength(c, True)
            circularity = (pi_4 * area) / (arclen * arclen)
            if circularity > 0.70: # XXX Yes, pretty low threshold.
                biggest_area = area
                if len(circles) > 2:
                    circles.pop(-1);
                circles.append(c)
                box = cv2.boundingRect(c)
                obj_center.append((box[0] + (box[2] / 2), box[1] + (box[3] / 2)))

    return circles, obj_center

def track_center(objcenter, newdata):
    for i in xrange(len(objcenter)):
        ostr, oc = objcenter[i]
        best = min((abs(c[0]-oc[0])**2+abs(c[1]-oc[1])**2, j)
                for j, c in enumerate(newdata))
        j = best[1]
        if i == j:
            objcenter[i] = (ostr, new_center[j])
        else:
            print "Swapping %s <-> %s" % ((i, objcenter[i]), (j, objcenter[j]))
            objcenter[i], objcenter[j] = objcenter[j], objcenter[i]
    
def get_last_line(ser):
    buffer_string = ''
    while True:
        buffer_string = buffer_string + ser.read(ser.inWaiting())
        if '\n' in buffer_string:
            lines = buffer_string.split('\n') # Guaranteed to have at least 2 entries
            return lines[-2]
            
values = ""            
def get_input():
    global ser;
    global values;
    ports = glob.glob('/dev/ttyACM[0-99]*')
    lastTime = millis = int(round(time.time() * 1000))
    controllerOut = False
    while 1:
        millis = int(round(time.time() * 1000))
        if millis - lastTime > 5000 and controllerOut == False:
            controllerOut = True
            print "Controller out at " + time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        try:
            lastline = get_last_line(ser)
            to_values = lastline.replace("\n","").split(',');
            if len(to_values) != CONTROL_VALUES_LEN:
                print lastline
            else:
                values = to_values
                lastTime = millis
                controllerOut = False
        except:
            values = ""
            
            print "Searching for device..."
            
            while 1:
                ports = glob.glob('/dev/ttyACM[0-99]*')
                if len(ports) >= 1: 
                    break
            
            for port in ports:
                try:
                    print "Connecting to " + port
                    ser = serial.Serial(port)
                    print "Connected!"
                except:
                    print port + " connection failed!"
                    time.sleep(1)

def get_screen_pos(c):
    M = cv2.moments(c)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return cX, cY
        
def get_position(c_f, cX_f, cY_f, xadj, yadj, zadj):
    global size_y;
    global size_x;
    
    area_f = cv2.contourArea(c_f)
    rad_f = math.sqrt(area_f / pi_4) * 2
    rCZ_f = (((size_y / 2) / rad_f) * 54.466) - zadj
    rCX_f = ((25 / rad_f) * cX_f) - rCZ_f - xadj
    rCY_f = ((25 / rad_f) * cY_f) - (rad_f * (size_y / size_x)) - yadj
    return rad_f, rCX_f, rCY_f, rCZ_f
    
def zero_controllers():
    global x_adj, y_adj, z_adj, yaw_adj;
    global x_adj_2, y_adj_2, z_adj_2, yaw_adj_2;
    x_adj += rCX;
    y_adj += rCY;
    z_adj += rCZ;
    #pitch_adj += pitch;
    yaw_adj += yaw;
    #roll_adj += roll;
        
    x_adj_2 += rCX;
    y_adj_2 += rCY;
    z_adj_2 += rCZ;
    #pitch_adj_2 += pitch_2;
    yaw_adj_2 += yaw_2;
    #roll_adj_2 += roll_2;

def sph2cart(az, el, r):
    rcos_theta = r * np.cos(el)
    x = rcos_theta * np.cos(az)
    y = rcos_theta * np.sin(az)
    z = r * np.sin(el)
    return x, y, z
    
from PIL import Image
import select
import v4l2capture

is_dk2 = True
steal_dk2 = False
show_image = True
pid = 0

# Open the camera to establish dominance, then start ovrd
if is_dk2:
    cams = glob.glob('/dev/video[0-99]*')
    video = v4l2capture.Video_device(cams[0])
    size_x, size_y = video.set_format(752, 480)
    video.create_buffers(75)
    video.queue_all_buffers()
    video.start()
    subprocess.Popen(['ovrd'], stdout=open(os.devnull, 'wb'))
# Start ovrd then steal it's cam output
elif steal_dk2:
    subprocess.Popen(['ovrd'], stdout=open(os.devnull, 'wb'))
    size_x = 752
    size_y = 480
    pid = get_pid("ovrd")
# Or just use another camera.
else: 
    cap = cv2.VideoCapture(1)
    
obj_center = None
cX = 0;
cY = 0;
rCX = 0;
rCY = 0;
rCZ = 0;
pitch = 0;
yaw = 0;
roll = 0;
rad = 0;

cX_2 = 0;
cY_2 = 0;
rCX_2 = 0;
rCY_2 = 0;
rCZ_2 = 0;
pitch_2 = 0;
yaw_2 = 0;
roll_2 = 0;
rad_2 = 0;
vx = 0;
vy = 0;
vz = 0;
vx_2 = 0;
vy_2 = 0;
vz_2 = 0;

x_adj = 0;
y_adj = 0;
z_adj = 0;
x_adj_2 = 0;
y_adj_2 = 0;
z_adj_2 = 0;

pitch_adj = 0;
yaw_adj = 0;
roll_adj = 0;
pitch_adj_2 = 0;
yaw_adj_2 = 0;
roll_adj_2 = 0;

input_r = [1,1,1,1,0,0,1,1,1,1,1]
input_l = [1,1,1,1,0,0,1,1,1,1,1]

fd = os.open('/tmp/ripvrcontroller', os.O_CREAT | os.O_TRUNC | os.O_RDWR)
assert os.write(fd, '\x00' * mmap.PAGESIZE) == mmap.PAGESIZE

ipc_buf = mmap.mmap(fd, mmap.PAGESIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)
hax_add = 0x0;

t = threading.Thread(target=get_input, args = ())
t.daemon = True
t.start()

while(1):  
    #DK2
    if is_dk2:
        select.select((video,), (), ())
        image_data = video.read_and_queue()
        image = Image.frombytes('L', (size_x, size_y), image_data)
        #_, img_orig = cv2.threshold(np.array(image), 0, 255, cv2.THRESH_TOZERO)
        _, img = cv2.threshold(np.array(image), 150, 255, cv2.THRESH_TOZERO)
    elif steal_dk2:
        read_ovrd_mem(pid)
        #hax_add += 1
        test_hax = open("memdump.bin", "rb")
        test_hax.seek(0x36600)
        print hex(len(ovrd_mem))
        image_data = ovrd_mem[0+hax_add:]
        #image_data = test_hax.read()[0+hax_add:]
        image = Image.frombytes('L', (size_x, size_y), image_data)
        #_, img_orig = cv2.threshold(np.array(image), 0, 255, cv2.THRESH_TOZERO)
        _, img = cv2.threshold(np.array(image), 0, 255, cv2.THRESH_TOZERO)
    else:
        _, img_orig = cap.read()
        img_gray = cv2.cvtColor(img_orig, cv.CV_BGR2GRAY);
        #img_orig = cv2.cvtColor(img_orig, cv.CV_BGR2GRAY);
        _, img = cv2.threshold(img_gray, 0, 255, cv2.THRESH_TOZERO)
    
    #img = cv2.imread('test.jpg',0)
    #cv2.imwrite( "test.jpg", img );
    
    #DK2
    if is_dk2:
        K = np.array( [[ 712.92391886,    0.,          377.93850445],
                       [   0.,          712.84499101,  223.03834466],
                       [   0.,            0.,            1.        ]])
        d = np.array( [ -5.58371512e-01,   4.54952848e-01,   1.24222639e-03,  -3.40534075e-04, -2.87118229e-01]) # just use first two terms (no translation)
    #PS Eye
    else:
        K = np.array([[  3.26025412e+03,   0.,              3.01456625e+02],
                  [  0.,               3.21208059e+03,  2.23903980e+02],
                  [  0.,               0.,              1.]])
        d = np.array([ -5.09500729e+00,   5.51782957e+02,   1.68189521e-02,  -4.88001221e-03, -4.07164911e+03])
    h, w = img.shape[:2]

    # undistort
    newcamera, roi = cv2.getOptimalNewCameraMatrix(K, d, (w,h), 0) 
    img = cv2.cvtColor(img, cv.CV_GRAY2BGR);
    
    circles, new_center = find_circles(img)
    
    '''
    if(len(circles) == 0):
        rCX = -1;
        rCY = -1;
        rCZ = -1;
    '''
    
    if(len(values) == CONTROL_VALUES_LEN):
        vx += float(values[28])
        vy += float(values[29])
        vz += float(values[30])
        vx_2 += float(values[28])
        vy_2 += float(values[29])
        vz_2 += float(values[30])
    
    if len(circles) == 1:
        c_a = circles[0]
        cX_a, cY_a = get_screen_pos(c_a)
        rad_a, rCX_a, rCY_a, rCZ_a = get_position(c_a, cX_a, cY_a, x_adj, y_adj, z_adj);
        
        a = numpy.array((rCX_a ,rCY_a, rCZ_a))
        r = numpy.array((rCX ,rCY, rCZ))
        l = numpy.array((rCX_2 ,rCY_2, rCZ_2))
        
        dist_ar = numpy.linalg.norm(a-r)
        dist_al = numpy.linalg.norm(a-l)
        #print dist_br, dist_al
        
        if(dist_ar <= dist_al):
            cX = cX_a
            cY = cY_a
            rad = rad_a
            rCX = rCX_a
            rCY = rCY_a
            rCZ = rCZ_a
        else:
            cX_2 = cX_a
            cY_2 = cY_a
            rad_2 = rad_a
            rCX_2 = rCX_a
            rCY_2 = rCY_a
            rCZ_2 = rCZ_a
    elif len(circles) >= 1:
        c_a = circles[0]
        cX_a, cY_a = get_screen_pos(c_a)
        rad_a, rCX_a, rCY_a, rCZ_a = get_position(c_a, cX_a, cY_a, x_adj, y_adj, z_adj);
        
        if len(circles) > 1:
            c_b = circles[1]
            cX_b, cY_b = get_screen_pos(c_b)
            rad_b, rCX_b, rCY_b, rCZ_b = get_position(c_b, cX_b, cY_b, x_adj, y_adj, z_adj);
        else:
            c_b = circles[0]
            cX_b, cY_b = -10000, -10000
            rad_b, rCX_b, rCY_b, rCZ_b = 10000,-10000,-10000,-10000
        
        a = numpy.array((rCX_a ,rCY_a, rCZ_a))
        r = numpy.array((rCX ,rCY, rCZ))
        b = numpy.array((rCX_b, rCY_b, rCZ_b))
        l = numpy.array((rCX_2 ,rCY_2, rCZ_2))
        
        dist_ar = numpy.linalg.norm(a-r)
        dist_br = numpy.linalg.norm(b-r)
        dist_al = numpy.linalg.norm(a-l)
        dist_bl = numpy.linalg.norm(b-l)
        #print dist_br, dist_al
        
        if(dist_bl <= dist_br):
            cX = cX_a
            cY = cY_a
            rad = rad_a
            rCX = rCX_a
            rCY = rCY_a
            rCZ = rCZ_a
            cX_2 = cX_b
            cY_2 = cY_b
            rad_2 = rad_b
            rCX_2 = rCX_b
            rCY_2 = rCY_b
            rCZ_2 = rCZ_b
        else:
            cX = cX_b
            cY = cY_b
            rad = rad_b
            rCX = rCX_b
            rCY = rCY_b
            rCZ = rCZ_b
            cX_2 = cX_a
            cY_2 = cY_a
            rad_2 = rad_a
            rCX_2 = rCX_a
            rCY_2 = rCY_a
            rCZ_2 = rCZ_a

       

    #print float(values[28]), float(values[29]), float(values[30])

    cv2.putText(img, "X-Pos: " + str(rCX), (cX - int(rad) - 20, cY - int(rad) - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(img, "Y-Pos: " + str(rCY), (cX - int(rad) - 20, cY - int(rad) - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(img, "Z-Pos: " + str(rCZ), (cX - int(rad) - 20, cY - int(rad) - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(img, "Radius: " + str(rad), (cX - int(rad) - 20, cY - int(rad) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    
    cv2.putText(img, "X-Pos: " + str(rCX_2), (cX_2 - int(rad_2) - 20, cY_2 - int(rad_2) - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 255, 0), 2)
    cv2.putText(img, "Y-Pos: " + str(rCY_2), (cX_2 - int(rad_2) - 20, cY_2 - int(rad_2) - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 255, 0), 2)
    cv2.putText(img, "Z-Pos: " + str(rCZ_2), (cX_2 - int(rad_2) - 20, cY_2 - int(rad_2) - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 255, 0), 2)
    cv2.putText(img, "Radius: " + str(rad_2), (cX_2 - int(rad_2) - 20, cY_2 - int(rad_2) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 255, 0), 2)
    
    if(len(values) == CONTROL_VALUES_LEN):
        if(values[21] == "0" and values[10] == "0" and values[9] == "0" and values[20] == "0"):
            zero_controllers()
    
        pitch = float(values[0]) - pitch_adj
        yaw = float(values[1]) - yaw_adj
        roll = float(values[2]) - roll_adj
        input_r[0] = int(values[3])
        input_r[1] = int(values[4])
        input_r[2] = int(values[5])
        input_r[3] = int(values[6])
        input_r[4] = int(values[7])
        input_r[5] = int(values[8])
        input_r[6] = int(values[9])
        input_r[7] = int(values[10])
        input_r[8] = int(values[11])
        input_r[9] = int(values[12])
        input_r[10] = int(values[13])
        input_l[0] = int(values[14])
        input_l[1] = int(values[15])
        input_l[2] = int(values[16])
        input_l[3] = int(values[17])
        input_l[4] = int(values[18])
        input_l[5] = int(values[19])
        input_l[6] = int(values[20])
        input_l[7] = int(values[21])
        input_l[8] = int(values[22])
        input_l[9] = int(values[23])
        input_l[10] = int(values[24])
        pitch_2 = float(values[25]) - pitch_adj_2
        yaw_2 = float(values[26]) - yaw_adj_2
        roll_2 = float(values[27]) - roll_adj_2
    
    #Our ball is on the edge of the controller so we need to adjust for that
    xtrans1, ytrans1, ztrans1 = sph2cart(yaw*math.pi/180, pitch*math.pi/180, 100)
    xtrans2, ytrans2, ztrans2 = sph2cart(yaw_2*math.pi/180, pitch_2*math.pi/180, 100)
    
    struct.pack_into("<ffffff11i", ipc_buf, 0, rCX+xtrans1, rCY+ytrans1, rCZ+ztrans1, pitch, yaw, roll, *input_r)
    struct.pack_into("<ffffff11i", ipc_buf, 17*4, rCX_2+xtrans2, rCY_2+ytrans2, rCZ_2+ztrans2, pitch_2, yaw_2, roll_2, *input_l)
    
    if show_image:
        #print(pitch, yaw, roll, pitch_2, yaw_2, roll_2)
        #print len(values)
        if(len(values) == 34):
            cv2.circle(img, (40,30), 5, (255 if values[6] == "1" else 0, 255, 255 if values[6] == "1" else 0), 7)
            cv2.circle(img, (25,15), 5, (255 if values[3] == "1" else 0, 255, 255 if values[3] == "1" else 0), 7)
            cv2.circle(img, (10,30), 5, (255 if values[5] == "1" else 0, 255, 255 if values[5] == "1" else 0), 7)
            cv2.circle(img, (25,45), 5, (255 if values[4] == "1" else 0, 255, 255 if values[4] == "1" else 0), 7)
                
            cv2.rectangle(img, (100,10), (100+20,10+20), (255 if values[10] == "1" else 0, 255, 255 if values[10] == "1" else 0), 3)
            cv2.rectangle(img, (100,50), (100+20,50+20), (255 if values[11] == "1" else 0, 255, 255 if values[11] == "1" else 0), 3)
            cv2.rectangle(img, (100,90), (100+20,90+20), (255 if values[12] == "1" else 0, 255, 255 if values[12] == "1" else 0), 3)
            cv2.rectangle(img, (100,130), (100+20,130+20), (255 if values[13] == "1" else 0, 255, 255 if values[13] == "1" else 0), 3)
                
            stick_x = -(float(values[8]) - 525) / 525
            stick_y = -(float(values[7]) - 525) / 525
            cv2.circle(img, (int(70 + 10*stick_x),int(70 + 10*stick_y)), 5, (255 if (values[9] == "0") else 0, 255, 0), 7)
            
            
            cv2.circle(img, (40+150,30), 5, (255 if values[16] == "1" else 0, 255, 255 if values[16] == "1" else 0), 7)
            cv2.circle(img, (25+150,15), 5, (255 if values[14] == "1" else 0, 255, 255 if values[14] == "1" else 0), 7)
            cv2.circle(img, (10+150,30), 5, (255 if values[17] == "1" else 0, 255, 255 if values[17] == "1" else 0), 7)
            cv2.circle(img, (25+150,45), 5, (255 if values[15] == "1" else 0, 255, 255 if values[15] == "1" else 0), 7)
                
            cv2.rectangle(img, (100+150,10), (100+150+20,10+20), (255 if values[21] == "1" else 0, 255, 255 if values[21] == "1" else 0), 3)
            cv2.rectangle(img, (100+150,50), (100+150+20,50+20), (255 if values[22] == "1" else 0, 255, 255 if values[22] == "1" else 0), 3)
            cv2.rectangle(img, (100+150,90), (100+150+20,90+20), (255 if values[23] == "1" else 0, 255, 255 if values[23] == "1" else 0), 3)
            cv2.rectangle(img, (100+150,130), (100+150+20,130+20), (255 if values[24] == "1" else 0, 255, 255 if values[24] == "1" else 0), 3)
                
            stick_x_2 = -(float(values[19]) - 525) / 525
            stick_y_2 = -(float(values[18]) - 525) / 525
            cv2.circle(img, (int(70 + 150 + 10*stick_x_2),int(70 + 10*stick_y_2)), 5, (255 if (values[20] == "0") else 0, 255, 0), 7)

        for i in xrange(len(circles)):
            cv2.drawContours(img, circles, i, (0, 255, 0))
            #cstr, ccenter = obj_center[i]
        
        cv2.imshow('detected circles',img)

    key = cv2.waitKey(1);
    
    if key & 0xFF == ord('z'):
        zero_controllers()

    if key & 0xFF == ord('q'):
        break
        
    if key & 0xFF == ord('w'):
        hax_add += 0x10
        
    if key & 0xFF == ord('e'):
        hax_add -= 0x10
        
    if key & 0xFF == ord('r'):
        hax_add += 0x1000
        
    #print hex(hax_add)
            
    if key & 0xFF == ord('p'):
        #cv2.imwrite( "test4.png", img_orig );
        cv2.imwrite( "test2.png", img );
    
        
video.close()
cv2.destroyAllWindows()
