import depthai as dai
import cv2
import numpy as np
import time
import serial

#camera configration
pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
cam.setPreviewSize(480, 360)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("video")
cam.preview.link(xout.input)

# for ball color
hmin, smin, vmin = 0, 98, 194
hmax, smax, vmax = 6, 255, 255

# frame width and height
FRAME_WIDTH  = 480
FRAME_HEIGHT = 360

# draw xhit and xgo lines:

#X_HIT_FROM_RIGHT_PX  = 60   # 1 yellow line (Xhit) when we reach it we begin sending to Arduino 
#X_GO_FROM_RIGHT_PX = 30   #2  pink line for solenoid hit time

x_hit_px  = 421 #for prediction
x_go_px = 455 # to active the solenoid وج الروبوت 

# for walls pixles

y_min_px = 11 # 3
y_max_px =  354 #FRAME_HEIGHT - 10 #4

# when we calclate vx = delta_x/delta_t if the value < 15 so its not a moving عالأغلب ضجيج او حركة بسيطة
VX_TOWARD_ROBOT_MIN = 15

# coverting pixet to mm to steps 
MM_FOR_REF = 350 #35cm = 288pixel
PX_FOR_REF = 288 
MM_PER_PX  = MM_FOR_REF / PX_FOR_REF
STEPS_PER_MM = 20 # calculation in word document

Y_ORIGIN_PX = 69 # 7 when we do homing  (begin of the rail where limit switch locate)

#for converting from px to mm we have to subtract y origin before multiplying to mm per px value
def px_to_mm_y(y_px, y_origin_px):
    return (float(y_px) - float(y_origin_px)) * MM_PER_PX

def px_to_steps_y(y_px, y_origin_px):
    mm = px_to_mm_y(y_px, y_origin_px)
    return int(round(mm * STEPS_PER_MM))


current_robot_y_position = None   # robot current position
last_kick_time = 0 #last time solenoid was active
Y_STRIKE_TOLERANCE_PX  = 7     # Yrobot - or + this value  (to see if ball and robot on same y )
STRIKE_COOLDOWN_TIME = 0.25   # time in second between each strike (solenoid)


#serial

ser = None
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
    time.sleep(2)
    print("serial Connected")
except Exception as e:
    print('error')

def send_line(line: str):
    if ser:
        try:
            ser.write((line + '\n').encode('ascii'))
            ser.flush()
        except Exception as e:
            print("[Serial ERROR]", e, "| Fallback:", line)
    else:
        print("[DRY-SEND]", line)

# here we will send commands to Arduino but now its just printing :)

def send_command(new_robot_y_position):
    global current_robot_y_position
    y_px = float(new_robot_y_position)
    if new_robot_y_position > y_max_px:
        y_px = y_max_px
    if new_robot_y_position < y_min_px:
        y_px = y_min_px
    y_mm = px_to_mm_y(y_px, Y_ORIGIN_PX)
    y_st = px_to_steps_y(y_px, Y_ORIGIN_PX)
    current_robot_y_position = y_px 
    print(f" Y target -> {y_px:.1f} px | {y_mm:.1f} mm | {y_st} steps   (send: 'Y:{y_st}')")
    send_line(f"Y:{y_st}")

# considering we have constant velocity so acceleration = 0

def predict_y_at_xhit(pos_xy, vel_xy, y_min, y_max, x_hit, dt_sim=1/360.0, max_steps=2000):
    x, y  = float(pos_xy[0]), float(pos_xy[1])
    vx, vy = float(vel_xy[0]), float(vel_xy[1])

    # since robot in the right side so when vx < 0 it doesnt moving right so ignore it
    if vx <= 0:
        return None  
    
    for _ in range(max_steps):
        x += vx * dt_sim
        y += vy * dt_sim

        if y <= y_min:
            y = y_min
            vy = -vy

        elif y >= y_max:
            y = y_max
            vy = -vy

        if x >= x_hit: # ball reach xhit line 
            return y
    return None

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and param is not None:
        frame = param
        bgr = frame[y, x]
        hsv_val = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0,0]
        print(f"Clicked at X={x}, Y={y}, BGR={bgr}, HSV={hsv_val}")

with dai.Device(pipeline) as device:
    q = device.getOutputQueue(name="video", maxSize=4, blocking=False)

    cv2.namedWindow("OAK-D Lite Ball Tracking")
    cv2.setMouseCallback("OAK-D Lite Ball Tracking", mouse_callback, None)

    # calculating velocity & predicting 
    last_cx = None
    last_cy = None
    last_time = None
    while True:
        frame = q.get().getCvFrame() # get new frame
        cv2.setMouseCallback("OAK-D Lite Ball Tracking", mouse_callback, frame)

        now = time.time()   


        # detecting the ball:
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (hmin, smin, vmin), (hmax, smax, vmax))
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cx = None
        cy = None
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                (xc, yc), r = cv2.minEnclosingCircle(c)
                radius = int(r)
                #ball center
                cv2.circle(frame, (cx, cy), radius, (0,255,0), 2)
                cv2.circle(frame, (cx, cy), 4, (255,0,0), -1)



        if cx is not None and cy is not None:
            if last_cx is None or last_time is None:
                # fist frame
                last_cx, last_cy = float(cx), float(cy)
                last_time = now
            else:
                
                dt = now - last_time
                if dt <= 0:
                    last_time = now
                    continue 


                vx = (float(cx) - last_cx) / dt
                vy = (float(cy) - last_cy) / dt

              
                y_at_xhit = None
                if vx > VX_TOWARD_ROBOT_MIN:
                    y_at_xhit = predict_y_at_xhit(
                        pos_xy=(float(cx), float(cy)),
                        vel_xy=(vx, vy),
                        y_min=y_min_px, y_max=y_max_px,
                        x_hit=x_hit_px, dt_sim=1/360.0
                    )

                if y_at_xhit is not None:
                    cv2.circle(frame, (x_hit_px, int(y_at_xhit)), 7, (255, 0, 0), -1)
                    send_command(y_at_xhit)  

                # active solenoid
                if ((cx +radius)>= x_go_px and vx > VX_TOWARD_ROBOT_MIN and 
                    current_robot_y_position is not None and
                    abs(cy - current_robot_y_position) <= Y_STRIKE_TOLERANCE_PX and
                    (now - last_kick_time) >= STRIKE_COOLDOWN_TIME):
                    last_kick_time = now
                    #here sending command to active solenoid
                    print("gooooooooooooooooooooooooooooo*********")
                    send_line("SOL:1")

                # for the next frame
                last_cx, last_cy = float(cx), float(cy)
                last_time = now
        else:
            # lost the ball :(
            last_cx = last_cy = None
            last_time = None

        #x_hit in yellow
        cv2.line(frame, (x_hit_px,  0), (x_hit_px,  FRAME_HEIGHT-1), (0, 255, 255), 2) 
        #the line where we have to active solenoid pink
        cv2.line(frame, (x_go_px, 0), (x_go_px, FRAME_HEIGHT-1), (255, 0, 255), 2)  

        cv2.imshow("ball tracking", frame)
        if cv2.waitKey(1) in (27, ord('q')):
            break

cv2.destroyAllWindows()

if ser:
    try:
        ser.close()
    except:
        pass