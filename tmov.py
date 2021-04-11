import pyrealsense2 as rs
import numpy as np
import socket
import cv2
import sys
import time
import board
import digitalio
import pwmio
import adafruit_character_lcd.character_lcd as characterlcd
import threading
import RPi.GPIO as GPIO
import RPi.GPIO as GPIO


# Modify this if you have a different sized character LCD
lcd_columns = 16
lcd_rows = 2
GPIO.setup(4,GPIO.OUT)
GPIO.setup(17,GPIO.OUT)
# Raspberry Pi Pin Config:
lcd_rs = digitalio.DigitalInOut(board.D26)  # LCD pin 4
lcd_en = digitalio.DigitalInOut(board.D19)  # LCD pin 6
lcd_d7 = digitalio.DigitalInOut(board.D27)  # LCD pin 14
lcd_d6 = digitalio.DigitalInOut(board.D22)  # LCD pin 13
lcd_d5 = digitalio.DigitalInOut(board.D24)  # LCD pin 12
lcd_d4 = digitalio.DigitalInOut(board.D25)  # LCD pin 11
lcd_rw = digitalio.DigitalInOut(
    board.D4
)  # LCD pin 5.  Determines whether to read to or write from the display.
# Not necessary if only writing to the display. Used on shield.

red = pwmio.PWMOut(board.D21)
green = pwmio.PWMOut(board.D12)
blue = pwmio.PWMOut(board.D18)

# Initialize the LCD class
# The lcd_rw parameter is optional.  You can omit the line below if you're only
# writing to the display.
lcd = characterlcd.Character_LCD_RGB(
    lcd_rs,
    lcd_en,
    lcd_d4,
    lcd_d5,
    lcd_d6,
    lcd_d7,
    lcd_columns,
    lcd_rows,
    red,
    green,
    blue,
    lcd_rw,
)
RED = [100, 0, 0]
GREEN = [0, 100, 0]
BLUE = [0, 0, 100]
"""
`guassian_blur`
input: color image from the camera.
output: Image wiht a gaussian blur filter applied on it
"""
def gaussian_blur(color_image):
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 0)
    return gray

"""
`detect_humans`
input: color image from the camera.
output: array of detected humans, where each `human`
is represented as [x, y, width, height].
"""
def detect_humans(color_image):
    # initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # using a greyscale picture, also for faster detection
    gray = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)

    # detect people in the image
    # returns the bounding boxes for the detected objects
    humans, _ = hog.detectMultiScale(gray, winStride=(6,6), padding=(2,2), scale=1.05)  

    return humans

"""
`detect_moving_object`
input: first frame with gaussian filter applied 
and the color image from the camera.
output: array of detected objects, where each `object`
is represented as [x, y, width, height].
"""
def detect_moving_objects(first_frame, color_image):
    # Apply gaussian filter on color image from camera
    guassian_image = gaussian_blur(color_image)

    # Calculate the difference between initial frame and current frame
    diff_image= cv2.absdiff(first_frame, guassian_image)

    # Find the contours in the image based on a threshold
    th_delta = cv2.threshold(diff_image, 10, 255, cv2.THRESH_BINARY)[1]
    #th_delta = cv2.adaptiveThreshold(diff_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #                                 cv2.THRESH_BINARY,11,2)
    th_delta = cv2.dilate(th_delta, None, iterations=0)
    (contours, _) = cv2.findContours(th_delta.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Fill the objects array with bounding rectangles of each contour
    objects = []
    for contour in contours:
        if cv2.contourArea(contour) < 10000:
            continue
        objects.append(cv2.boundingRect(contour))
    first_frame = guassian_image

    return objects

"""
`draw_rectangles`
input: color image from the camera
humans as an array of x,y,w,h
objects as an array of x,y,w,h
output: color image with bounding rectangles drawn
"""
def draw_rectangles(color_image, objects,depth_frame):
  #  boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in humans])

#     for (xA, yA, xB, yB) in boxes:
#         # display the detected boxes in the colour picture
#         cv2.rectangle(color_image, (xA, yA), (xB, yB),
#                         (0, 130, 0), 2)
#         # display a circle at the center of detected person
#         cv2.circle(color_image, ((xB+xA)//2, (yB+yA)//2), 5, (255,0,0),1)
    
    for (x, y, w, h) in objects:
        cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 255), 3)
        dist = depth_frame.get_distance((x+w)//2, (y+h)//2)
        dist=str(round(dist,3))
        #cv2.putText(color_image,dist, ((x+w)//2, (y+h)//2), 0, 2,(0, 255, 255),1,cv2.LINE_AA)

    return color_image

"""
`calculate_distances`
input: humans array, depth frame from the camera.
output: array of calculated of distances.
"""
def calculate_distances(humans, depth_frame, log):

    distances = []
    dist = 99999
    closestCoord = []
    distance_sum = 0
    avg_dist = 99999
    to_point = []*3
    #breakpoint()
#     for human in humans:
#         for i in range(human[2]):
#             for j in range(human[3]): #human[0:1] is coordinate of top left corner of the indicator box.
#                 x = human[0] + i
#                 y = human[1] + j
#                 #to get dist need the x and y to map to a unit of measurement (pixel numbers now)
#                 dist =  depth_frame.get_distance(x, y) #returns depth of pixel (x,y) in meters 
#                 distance_sum += dist
#                 #rs.rs2_deproject_pixel_to_point(to_point,[x,y],depth)
#                 #dist = distCalc.distance_calc(to_point[0],to_point[1],depth)
#                 distances.append(dist)
#                 if len(distances) >= 1:
#                     avg_dist = distance_sum/len(distances)
#                 #check if distance is too close, 0.6096 meters = 2'
#                 if avg_dist < leastDist:
#                     leastDist = dist
#                     #closestCoord.append([x,y,dist])
    for (x, y, w, h) in humans:
        x = (x+w)//2
        y = (y+h)//2
        dist = depth_frame.get_distance(x, y)
        if dist <  1.5:#0.6096:
            print("DANGER " + str(dist) +" " +str(x)+""+str(y))
            log.write("DANGER " + str(dist) +" " +str(x)+","+str(y))
            log.write(str(x) + " ," + str(y) + " ," + str(w) + " ," + str(h))
            return -99999

    # x = human[0]dist1
    # y = human[1]
    # dist = depth_frame.get_distance(x, y)
    # distances.append(dist)
    return dist #distances

def lcd_display(lcd, mesg,red):
    
    lcd.clear()
    lcd.message = mesg
    # Wait 5s
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    if red==1:
        GPIO.output(4,GPIO.HIGH)
        
    else:
        GPIO.output(17,GPIO.HIGH)
         
    time.sleep(3)
    GPIO.output(4,GPIO.LOW)    
    GPIO.output(17,GPIO.LOW)

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    first_frame = None
    # Current Day
    
    import socket, sys

    HOST = '127.0.0.1'
    PORT = 65432

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = ((HOST, PORT))
    #message = b'Hello There!'
    
    Day = time.strftime("%m-%d-%Y", time.localtime())
    # Current Time
    Time = time.strftime("%I:%M:%S %p", time.localtime())
    filename = "logs/" + str(Day) + str(Time) + ".log"

    try: 
        # Start streaming
        pipeline.start(config)
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Detect humans and draw rectangles around them
            #humans = detect_humans(color_image)

            # Define the first frame if it's not defined
            # Otherwise detect moving objects
            objects = []
            if first_frame is None:
                first_frame = gaussian_blur(color_image)
            else:
                objects = detect_moving_objects(first_frame, color_image)
            first_frame = gaussian_blur(color_image)
            

            # Draw detected humans AND objects on color_image
            #color_image = draw_rectangles(color_image, humans, objects,depth_frame)
            color_image = draw_rectangles(color_image, objects,depth_frame)

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                images = np.hstack((resized_color_image, depth_colormap))
            else:
                images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)
            
            log = open(filename,"a")
            log.write("\n")
            
            #dist  = calculate_distances(humans, depth_frame,log)
            dist1 = calculate_distances(objects, depth_frame,log)
            if dist1 == -99999:
                red=1
                message = b'abort'
            else:
                red=0
                message = b'OK'
            #print("Human is " + str(dist) + " away\n")
            print("Object is " + str(dist1) + " away\n")
            #log.write("Human is " + str(dist) + " away\n")
            log.write("Object is " + str(dist1) + " away\n")
            
            
            sent = s.sendto(message, address)

            x = threading.Thread(target=lcd_display,args = (lcd,str(dist1),red,), daemon=True)
            x.start()

            log.close()

    except Exception as e:
        print("oops... " + str(e))
        pass

    finally:
        # Stop streaming
        pipeline.stop()
        s.close

if __name__ == "__main__":
    main()

