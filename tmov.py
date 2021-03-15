import pyrealsense2 as rs
import numpy as np
import cv2

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
def draw_rectangles(color_image, humans, objects,depth_frame):
    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in humans])

    for (xA, yA, xB, yB) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(color_image, (xA, yA), (xB, yB),
                        (0, 130, 0), 2)
        # display a circle at the center of detected person
        cv2.circle(color_image, ((xB+xA)//2, (yB+yA)//2), 5, (255,0,0),1)
    
    for (x, y, w, h) in objects:
        cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 255), 3)
        dist = depth_frame.get_distance((x+w)//2, (y+h)//2)
        dist=str(round(dist,3))
        cv2.putText(color_image,dist, ((x+w)//2, (y+h)//2), 0, 2,(0, 255, 255),1,cv2.LINE_AA)

    return color_image

"""
`calculate_distances`
input: humans array, depth frame from the camera.
output: array of calculated of distances.
"""
def calculate_distances(humans, depth_frame):


    distances = []
    for human in humans:
        x = human[0]
        y = human[1]
        dist = depth_frame.get_distance(x, y)
        distances.append(dist)
    return distances


def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    first_frame = None

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
            humans = detect_humans(color_image)

            # Define the first frame if it's not defined
            # Otherwise detect moving objects
            objects = []
            if first_frame is None:
                first_frame = gaussian_blur(color_image)
            else:
                objects = detect_moving_objects(first_frame, color_image)
            first_frame = gaussian_blur(color_image)
            

            # Draw detected humans AND objects on color_image
            color_image = draw_rectangles(color_image, humans, objects,depth_frame)

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

    except Exception as e:
        print(e)
        pass

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    main()
