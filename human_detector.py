import pyrealsense2 as rs
import numpy as np
import cv2

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
    humans, weights = hog.detectMultiScale(gray, winStride=(8,8)) 

    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in humans])

    for (xA, yA, xB, yB) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(color_image, (xA, yA), (xB, yB),
                        (0, 130, 0), 2)
        # display a circle at the center of detected person
        cv2.circle(color_image, ((xB+xA)//2, (yB+yA)//2), 5, (255,0,0),1)
    
    return humans, color_image

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

            # Deteect humans and draw rectangles around them
            humans, color_image = detect_humans(color_image)

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