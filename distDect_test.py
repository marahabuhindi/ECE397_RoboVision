import distCalc
import pyrealsense2 as rs
import pdb

def distDetect():
    leastDist = 99999
    closestCoord = []
    to_point = []*3
    distances = []
    init = rs.intrinsics()
    try:
        pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        pipeline.start(config)

        while True:
            frames = pipeline.wait_for_frames()
            depth = frames.get_depth_frame()
            if not depth: continue
            #might need break
            depth0 = depth.get_distance(300,250)
            print(depth0)
            breakpoint()
            to_point = rs.rs2_deproject_pixel_to_point(init,[300,250],depth0)
            dist = distCalc.distance_calc(to_point[0],to_point[1],depth0)
            distances.append(dist)
            print(dist)
            if dist <  0.6096:
                print("DANGER") 
                return -99999, closestCoord
            if dist < leastDist:
                leastDist = dist
                closestCoord.append(x,y,dist)
            return leastDist

    #to get dist need the x and y to map to a unit of measurement (pixel numbers now)
    #depth =  depth_frame.get_distance(300,250) #returns depth of pixel (x,y) in meters 
    #rs.rs2_deproject_pixel_to_point(to_point,[300,250],depth)
    #dist = distCalc.distance_calc(to_point[0],to_point[1],depth)
    #distances.append(dist)

    #check if distance is too close, 0.6096 meters = 2'


        exit(0)

    except Exception as e:
        print(e)
        pass
        
print(distDetect())
