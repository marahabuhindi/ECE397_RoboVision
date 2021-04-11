import pyrealsense2 as rs

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
        dist = depth.get_distance(291,293)
        dist_feet = dist*3.28084
        print(dist_feet)

    exit(0)

except Exception as e:
    print(e)
    pass
