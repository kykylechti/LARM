import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
print("pipline.start")
pipeline.start(config)
for i in range(1, 10):
    pipeline.wait_for_frames()
    print(f"Received frame {i}")