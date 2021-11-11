import pyrealsense2 as rs
import numpy as np
import cv2

class CameraL(object):
    def __init__(self):
        self.pipeline=rs.pipeline()   # 配置管道流
        self.config=rs.config()      # 生成配置容器
        self.config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)
        self.config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
        self.align_to=rs.stream.color
        self.align=rs.align(self.align_to)  # 对齐颜色空间，给颜色空间加入深度信息
        self.pipeline_proflie=self.pipeline.start(self.config)
        self.device=self.pipeline_proflie.get_device()
        advanced_mode=rs.rs400_advanced_mode(self.device)
        self.mtx=self.getIntrinsics()
        #with open(r"config/d435_high_accuracy.json", 'r') as file:
        #    json_text = file.read().strip()
        #advanced_mode.load_json(json_text)

        self.hole_filling=rs.hole_filling_filter()

        align_to=rs.stream.color
        self.align=rs.align(align_to)

        # cam init
        print('cam init ...')
        i=60
        while i>0:
            frames=self.pipeline.wait_for_frames()
            aligned_frames=self.align.process(frames) #对其颜色信息
            depth_frame=aligned_frames.get_depth_frame() 
            color_frame=aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image=np.asanyarray(depth_frame.get_data())
            color_image=np.asanyarray(color_frame.get_data())
            i-=1
        print("cam init done.")
    
    def get_data(self,hole_filling=False):
        while True:
            frames=self.pipeline.wait_for_frames()
            aligned_frames=self.align.process(frames)
            depth_frame=aligned_frames.get_depth_frame()
            if hole_filling:
                depth_frame=self.hole_filling.process(depth_frame)
            color_frame=aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image=np.asanyarray(depth_frame.get_data())
            color_image=np.asanyarray(color_frame.get_data())
            break
        return color_image,depth_image
    
    def inpaint(self,img,missing_value=0):
        '''
        pip opencv-python == 3.4.8.29
        :param image:
        '''

    def get_data(self,hole_filling=False):
        while True:
            frames=self.pipeline.wait_for_frames()
            aligned_frames=self.align.process(frames)
            depth_frame=aligned_frames.get_depth_frame()
            if hole_filling:
                depth_frame=self.hole_filling.process(depth_frame)
            color_frame=aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image=np.asanyarray(depth_frame.get_data())
            color_image=np.asanyarray(color_frame.get_data())
            break
        return color_image,depth_frame
    
    def getIntrinsics(self):
        frames=self.pipeline.wait_for_frames()
        aligned_frames=self.align.process(frames)
        color_frame=aligned_frames.get_color_frame()
        intrinsics=color_frame.get_profile().as_video_stream_profile().get_intrinsics()
        mtx=[intrinsics.width,intrinsics.height,intrinsics.ppx,intrinsics.ppy,intrinsics.fx,intrinsics.fy]
        camIntrinsics=np.array([[mtx[4],0,mtx[2]],
                                [0,mtx[5],mtx[3]],
                                [0,0,1.]])
        return camIntrinsics