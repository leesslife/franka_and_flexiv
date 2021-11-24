import cv2
from numpy.lib.npyio import savetxt
import pyrealsense2 as rs
import numpy as np
import os
import json
import h5py

class realsense:

    def __init__(self,config=None):
        # 获取配置信息
        self.config=config if config!=None else rs.config()
        # 获取设备号的名字与序列号
        self.name,self.serial_number = self.getCamInfo()

    def start(self,resolution='720p',depth_preset=3,fps=30,
              color_scheme=0,histogram_equalization=True,enable_ir_emitter=False):
        '''
        开启realsense, 若想使用realsense, 必须先调用这一句

        Parameters
        ----------
        resolution :str, optional
            分辨率, 仅支持两种分辨率 '720p' -- (1280x720 长x宽). The default is '720p'.
        depth_preset : int, optional
            深度模式设置 区间[0,4],The default is 3.
            常用  3:高精度模式
                  4: 高密度模式
                  5: 中密度模式
        fps : 帧率, optional
            支持 30,16,8. The default is 30.
        color_scheme : int, optional
            深度图着色模式,区间[0,8]. The default is 0.
        histogram_equalization : bool, optional
            是否对对着色的深度图作直方图均布. The default is True.
        enable_ir_emitter :bool, optional
            ir发射器, 一般需要开启, 不开的话深度图质量会下降. The default is False.

        Returns
        -------
        None.

        '''
        # 开启realsense        
        if resolution=='480p':
            self.W,self.H=640, 480
        elif resolution=='720p':
            self.W,self.H=1280, 720
            #self.W,self.H=640,360
        else:
            self.W,self.H=1280, 720
            resolution = '720p'
            
        self.resolution=resolution
        self.fps=fps        
        
        self.config.enable_stream(rs.stream.depth,self.W, self.H, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, self.W, self.H, rs.format.bgr8, fps)
        
        # 把每台摄像头的序列号加入进入，用于连接不同的设备
        connect_device = []
        for d in rs.context().devices:
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                connect_device.append(d.get_info(rs.camera_info.serial_number))

        self.init_cam_devices(connect_device)

    def init_cam_devices(self,connect_device=None
                         ,depth_preset=3,fps=30,color_scheme=0,histogram_equalization=True,enable_ir_emitter=True):
        #print(connect_device)
    
        self.pipeline_flexiv = rs.pipeline()
        self.pipeline_franka = rs.pipeline()
        self.config.enable_device(connect_device[0])
        self.pipeline_profile_flexiv = self.pipeline_flexiv.start(self.config)
        # 设置彩色相机
        color_sensor=self.pipeline_profile_flexiv.get_device().query_sensors()[1] 
        color_sensor.set_option(rs.option.sharpness,100) # 设置锐度为100(最大)
        
        # 设置深度模式
        self.depth_sensor_flexiv=self.pipeline_profile_flexiv.get_device().first_depth_sensor()
        self.depth_sensor_flexiv.set_option(rs.option.visual_preset,depth_preset) # 设置深度模式
        self.depth_sensor_flexiv.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
        self.depth_scale_flexiv = self.depth_sensor_flexiv.get_depth_scale()    # 获取深度比例因子

        self.default_align_flexiv = rs.align(rs.stream.color) # 定义一个默认 对齐参数
        self.default_colorizer_flexiv=rs.colorizer() # 定义一个默认 深度图着色器
        self.default_colorizer_flexiv.set_option(rs.option.color_scheme,color_scheme)
        self.default_colorizer_flexiv.set_option(rs.option.histogram_equalization_enabled,
                                          histogram_equalization)

        self.config.enable_device(connect_device[1])
        self.pipeline_profile_franka = self.pipeline_franka.start(self.config)
        
        # 设置彩色相机
        color_sensor=self.pipeline_profile_franka.get_device().query_sensors()[1] 
        color_sensor.set_option(rs.option.sharpness,100) # 设置锐度为100(最大)
        
        # 设置深度模式
        self.depth_sensor_franka=self.pipeline_profile_franka.get_device().first_depth_sensor()
        self.depth_sensor_franka.set_option(rs.option.visual_preset,depth_preset) # 设置深度模式
        self.depth_sensor_franka.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
        self.depth_scale_franka= self.depth_sensor_franka.get_depth_scale()    # 获取深度比例因子

        self.default_align_franka = rs.align(rs.stream.color) # 定义一个默认 对齐参数
        self.default_colorizer_franka=rs.colorizer() # 定义一个默认 深度图着色器
        self.default_colorizer_franka.set_option(rs.option.color_scheme,color_scheme)
        self.default_colorizer_franka.set_option(rs.option.histogram_equalization_enabled,
                                              histogram_equalization)

    def stop(self):
        # 关闭realsense
        self.pipeline_flexiv.stop()
        self.pipeline_franka.stop()
        
    def getCamInfo(self):
        # 获取realsense 型号 及 出厂序列号
        ctx = rs.context()
        serial_numbers=()
        names=()
        if len(ctx.devices) > 0:
            for i in ctx.devices:
                serial_number=i.get_info(rs.camera_info.serial_number)
                name=i.get_info(rs.camera_info.name)
                serial_numbers+=(serial_number,)
                names+=(name,)
                print ('Found device: ',
                name, ' ',
                serial_number)
        else:
            print("No Device Connected")
        return names,serial_numbers
    
    #######################raw_frame#######################
    # raw_frame的单独处理
    # 以应对 循环,或 单帧需要多种处理的情况
    #def rawFrame(self):
    #    # 获取 一帧raw_frame
    #    raw_frames =self.pipeline.wait_for_frames()
    #    return raw_frames
    
    #def rawProcess(self,raw_frames,*post_filter,align=None):
        '''
        处理raw_frame, 拆分其为 raw_color_frame,raw_depth_frame

        Parameters
        ----------
        raw_frames : obj
            DESCRIPTION.
        *post_filter : 滤波器
            可不填.
        align : obj, optional
            若为None则使用默认的对齐参数, 否则使用用户定义的对齐参数. The default is None.

        Returns
        -------
        
        raw_color_frame : obj
            DESCRIPTION.
        raw_depth_frame : obj
            DESCRIPTION.

        '''
       # if len(post_filter)!=0:
        #    for i in post_filter:
       #         raw_frames=i.process(raw_frames)
        #    raw_frames=raw_frames.as_frameset()
            
        # 对齐
        #aligned_frame=self.default_align.process(raw_frames) if align==None\
       # else align.process(raw_frames)
        #raw_depth_frame = aligned_frame.get_depth_frame()
        #raw_color_frame = aligned_frame.get_color_frame()
       # return raw_color_frame,raw_depth_frame
    
    #def rawColorize(self,raw_depth_frame,colorizer=None):
        # 着色
     #   raw_depth_frame_colorized=self.default_colorizer.colorize(raw_depth_frame) \
     #   if colorizer==None else colorizer.colorize(raw_depth_frame)
     #   return raw_depth_frame_colorized
        
    #def raw2Numpy(self,raw_frame):
        # raw_frame转为 numpy 的array
    #    numpy_frame=np.asanyarray(raw_frame.get_data())
    #    return numpy_frame
    
    #########################################################
    
    def numpyFrame(self,*post_filter,colorizer=None,align=None,robot_name="Flexiv"):
        # 单次拍照,直接将图像转为numpy.array--即raw_frame函数的集合体
        # 共返回3个array, 分别为
        # color_img: 彩色图片 uint8,3通道
        # depth_map: 深度图   uint16,单通道
        # depth_img: 着色后的深度图 uint8, 3通道
    
        color_depth_frame_flexiv =self.pipeline_flexiv.wait_for_frames()
        color_depth_frame_franka =self.pipeline_franka.wait_for_frames()

        if len(post_filter)!=0:
            for i in post_filter:
                color_depth_frame_flexiv=i.process(color_depth_frame_flexiv)
                color_depth_frame_franka=i.process(color_depth_frame_franka)

            color_depth_frame_flexiv=color_depth_frame_flexiv.as_frameset()
            color_depth_frame_franka=color_depth_frame_franka.as_frameset()

        # 官方文档明确说明, 滤波器应先于对齐
        # 对齐
        aligned_frame_flexiv=self.default_align_flexiv.process(color_depth_frame_flexiv)
        raw_depth_frame_flexiv = aligned_frame_flexiv.get_depth_frame()
        raw_color_frame_flexiv = aligned_frame_flexiv.get_color_frame()

        aligned_frame_franka=self.default_align_franka.process(color_depth_frame_franka)
        raw_depth_frame_franka =aligned_frame_franka.get_depth_frame()
        raw_color_frame_franka =aligned_frame_franka.get_color_frame()

        # 如果未自定义colorizer,则使用预定义的colorizer
        raw_depth_frame_colorized_flexiv=self.default_colorizer_flexiv.colorize(raw_depth_frame_flexiv) 
        raw_depth_frame_colorized_franka=self.default_colorizer_franka.colorize(raw_depth_frame_franka)\
        if colorizer==None else colorizer.colorize(raw_depth_frame_flexiv) and colorizer.colorize(raw_depth_frame_franka)

        if robot_name=="Flexiv":
            depth_map = np.asanyarray(raw_depth_frame_flexiv.get_data())
            color_img = np.asanyarray(raw_color_frame_flexiv.get_data())
            depth_img = np.asanyarray(raw_depth_frame_colorized_flexiv.get_data())
            return color_img,depth_map,depth_img
        else:
            depth_map = np.asanyarray(raw_depth_frame_franka.get_data())
            color_img = np.asanyarray(raw_color_frame_franka.get_data())
            depth_img = np.asanyarray(raw_depth_frame_colorized_franka.get_data())
            return color_img,depth_map,depth_img
    
    def getCamPara(self,save=False,save_dir='./cam_para',unaligned=False,robot_name="Flexiv"):
        
        frames_flexiv = self.pipeline_flexiv.wait_for_frames()
        aligned_frames_flexiv = self.default_align_flexiv.process(frames_flexiv)
        depth_frame_flexiv = aligned_frames_flexiv.get_depth_frame()
        color_frame_flexiv = aligned_frames_flexiv.get_color_frame()
        

        frames_franka = self.pipeline_franka.wait_for_frames()
        aligned_frames_franka = self.default_align_franka.process(frames_franka)
        depth_frame_franka = aligned_frames_franka.get_depth_frame()
        color_frame_franka = aligned_frames_franka.get_depth_frame()
        
        # 下面两行的所求的内参为相机的原始内参, 但经对齐后相机内参有变
        if unaligned==True:

            color_intr_flexiv = self.pipeline_profile_flexiv.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            depth_intr_flexiv = self.pipeline_profile_flexiv.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

            color_intr_franka = self.pipeline_profile_franka.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            depth_intr_franka = self.pipeline_profile_franka.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

        # 下两行语句为相机对齐后的内参, 有别于原始内参
        else:
            color_intr_flexiv = color_frame_flexiv.profile.as_video_stream_profile().intrinsics
            depth_intr_flexiv = depth_frame_flexiv.profile.as_video_stream_profile().intrinsics

            color_intr_franka = color_frame_franka.profile.as_video_stream_profile().intrinsics
            depth_intr_franka = depth_frame_franka.profile.as_video_stream_profile().intrinsics
        
        # 外参矩阵
        depth_to_color_extr_flexiv = depth_frame_flexiv.profile.get_extrinsics_to(color_frame_flexiv.profile)
        depth_to_color_extr_franka = depth_frame_franka.profile.get_extrinsics_to(color_frame_franka.profile)
        
        # Fliexiv 内参
        depth_cam_matrix_flexiv=np.array([
                                          [depth_intr_flexiv.fx,0,                    depth_intr_flexiv.ppx],
                                          [0,                   depth_intr_flexiv.fy, depth_intr_flexiv.ppy],
                                          [0,                   0,                    1]])
        depth_scale_flexiv = self.depth_sensor_flexiv.get_depth_scale()    
        depth_dist_coeff_flexiv=np.array(depth_intr_flexiv.coeffs)

        color_cam_matrix_flexiv=np.array([
                                         [color_intr_flexiv.fx, 0,                     color_intr_flexiv.ppx],
                                         [0,                    color_intr_flexiv.fy,  color_intr_flexiv.ppy],
                                         [0,                    0,                     1]])
        color_dist_coeff_flexiv=np.array(color_intr_flexiv.coeffs)

        # Franka内参
        depth_cam_matrix_franka=np.array([
                                          [depth_intr_franka.fx,0,                    depth_intr_franka.ppx],
                                          [0,                   depth_intr_franka.fy, depth_intr_franka.ppy],
                                          [0,                   0,                    1]])
        depth_scale_franka = self.depth_sensor_franka.get_depth_scale()    
        depth_dist_coeff_franka=np.array(depth_intr_franka.coeffs)

        color_cam_matrix_franka=np.array([
                                         [color_intr_franka.fx, 0,                     color_intr_franka.ppx],
                                         [0,                    color_intr_franka.fy,  color_intr_franka.ppy],
                                         [0,                    0,                     1]])   
        color_dist_coeff_franka=np.array(color_intr_franka.coeffs)

        cam_para_json={}
        if robot_name=="Flexiv":
            cam_para={'color_cam_matrix_flexiv':color_cam_matrix_flexiv,
                      'color_dist_coeff_flexiv':color_dist_coeff_flexiv,
                      'depth_cam_matrix_flexiv':depth_cam_matrix_flexiv,
                      'depth_dist_coeff_flexiv':depth_dist_coeff_flexiv,
                      'depth_scale_flexiv':depth_scale_flexiv,
                      'W_flexiv':color_intr_flexiv.width,
                      'H_flexiv':color_intr_flexiv.height}
        elif robot_name=="Franka":
            cam_para={'color_cam_matrix_franka':color_cam_matrix_franka,
                      'color_dist_coeff_franka':color_dist_coeff_franka,
                      'depth_cam_matrix_franka':depth_cam_matrix_franka,
                      'depth_dist_coeff_franka':depth_dist_coeff_franka,
                      'depth_scale_franka':depth_scale_franka,
                      'W_franka':color_intr_franka.width,
                      'H_franka':color_intr_franka.height}
        else:
            print("robot_name is wrong!")
        
        if robot_name=="Flexiv":
            if save==True:
                cam_para_json={'color_cam_matrix_flexiv':color_cam_matrix_flexiv.tolist(),
                               'color_dist_coeff_flexiv':color_intr_flexiv.coeffs,
                               'depth_cam_matrix_flexiv':depth_cam_matrix_flexiv.tolist(),
                               'depth_dist_coeff_flexiv':depth_intr_flexiv.coeffs,
                               'depth_scale_flexiv':depth_scale_flexiv,
                               'W_flexiv':color_intr_flexiv.width,
                               'H_flexiv':color_intr_flexiv.height}

                if os.path.exists(save_dir)==False:
                    os.mkdir(save_dir)
                name =input('请输入文件名: ')
                if len(name)!=0:
                    if not name.endswith('.json'):
                        name = ''.join([name,'.json'])
                    path=os.path.join(save_dir,name)
                    with open(path,'w') as f:
                        json.dump(cam_para_json,f,indent=1)
                    print('已保存相机参数至',path)
                else:
                    print('未检测到文件名输入!')
        elif robot_name=="Franka":
            if save==True:
                cam_para_json={'color_cam_matrix_franka':color_cam_matrix_franka.tolist(),
                               'color_dist_coeff_franka':color_intr_franka.coeffs,
                               'depth_cam_matrix_franka':depth_cam_matrix_franka.tolist(),
                               'depth_dist_coeff_franka':depth_intr_franka.coeffs,
                               'depth_scale_franka':depth_scale_franka,
                               'W_franka':color_intr_franka.width,
                               'H_franka':color_intr_franka.height}
                if os.path.exists(save_dir)==False:
                    os.mkdir(save_dir)
                name =input('请输入文件名: ')
                if len(name)!=0:
                    if not name.endswith('.json'):
                        name = ''.join([name,'.json'])
                    path=os.path.join(save_dir,name)
                    with open(path,'w') as f:
                        json.dump(cam_para_json,f,indent=1)
                    print('已保存相机参数至',path)
                else:
                    print('未检测到文件名输入!')
        else:
            print("robot_name is wrong!")
        if robot_name=="Flexiv":
            return color_intr_flexiv,depth_intr_flexiv,depth_to_color_extr_flexiv,cam_para_json
        elif robot_name=="Franka":
            return color_intr_franka,depth_intr_franka,depth_to_color_extr_franka,cam_para_json
        else:
            print("robot name is wrong!")

    def videoStream(self,*post_filter,max_shotcut=100,colorizer=None,align=None,
                    save_depth_img=True,save_depth_map=True,
                    record_color_img=True,record_depth_img=True,record_depth_map=True,
                    color_img_video_name='color_img.avi',depth_img_video_name='depth_img.avi',
                    depth_map_h5_name='depth_map.h5',save_dir='./realsense'):

        os.makedirs(save_dir,exist_ok=True)
        video_save_dir = save_dir+'/video'
        os.makedirs(video_save_dir,exist_ok=True)        

        print('按s拍照,按space录像,按esc退出,按s清空save_dir下所有图片,图片或视频保存于', save_dir)
        shotcut=0
        frame_id=0
        #win_name='Realsense'+ '(' +self.resolution + ')'
        win_flexiv="Realsense flexiv"
        win_franka="Realsense franka"
        #fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # cv2.VideoWriter参数:
        # 1-需保存视频的文件名
        # 2-解码参数
        # 3-fps帧率
        # 4-分辨率
        # 5-True为彩色视频, False为黑白视频
        #if record_color_img==True:
        #    color_img_video_path=os.path.join(video_save_dir,color_img_video_name)
        #    color_img_video= cv2.VideoWriter(color_img_video_path,
        #                                     fourcc,self.fps, (self.W,self.H),True)
        #if record_depth_img==True:
        #    depth_img_video_path=os.path.join(video_save_dir,depth_img_video_name)
        #    depth_img_video= cv2.VideoWriter(depth_img_video_path,
        #                                     fourcc,self.fps, (self.W,self.H),True)
        #if record_depth_map==True:
        #    depth_map_h5_path=os.path.join(video_save_dir,depth_map_h5_name)
        #    depth_map_h5=h5py.File(depth_map_h5_path,'w')
            
        #cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        #录像参数初始化
        recording=False 
        while 1:
            # try用于预防相机松动,连接线接触不良等以外情况导致的程序卡死
            #try:
                key = cv2.waitKey(1) & 0xFF    
                color_img_flexiv, depth_map_flexiv, depth_img_flexiv=self.numpyFrame(*post_filter,colorizer=colorizer,align=align,robot_name="Flexiv")
                color_img_franka, depth_map_franka, depth_img_franka=self.numpyFrame(*post_filter,colorizer=colorizer,align=align,robot_name="Franka")
                #color_img_flexiv, depth_map_flexiv, depth_img_flexiv=self.numpyFrame(*post_filter,align=align,robot_name="Flexiv")
                #color_img_franka, depth_map_franka, depth_img_franka=self.numpyFrame(*post_filter,align=align,robot_name="Franka")
                color_depth_flexiv=np.hstack((color_img_flexiv, depth_img_flexiv))
                color_depth_franka=np.hstack((color_img_franka, depth_img_franka))
                color_depth=np.vstack((color_depth_flexiv,color_depth_franka))
                cv2.imshow(win_flexiv, color_depth)
                #cv2.imshow(win_franka,color_depth_franka)
                # 录像
                #if key== 32 and recording==False:
                #    recording = True
                #    print('recording start')
                #    cv2.setWindowTitle(win_name, win_name+'--recording...')
                #elif key== 32 and recording==True:
                #    recording = False
                #    print('recording end')
                #    cv2.setWindowTitle(win_name,win_name)
                #if recording==True:
                #    frame_id+=1
                #    if record_color_img==True:
                #        color_img_video.write(color_img)
                #    if record_depth_img==True:
                #        depth_img_video.write(depth_img)
                #    if record_depth_map==True:
                #        depth_map_name='depth_map'+str(frame_id).zfill(6)+'.png'
                #        depth_map_buffer=cv2.imencode('.png',depth_map)[1]
                #        depth_map_h5[depth_map_name]=depth_map_buffer
    
                # 拍照
                if key== ord('s'):
                    self.saveImg(color_img_flexiv,shotcut,name='color_img_flexiv',suffix='.jpg',save_dir=save_dir)
                    self.saveImg(color_img_franka,shotcut,name='color_img_franka',suffix='.jpg',save_dir=save_dir)
                    if save_depth_map==True:
                        self.saveImg(depth_map_flexiv,shotcut,name='depth_map_flexiv',suffix='.jpg',save_dir=save_dir)
                        self.saveImg(depth_map_franka,shotcut,name='depth_map_franka',suffix='.jpg',save_dir=save_dir)
                    if save_depth_img==True:
                        self.saveImg(depth_img_flexiv,shotcut,name='depth_img_flexiv',suffix='.jpg',save_dir=save_dir)
                        self.saveImg(depth_img_franka,shotcut,name='depth_img_franka',suffix='.jpg',save_dir=save_dir)
                    print('shotcut'+str(shotcut))
                    shotcut+=1
                    if shotcut==max_shotcut:
                        shotcut=0
                        
                # 删除文件夹下所有color_img,depth_map,depth_img
                #if key==ord('d'):
                #    for i in os.listdir(save_dir):
                #        if 'color_img_flexiv' in i or 'depth_map' in i or 'depth_img' in i:
                #            path = os.path.join(save_dir,i)
                #            if path.endswith('.jpg') or path.endswith('.png'):
                #                os.remove(path)
                #    print('图片已清空')
                #    shotcut=0
                
                # 退出
                if key== 27: # esc
                    break
            
            # 若意外发生则关闭循环
            #except:
                #print('error!')
                #break
        cv2.destroyAllWindows()
        #if record_depth_map==True:
        #    depth_map_h5.close()
        #if record_depth_img==True:
        #    depth_img_video.release()
        #if record_color_img==True:
        #    color_img_video.release()
        print('video stream end')

    # 凡带filter字段的函数均为滤波器,如需使用, 请输入予函数中的 post_filter参数
    # filter 下注释为参数取值范围(左闭右闭)
    def align(self,alignto='color'):
        if alignto=='color' or alignto==0:
            alignto=rs.stream.color
        elif alignto=='depth' or alignto==1:
            alignto=rs.stream.depth
        elif alignto=='any' or alignto==2:
            alignto=rs.stream.any
        align=rs.align(alignto)
        return align
    
    def filterTemporal(self, alpha=0.5, delta=20):
        # 时间滤波器
        # alpha [0:1]
        # delta [0:100]
        filter_temporal = rs.temporal_filter()
        filter_temporal.set_option(rs.option.filter_smooth_alpha,alpha)
        filter_temporal.set_option(rs.option.filter_smooth_delta,delta)
        return filter_temporal
    def filterThreshold(self,min_distance=0,max_distance=16):
        # 距离滤波, [min_distance,max_distance]区间内的深度予以保留, 其他区间深度置0
        # distance [0:16] 单位m
        filter_threshold=rs.threshold_filter()
        filter_threshold.set_option(rs.option.min_distance,min_distance)
        filter_threshold.set_option(rs.option.max_distance,max_distance)
        return filter_threshold
    def filterSpatial(self,magnitude=2,alpha=0.5,delta=20):
        # magnitude [0:5] 整数
        # alpha [0:1]
        # delta [0:100]
        # 保边平滑滤波器
        # 根据根据realsense官方文档, 该滤波器 功能与 cv2.edgePreservingFilter (flags = cv2.RECURS_FILTER)完全一致
        # 即原论文中的'recursive filter'
        # 注意此处flag 不为 cv2.NORMCONV_FILTER, 即原论文中normalized convolution fliter 由于该方法对导数采用后处理导致计算效率极低
        # recursive filter 效率高于 normalized convolution fliter,但二者效果相近
        # 待复现,原因为cv2.edgePreservingFilter 仅支持uint8 图片处理
        # 参见realsense文档https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md

        filter_spatial=rs.spatial_filter()
        filter_spatial.set_option(rs.option.filter_magnitude,magnitude)
        filter_spatial.set_option(rs.option.filter_smooth_alpha,alpha)
        filter_spatial.set_option(rs.option.filter_smooth_delta,delta)
        return filter_spatial
    
    def filterDecimation(self,magnitude=2):
        # 降采样滤波
        # magnitude [0:5] 整数
        filter_decimation=rs.decimation_filter()
        filter_decimation.set_option(rs.option.filter_magnitude,magnitude)
        return filter_decimation
    def filterDisparity(self):
        # 不建议使用
        disparity = rs.disparity_transform()
        return disparity
    
    def filterHoleFilling(self,filling_mode=1):
        # filling_mode [0:3] 整数
        filter_hole_filling=rs.hole_filling_filter()
        filter_hole_filling.set_option(rs.option.holes_fill,filling_mode)
        return filter_hole_filling
    
    def colorizer(self,color_scheme=0,histogram_equalization=True):
        # colorizer[0,8]
        # histogram_equalization[True,False]
        # 着色器
        colorizer=rs.colorizer()
        colorizer.set_option(rs.option.color_scheme,color_scheme)
        colorizer.set_option(rs.option.histogram_equalization_enabled,histogram_equalization)
        return colorizer
    
    def saveImg(self,img,num=0,name='',suffix='.jpg',zfill=4,save_dir ='./realsense'):
        path=''.join([save_dir,'/',name,str(num).zfill(zfill),suffix])
        res=cv2.imwrite(path,img)
        return res
        
    def readColorDepth(self, read_dir='./realsense'):
        # 读取文件夹下所有的 color_img,depth_map,depth_img
        color_img=()
        depth_map=()
        depth_img=()
        
        for i in os.listdir(read_dir):
            path=os.path.join(read_dir,i)
            if 'color_img' in i:
                color_img+=(cv2.imread(path),)
            elif 'depth_map' in i:
                depth_map+=(cv2.imread(path,-1),) #cv2.IMREAD_ANYDEPTH=2
            elif 'depth_img' in i:
                depth_img+=(cv2.imread(path),)
        return color_img, depth_map, depth_img  
    
class realsenseManager:
    def __init__(self):
        self.serial_numbers=self.getCamInfo()[1]
        
    def getCamInfo(self):
        # 获取realsense 型号 及 出厂序列号
        ctx = rs.context()
        serial_numbers=()
        names=()
        if len(ctx.devices) > 0:
            for i in ctx.devices:
                serial_number=i.get_info(rs.camera_info.serial_number)
                name=i.get_info(rs.camera_info.name)
                serial_numbers+=(serial_number,)
                names+=(name,)
                print ('Found device: ',
                name, ' ',
                serial_number)
        else:
            print("No Device Connected")
        return names,serial_numbers
    
    def enableDevice(self,serial_number):
        # 可以输入相机的序列号
        # 如果输入的序列号为1,2,3... 则默认为用户指定 第几个 设备
        if type(serial_number)==int and serial_number<10:
            serial_number=self.serial_numbers[serial_number]            
        config = rs.config()
        config.enable_device(serial_number)
        return realsense(config)



if __name__ == '__main__':
    #cam=realsense()
    #cam.start(resolution='720p',depth_preset=3,color_scheme=0,histogram_equalization=True,enable_ir_emitter=1)
    #color_intr_flexiv,depth_intr_flexiv,depth_to_color_extr_flexiv,cam_para_flexiv=cam.getCamPara(robot_name="Flexiv")
    #color_intr_franka,depth_intr_franka,depth_to_color_extr_franka,cam_para_franka=cam.getCamPara(robot_name="Franka")
    #filter_holefilling = cam.filterHoleFilling(1)
    #filter_spatial = cam.filterSpatial(5,0.25,50)
    #filter_decimation = cam.filterDecimation(3)
    '''
    camInfo=GetACamPara()

    while 1:
        key = cv2.waitKey(1) & 0xFF    
        color_img_flexiv, depth_map_flexiv, depth_img_flexiv=camInfo.getImage(robot_name="Flexiv")
        color_img_franka, depth_map_franka, depth_img_franka=camInfo.getImage(robot_name="Franka")
        color_intr_flexiv,depth_intr_flexiv=camInfo.getIntrInfo(robot_name="Flexiv")
        color_intr_franka,depth_intr_franka=camInfo.getIntrInfo(robot_name="Franka")
        print(color_intr_flexiv.fx)
        print(depth_intr_flexiv.fx)
        print(color_intr_franka.fx)
        print(depth_intr_franka.fx)
        color_depth_flexiv=np.hstack((color_img_flexiv, depth_img_flexiv))
        color_depth_franka=np.hstack((color_img_franka, depth_img_franka))
        color_depth=np.vstack((color_depth_flexiv,color_depth_franka))
        cv2.imshow("test", color_depth)
                
        if key== 27: # esc
            break
            
    #cam.videoStream(filter_decimation,filter_spatial,max_shotcut=100,save_dir='./test')
    #color_intr,depth_intr,depth_to_color_extr,cam_para\
    #=cam.getCamPara(save=False,save_dir='./cam_para')
    #cam.stop()
    camInfo.stop()
    '''