import queue
import os
import time
import threading
import signal
import rclpy
import numpy as np
import rclpy.logging
import std_msgs.msg
import multiprocessing
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from .TI import get_data
from .plotheatmap import plotHM
from .benchmark import InstrumentationTimer, Instrumentor

class rospublisher(Node):
    def __init__(self, queue_size=100, max_depth=multiprocessing.cpu_count()):
    
        super().__init__('iwr6843aop')

        ## ros parameters
        self.declare_parameter('cfg_path', os.getcwd() + '/src/ti_ros2_driver/cfg/staticRetention.cfg')
        self.declare_parameter('command_port', '/dev/ttyUSB0')
        self.declare_parameter('data_port', '/dev/ttyUSB1')
        self.declare_parameter('topic', "ti_mmwave_0")
        self.declare_parameter('frame_id', "ti_mmwaver_0")
        self.declare_parameter("roi", "")
        self.declare_parameter("publish_target_tracker", False)
        self.declare_parameter('output_RA_heatmap', False)
        self.declare_parameter('output_RD_heatmap', False)
        self.declare_parameter("debug_mode", False)
        self.declare_parameter("debug_log_path", os.getcwd() + '/src/ti_ros2_driver/debug/result.json')
        self.cfg_path_ = self.get_parameter("cfg_path").get_parameter_value().string_value
        self.command_port_ = self.get_parameter("command_port").get_parameter_value().string_value
        self.data_port_ = self.get_parameter("data_port").get_parameter_value().string_value
        self.topic_ = self.get_parameter("topic").get_parameter_value().string_value
        self.frame_id_ = self.get_parameter("frame_id").get_parameter_value().string_value
        xyz = self.get_parameter("roi").get_parameter_value().string_value
        self.publish_tar = self.get_parameter("publish_target_tracker").get_parameter_value().bool_value
        self.plot_RA_ = self.get_parameter("output_RA_heatmap").get_parameter_value().bool_value
        self.plot_RD_ = self.get_parameter("output_RD_heatmap").get_parameter_value().bool_value
        self.debug_ = self.get_parameter("debug_mode").get_parameter_value().bool_value
        self.debug_log_path_ = self.get_parameter("debug_log_path").get_parameter_value().string_value
        roi = {}
        if xyz:
            chs = [ch.strip() for ch in xyz.split(";")]
            if len(chs)!=3: self.get_logger().warn('Invalid roi bounds setting. No roi will be specified.')
            else:
                for ch_n, ch_v in enumerate(chs):
                    ll, ul = ch_v.split("_")
                    ll, ul = float(ll), float(ul)
                    if ch_n == 0: roi.update({"x_l": ll, "x_u": ul})
                    if ch_n == 1: roi.update({"y_l": ll, "y_u": ul})
                    if ch_n == 2: roi.update({"z_l": ll, "z_u": ul})

        ## ros node and qos
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=max_depth
        )
        self.publisher_ = self.create_publisher(PointCloud2, self.topic_, qos_profile)
        if self.publish_tar: self.target_publisher_ = self.create_publisher(PointCloud2, self.topic_ + "_target", qos_profile)

        ## parse ti raw data in a producer-consumer mode
        self._pcbuffer = queue.Queue(queue_size)
        self.ti = get_data(command_port=self.command_port_, data_port=self.data_port_, cfg_path=self.cfg_path_, roi=roi)
        if self.plot_RA_ or self.plot_RD_: self.heatmap = plotHM(if_ra=self.plot_RA_, if_rd=self.plot_RD_, hang=float(self.ti._ms_per_frame/1000))
            
        self.cv = threading.Condition()
        self.t_datain = threading.Thread(target=self.__producer,args=(self.ti.read(),))
        self.t_dataout = threading.Thread(target=self.__consumer)
        self.shut_down = 0
        if self.debug_: self.enterDebugMode()
        self.t_datain.start()
        self.t_dataout.start()
        
        if self.plot_RA_ or self.plot_RD_: self.heatmap.show()
        
    def __producer(self, g):
        # print("pull threading",threading.current_thread().ident)
        action = g
        empty_times = 0
        while self.shut_down==0:
            time.sleep(float(self.ti._ms_per_frame/4000))
            if empty_times > 20:
                if self.debug_: 
                    producer_timer.stop()
                    self.timer.stop()
                    self.vis.EndSession()
                self.get_logger().error("No data received for more than 20 times, queue released")
                self.shut_down = 1
            if self.debug_: producer_timer = InstrumentationTimer()
            try:
                if not self.ti.cfg_mode:
                    if not self.ti.output_heat_map: res = next(action)
                    else: res, ra_0, ra_1, rd = next(action)
                else: res, res_tar, res_conv = next(action)
                empty_times = 0
            except:
                action = g
                empty_times += 1
            try:
                self.cv.acquire()
                if not self.ti.cfg_mode:
                    if not self.ti.output_heat_map: self._pcbuffer.put_nowait(res)
                    else: self._pcbuffer.put_nowait([res, ra_0, ra_1, rd])
                else: self._pcbuffer.put_nowait([res, res_tar, res_conv])
                # print("after putting, size is ", self._pcbuffer.qsize())
                self.cv.notify()
                self.cv.release()
            except Exception as e:
                self.cv.release()
                # print(e)
            if self.debug_: producer_timer.stop()
        if rclpy.ok(): self.release_ros()

            
    def __consumer(self):
        while self.shut_down==0:
            if self.debug_: consumer_timer = InstrumentationTimer()
            self.cv.acquire()
            self.cv.wait()
            while True:
                try:
                    if not self.ti.cfg_mode:
                        if not self.ti.output_heat_map: ti_pc = self._pcbuffer.get_nowait()
                        else: 
                            ti_data = self._pcbuffer.get_nowait()
                            ti_pc = ti_data[0]
                    else:
                            ti_data = self._pcbuffer.get_nowait()
                            ti_pc = ti_data[0]
                    # print("after getting, size is ", self._pcbuffer.qsize())
                    break
                except Exception as e:
                    self.get_logger().error(e)
            self.cv.release()
            (pcl_msg, num_points) = self.processPointCloud(ti_pc)
            if self.plot_RD_ or self.plot_RA_: self.processHeatMap(ti_data[1], ti_data[2], ti_data[3])
            if self.publish_tar: (tar_msg, num_tars) = self.processTargetTracker(ti_data[1], ti_data[2])
            if rclpy.ok(): 
                self.publisher_.publish(pcl_msg)
                if self.publish_tar: self.target_publisher_.publish(tar_msg)
            else: return
            if self.debug_: 
                self.get_logger().info('Publishing %s points' % num_points)
                if self.publish_tar: self.get_logger().info('Detected %s points' % num_tars)
            pcl_msg.fields.clear()
            time.sleep(float(self.ti._ms_per_frame/1000))
            if self.debug_: consumer_timer.stop()

    def processPointCloud(self, pc):
        pcl_msg = PointCloud2()
        cloud_arr = np.asarray(pc).astype(np.float32)
        pcl_msg.header = std_msgs.msg.Header()
        pcl_msg.header.stamp = self.get_clock().now().to_msg()
        pcl_msg.header.frame_id = self.frame_id_
        pcl_msg.height = 1
        pcl_msg.width = cloud_arr.shape[0]
        pcl_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                            PointField(name='velocity', offset=16, datatype=PointField.FLOAT32, count=1),
                            ]  
        pcl_msg.point_step = cloud_arr.dtype.itemsize * cloud_arr.shape[1]
        pcl_msg.row_step = pcl_msg.point_step * cloud_arr.shape[0]
        pcl_msg.is_dense = True
        pcl_msg.data = cloud_arr.tostring()
        return pcl_msg, cloud_arr.shape[0]

    def processHeatMap(self, RA_Mat_F, RA_Mat_T, RD_Mat):
        assert self.ti.output_heat_map
        if not self.heatmap.is_init:
            # print(self.ti.angle == 8)
            if self.ti.angle == 8: self.heatmap.init_hm(self.ti._rangeFFTSize, self.ti._rangeDopplerSize, self.ti._range_max, self.ti._vel_max, self.ti._range_resolution, self.ti._vel_abs_max, self.ti._numVirtualAnt-4, 400)
            if self.ti.angle == 4: self.heatmap.init_hm(self.ti._rangeFFTSize, self.ti._rangeDopplerSize, self.ti._range_max, self.ti._vel_max, self.ti._range_resolution, self.ti._vel_abs_max, self.ti._numVirtualAnt, 400)
        if self.plot_RD_: RD_Mat = np.reshape(RD_Mat, (self.ti._rangeFFTSize, self.ti._rangeDopplerSize))
        if self.plot_RA_: 
            # https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/1311419/iwr6843isk-ods-iwr6843isk-ods-chirp-order-and-virtual-antenna-configuration
            RA_Mat = np.array([RA_Mat_T[i] + 1j * RA_Mat_F[i] for i in range(self.ti._rangeFFTSize*self.ti._numVirtualAnt)])
            RA_Mat = np.reshape(RA_Mat, (self.ti._rangeFFTSize, self.ti._numVirtualAnt))
            try:
                RA_Mat = np.abs(np.fft.fft(RA_Mat, self.ti._numVirtualAnt))
                if self.ti.angle == 8: RA_Mat = RA_Mat[:,4:]  # Range-Azimuth FFT
                if self.ti.angle == 4: RA_Mat = RA_Mat[:,:-4] # Range-Elevation FFT
            except Exception as e:
                self.get_logger().error("Fail to perform fft for Range-Amizuth complex dtype\n" + e)
        self.heatmap.get(RA_Mat, RD_Mat)

    def processTargetTracker(self, target, conv):
        tar_msg = PointCloud2()
        cloud_tar = np.asarray(target).astype(np.float32)
        tar_msg.header = std_msgs.msg.Header()
        tar_msg.header.stamp = self.get_clock().now().to_msg()
        tar_msg.header.frame_id = self.frame_id_
        tar_msg.height = 1
        tar_msg.width = cloud_tar.shape[0]
        tar_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                            PointField(name='velocity_x', offset=12, datatype=PointField.FLOAT32, count=1),
                            PointField(name='velocity_y', offset=16, datatype=PointField.FLOAT32, count=1),
                            PointField(name='velocity_z', offset=20, datatype=PointField.FLOAT32, count=1),
                            PointField(name='acceleration_x', offset=24, datatype=PointField.FLOAT32, count=1),
                            PointField(name='acceleration_y', offset=28, datatype=PointField.FLOAT32, count=1),
                            PointField(name='acceleration_z', offset=32, datatype=PointField.FLOAT32, count=1),
                            ]  
        tar_msg.point_step = cloud_tar.dtype.itemsize * cloud_tar.shape[1]
        tar_msg.row_step = tar_msg.point_step * cloud_tar.shape[0]
        tar_msg.is_dense = True
        tar_msg.data = cloud_tar.tostring()
        return tar_msg, cloud_tar.shape[0]

    def enterDebugMode(self):
        self.vis = Instrumentor()
        self.vis.BeginSession(self.debug_log_path_)
        self.timer = InstrumentationTimer()

    def release_ros(self):
        print("Clearing ros node")
        with self._pcbuffer.mutex:
            self._pcbuffer.queue.clear()
        self.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        self.ti.close()
        print("Exiting")

    def ctrlc_handler(self, signum, frame):
        if self.plot_RA_ or self.plot_RD_: self.heatmap.close()
        if self.debug_:self.timer.stop()
        if self.debug_:self.vis.EndSession()
        self.shut_down = 1
        self.t_datain.join()
        self.t_dataout.join()
        self.release_ros()

def main(): 
    rclpy.init()
    minimal_publisher = rospublisher()
    signal.signal(signal.SIGINT, minimal_publisher.ctrlc_handler)

if __name__ == '__main__':
    main()