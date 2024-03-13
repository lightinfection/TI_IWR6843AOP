import queue
import os
import time
import threading
import signal
import rclpy
import numpy as np
import std_msgs.msg
import multiprocessing
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from .TI import get_data
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
        self.declare_parameter("debug_mode", False)
        self.cfg_path_ = self.get_parameter("cfg_path").get_parameter_value().string_value
        self.command_port_ = self.get_parameter("command_port").get_parameter_value().string_value
        self.data_port_ = self.get_parameter("data_port").get_parameter_value().string_value
        self.topic_ = self.get_parameter("topic").get_parameter_value().string_value
        self.frame_id_ = self.get_parameter("frame_id").get_parameter_value().string_value
        self.debug_ = self.get_parameter("debug_mode").get_parameter_value().bool_value
        print(self.debug_)
        ## ros node
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=max_depth
        )
        self.publisher_ = self.create_publisher(PointCloud2, self.topic_, qos_profile)

        ## parse ti raw data in a producer-consumer mode
        self._pcbuffer = queue.Queue(queue_size)
        self.ti = get_data(command_port=self.command_port_, data_port=self.data_port_, cfg_path=self.cfg_path_)

        self.cv = threading.Condition()
        t_datain = threading.Thread(target=self.__producer,args=(self.ti.read(),))
        t_dataout = threading.Thread(target=self.__consumer)
        t_datain.start()
        t_dataout.start()

        t_datain.join()
        t_dataout.join()
        self.destroy_node()
        rclpy.shutdown()
        
    def __producer(self, g):
        # print("pull threading",threading.current_thread().ident)
        action = g
        while rclpy.ok(context=self._context):
            time.sleep(float(self.ti._ms_per_frame/4000))
            if(self.debug_): self.producer_timer = InstrumentationTimer()
            try:
                res = next(action)
            except Exception as exception:
                print(exception)
                action = g
            try:
                self.cv.acquire()
                self.cv.notify()
                self._pcbuffer.put_nowait(res)
                self.cv.release()
                # print("after putting",self._pcbuffer.qsize())
            except queue.Full:
                print("Fail to put data into queue because it's full already")
                pass
            if(self.debug_): self.producer_timer.stop()
            global shut_down
            if shut_down == 1:
                self._release()
                return
            
    def __consumer(self):
        pcl_msg = PointCloud2()
        empty_times = 0
        while rclpy.ok(context=self._context):
            if(self.debug_): self.consumer_timer = InstrumentationTimer()
            self.cv.acquire()
            while True:
                if(not self._pcbuffer.empty()):
                    try:
                        # print("after getting", self._pcbuffer.qsize())
                        output = self._pcbuffer.get_nowait()
                        cloud_arr = np.asarray(output).astype(np.float32)
                        pcl_msg.header = std_msgs.msg.Header()
                        pcl_msg.header.stamp = self.get_clock().now().to_msg()
                        pcl_msg.header.frame_id = self.frame_id_
                        pcl_msg.height = 1
                        pcl_msg.width = cloud_arr.shape[0]
                        pcl_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                                            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                                            ]  
                        pcl_msg.point_step = cloud_arr.dtype.itemsize*cloud_arr.shape[1]
                        pcl_msg.row_step = pcl_msg.point_step*cloud_arr.shape[0]
                        pcl_msg.is_dense = True
                        pcl_msg.data = cloud_arr.tostring()
                        time.sleep(float(self.ti._ms_per_frame/1000))
                        self.publisher_.publish(pcl_msg)
                        self.get_logger().info('Publishing %s points' % cloud_arr.shape[0])
                        empty_times = 0
                        break
                    except Exception as exception:
                        print(exception)
                        continue
                    
                empty_times += 1
                if(empty_times>20):
                    self._release()
                    print("not receiving data, relase queue")
                    return
                self.publisher_.publish(pcl_msg)
                pcl_msg.fields.clear()
                self.cv.wait()
            self.cv.release()
            if(self.debug_): self.consumer_timer.stop()
            global shut_down
            if shut_down == 1:
                self._release()
                return
    
    def _release(self):
        if(self.debug_): self.producer_timer.stop()
        if(self.debug_): self.consumer_timer.stop()
        self.destroy_node()
        rclpy.shutdown()
        self.ti.close()
        with self._pcbuffer.mutex:
            self._pcbuffer.queue.clear()

def main():
    global shut_down, vis, timer
    shut_down = 0
    signal.signal(signal.SIGINT, ctrlc_handler)
    
    rclpy.init()
    vis = Instrumentor()
    vis.BeginSession()
    timer = InstrumentationTimer()
    minimal_publisher = rospublisher()
    timer.stop()
    vis.EndSession()

def ctrlc_handler(signum, frame):
    global shut_down
    shut_down = 1
    timer.stop()
    vis.EndSession()
    time.sleep(0.25)
    print("Exiting")
    exit(1)

if __name__ == '__main__':
    main()