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
        global cfg_path
        global command_port
        global data_port
        global topic
        global frame_id
        self.declare_parameter('cfg_path', cfg_path)      
        self.declare_parameter('command_port', command_port)
        self.declare_parameter('data_port', data_port)
        self.declare_parameter('topic', topic)     
        self.declare_parameter('frame_id', frame_id)        
        cfg_path = self.get_parameter('cfg_path').get_parameter_value().string_value
        command_port = self.get_parameter('command_port').get_parameter_value().string_value
        data_port = self.get_parameter('data_port').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=max_depth
        )
        self.publisher_ = self.create_publisher(PointCloud2, topic, qos_profile)
        self.ti = get_data(command_port=command_port, data_port=data_port, cfg_path=cfg_path)
        self.timer = self.create_timer(float(self.ti._ms_per_frame/1000), self._timer_callback)
        
        self._pcbuffer = queue.Queue(queue_size)
        self._t = threading.Thread(target=self._producer,args=(self.ti._read(),))
        self._t.start()
        
    def _producer(self, g):
        # print("pull threading",threading.current_thread().ident)
        action = g
        while rclpy.ok(context=self._context):
            self.producer_timer = InstrumentationTimer()
            try:
                res = next(action)
                time.sleep(float(self.ti._ms_per_frame/2000))
            except Exception as exception:
                print(exception)
                action = g
            try:
                self._pcbuffer.put_nowait(res)
                self.producer_timer.stop()
                # print("after putting",self._pcbuffer.qsize())
            except queue.Full:
                print("Fail to put data into queue because it's full already")
                pass
            global shut_down
            if shut_down == 1:
                self.release()
                return
            
    def _consumer(self):
        self.consumer_timer = InstrumentationTimer()
        empty_times = 0
        while rclpy.ok(context=self._context):
            if(self._pcbuffer.empty()):
                empty_times += 1
                time.sleep(float(self.ti._ms_per_frame/4000))
                continue
            else:
                empty_times = 0
            if(empty_times>200):
                self.release()
                print("not receiving data for 10s, relase queue")
                return
            global shut_down
            if shut_down == 1:
                self.release()
                return
            try:
                # print("after getting", self._pcbuffer.qsize())
                output = self._pcbuffer.get_nowait()
                self.consumer_timer.stop()
                return output
            except Exception as exception:
                print(exception)
                continue
            
    def _timer_callback(self):
        if rclpy.ok(context=self._context):
            self.spin_timer = InstrumentationTimer()
            cloud_arr = np.asarray(self._consumer()).astype(np.float32)
            pcl_msg = PointCloud2()
            pcl_msg.header = std_msgs.msg.Header()
            pcl_msg.header.stamp = self.get_clock().now().to_msg()
            pcl_msg.header.frame_id = frame_id
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
            self.publisher_.publish(pcl_msg)
            self.get_logger().info('Publishing %s points' % cloud_arr.shape[0] )
            self.spin_timer.stop()
    
    def release(self):
        self.producer_timer.stop()
        self.consumer_timer.stop()
        self.spin_timer.stop()
        self.ti.close()
        with self._pcbuffer.mutex:
            self._pcbuffer.queue.clear()

def main():
    global shut_down
    shut_down = 0
    global data_port
    data_port = '/dev/ttyUSB1'
    global command_port
    command_port = '/dev/ttyUSB0'
    global cfg_path
    cfg_path = os.getcwd() + '/src/TI_IWR6843AOP_ROS2/ti_test_py/cfg/staticRetention.cfg'
    global topic
    topic = "ti_mmwave_0"
    global frame_id
    frame_id = "ti_mmwaver_0"

    signal.signal(signal.SIGINT, ctrlc_handler)
    global vis
    vis = Instrumentor()
    vis.BeginSession()
    global timer
    timer = InstrumentationTimer()
    rclpy.init()
    minimal_publisher = rospublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher._t.join()
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    timer.stop()
    vis.EndSession()

def ctrlc_handler(signum, frame):
    shut_down = 1
    time.sleep(0.25)
    print("Exiting")
    timer.stop()
    vis.EndSession()
    exit(1)

if __name__ == '__main__':
    main()