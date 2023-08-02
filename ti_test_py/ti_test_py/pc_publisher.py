import queue
import time
import threading
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from TI import get_data

class rospublisher(Node):
    def __init__(self):
        super.__init__(self)
        global cfg_path
        global data_port
        global command_port
        global topic
        global frame_id
        # self.declare_parameter('data_port', data_port)
        # self.declare_parameter('command_port', command_port)
        # self.declare_parameter('cfg_path', cfg_path)      
        # self.declare_parameter('topic', topic)     
        # self.declare_parameter('frame_id', frame_id)      
        data_port = self.get_parameter('data_port').get_parameter_value().string_value
        command_port = self.get_parameter('command_port').get_parameter_value().string_value
        cfg_path = self.get_parameter('cfg_path').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self._pcbuffer = queue.Queue()

    def _pull_to_queue(self):
        ti = get_data(command_port=command_port, data_port=data_port, cfg_path=cfg_path)
        g = ti._read()
        while 1:
            try:
                res = next(g)
                time.sleep(ti._ms_per_frame/2000)
            except Exception as exception:
                print(exception)
                g = ti._read()
            try:
                self._pcbuffer.put_nowait(res)
            except Exception as exception:
                print(exception, "\nFail to put data into queue.")
            global shut_down
            if shut_down == 1:
                return
            
    def _read_from_queue(self, res):
        self._pcbuffer.get_nowait(res)

def ctrlc_handler(signum, frame):
    global shut_down
    shut_down = 1
    time.sleep(0.25)
    print("Exiting")
    exit(1)     

def main():
    signal.signal(signal.SIGINT, ctrlc_handler)
    

if __name__ == '__main__':
    main()