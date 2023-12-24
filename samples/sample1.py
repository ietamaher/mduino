#!/usr/bin/python

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time
import rclpy
from rclpy.node import Node
from mduino.msg import Query
from mduino.msg import State
from rclpy.executors import MultiThreadedExecutor

# Global variables
_state_driver = 0  # Communication flag variable (0: communicable, 1: communicating)
msg = Query()

 


class MySubscription(Node):
    def __init__(self):
        super().__init__("my_sub")
        self.sub = self.create_subscription(
            State, "mduino_state", self.state_callback, 1
        )

    def state_callback(self, res):
        """Status callback function

        Reflects the subscribed status data in global variables

        """
        global _state_driver
        _state_driver = res.state_driver


class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_pub")
        self.seq = 0
        self.pub = self.create_publisher(Query, "mduino_query", 1)
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        if _state_driver == 1:
            return

        if self.seq == 0:
            print("START")
            self.seq = 1
        elif self.seq == 1:
            self.write_data()
            self.seq = 2
        elif self.seq == 2:
            print("END")
            self.seq = 3
        else:
            return

    def write_data(self):
        global msg
        # Write (position of operation data No. 0)
        msg.slave_id = 0x01  # Machine selection (Hex): Machine 1
        msg.func_code = 1  # Function code selection: 1 (Write)
        msg.write_addr = 6146  # Select starting address (Dec): Position of operation data No. 0
        msg.write_num = 2  # Write data size: 1 (32bit)
        msg.data[0] = 1000  # Position [step]
        msg.data[1] = 5000  # Position [step]

        self.pub.publish(msg)  # Send the above content to the query generation node. After creating msg in the node, send it to the driver
        self.wait()  # Wait for processing

    def wait(self):
        """Waiting service function

        Waits until communication becomes possible after the specified time (30ms)

        """
        time.sleep(0.03)  # Set wait time (1 = 1.00s)
        # Loop until communication is finished
        while _state_driver == 1:
            pass


def main(args=None):
    """Main function

    Processing 1: Write the position of operation data No. 0

    """
    rclpy.init(args=args)
    try:
        pub = MyPublisher()
        sub = MySubscription()
        executor = MultiThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub.destroy_node()
        sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
