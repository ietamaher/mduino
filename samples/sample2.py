#!/usr/bin/python

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time
import rclpy
from rclpy.node import Node
from mduino.msg import Query
from mduino.msg import Response
from mduino.msg import State
from rclpy.executors import MultiThreadedExecutor

# Global variables
_state_driver = 0  # Communication flag variable (0: communicable, 1: communicating)
msg = Query()

 


class MySubscription(Node):
    def __init__(self):
        super().__init__("my_sub")
        self.sub1 = self.create_subscription(
            Response, "mduino_response", self.response_callback, 1
        )
        self.sub2 = self.create_subscription(
            State, "mduino_state", self.state_callback, 1
        )

    def response_callback(self, res):
        """Response callback function

        Reflects the subscribed response data in global variables

        """
        if res.slave_id == 1 and res.func_code == 3:
            # Update the value when the machine number is 1 and it is a read operation
            motor_pos = res.data[0]
            print("position = {0:}[step]".format(motor_pos))  # Display the read value

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
            self.read_data()
            self.seq = 2
        elif self.seq == 2:
            print("END")
            self.seq = 3
        else:
            return

    def read_data(self):
        global msg
        # Read (position of operation data No. 0)
        msg.slave_id = 0x01  # Machine selection (Hex): Machine 1
        msg.func_code = 2  # Function code selection: 0 (Read)
        msg.read_addr = 204  # Select starting address (Dec): Position of operation data No. 0
        msg.read_num = 10  # Read data size: 1 (32bit)
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

    Processing 1: Read the position of operation data No. 0

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
