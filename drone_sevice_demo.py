import socket
import threading
import time

import rclpy
from rclpy.node import Node

from drone_interfaces.srv import Move
from drone_interfaces.srv import Output

from std_srvs.srv import Empty

drone_response = "no_response"
timer = time.time()

class DroneServer(Node):

    def __init__(self):
        super().__init__('drone_server')

        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
        self.socket.bind((self.local_ip, self.local_port))

        # thread for receiving cmd ack
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        self.tello_ip = '192.168.10.1'
        self.tello_port = 8889
        self.tello_address = (self.tello_ip, self.tello_port)
        self.MAX_TIME_OUT = 15.0
        
        timer_period = 12 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.srv = self.create_service(
            Move, 'move_forward', self.move_forward_callback)

        self.srv = self.create_service(
            Move, 'move_backward', self.move_backward_callback)

        self.srv = self.create_service(
            Move, 'move_left', self.move_left_callback)

        self.srv = self.create_service(
            Move, 'move_right', self.move_right_callback)

        self.srv = self.create_service(
            Empty, 'flip_forward', self.flip_forward_callback)
        self.srv = self.create_service(
            Empty, 'flip_backward', self.flip_backward_callback)
        self.srv = self.create_service(
            Empty, 'flip_left', self.flip_left_callback)
        self.srv = self.create_service(
            Empty, 'flip_right', self.flip_right_callback)

        self.srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        self.srv = self.create_service(Empty, 'land', self.land_callback)

        self.srv = self.create_service(Empty, 'command', self.command_callback)
        self.srv = self.create_service(Output, 'battery', self.battery_callback)

    def timer_callback(self):
        global timer

        now = time.time()

        if (now - timer > 5.0):
            print("\n\nWarning No input for 15 seconds!!! Automatic Call Initiated\n\n")
            self.command_callback()
        
        print("15 second automatic callback")
        
        battery = self.status_check()
            
        try:
            if int(battery) < 20:
                print("\n\nWarning Battery Below 20!!! Automatic Landing Started\n\n")
                self.land_callback(self,response="",request="")
        except:
            print("\n\nWarning | Battery can't be read | Landing Drone\n\n")
        
        print(battery)
        timer = time.time()
        
    def send_command(self, msg):
        global timer
        command = msg  # the actual command string

        self.get_logger().info('I heard: "%s"' % msg)
        self.socket.sendto(command.encode('utf-8'), self.tello_address)
        print('sending command: %s to %s' % (command, self.tello_ip))

        start = time.time()
        now = time.time()
        diff = now - start
        if diff > self.MAX_TIME_OUT:
            print('Max timeout exceeded... command %s' % command)
            return
        print('Done!!! sent command: %s to %s' % (command, self.tello_ip))
        
        timer = time.time()

    def command_callback(self):
        self.get_logger().info('Incoming request: Command')
        command = "command"
        print(command)
        self.send_command(command)

    def move_forward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Forward: %dcm' % (request.distance))
        command = "forward %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)  # wait for the response
        response.result = drone_response
        return response

    def move_backward_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move Backward: %dcm' % (request.distance))
        command = "back %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)  # wait for the response
        response.result = drone_response
        return response

    def move_left_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move left: %dcm' % (request.distance))
        command = "left %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)  # wait for the response
        response.result = drone_response
        return response

    def move_right_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Move right: %dcm' % (request.distance))
        command = "right %d" % request.distance
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)  # wait for the response
        response.result = drone_response
        return response

    def flip_forward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip forward')
        command = "flip f"
        print(command)
        self.send_command(command)
        return response

    def flip_backward_callback(self, request, response):
        self.get_logger().info('Incoming request: flip backward')
        command = "flip b"
        print(command)
        self.send_command(command)
        return response

    def flip_left_callback(self, request, response):
        self.get_logger().info('Incoming request: flip left')
        command = "flip l"
        print(command)
        self.send_command(command)
        return response

    def flip_right_callback(self, request, response):
        self.get_logger().info('Incoming request: flip right')
        command = "flip r"
        print(command)
        self.send_command(command)
        return response

    def takeoff_callback(self, request, response):
        self.get_logger().info('Incoming request: Takeoff')
        command = "takeoff"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Incoming request: Land')
        command = "land"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response

    def battery_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: battery?')
        command = "battery?"
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)  # wait for the response
        response.result = drone_response
        print(drone_response)
        return response
    
    def status_check(self):
        global drone_response
        self.get_logger().info('Incoming request: battery?')
        command = "battery?"
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)  # wait for the response
        response = drone_response
        # print(drone_response)
        return response

    def _receive_thread(self):
        global drone_response
        # Listen to responses from the Tello.
        while True:
            try:
                self.response, ip = self.socket.recvfrom(1024)
                print('from %s: %s' % (ip, self.response))
                # convert from byte string to string
                drone_response = str(self.response,'utf-8')
            except (socket.error, exc):
                print("Caught exception socket.error : %s" % exc)


def main(args=None):
    rclpy.init(args=args)

    node = DroneServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
