
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
