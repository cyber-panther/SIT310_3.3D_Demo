import rclpy
from rclpy.node import Node

from drone_interfaces.srv import Move
from drone_interfaces.srv import Output
from std_srvs.srv import Empty

from pynput import keyboard
import time

timer = time.time()


class KeyboardController(Node):

    def on_release(self, key):

        if key == keyboard.Key.space:
            cli = self.node.create_client(Empty, 'takeoff')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
                print(result)
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("takeoff successful")

        if key == keyboard.Key.up:
            cli = self.node.create_client(Move, 'move_forward')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("move forward successful")

        if key == keyboard.Key.down:
            cli = self.node.create_client(Move, 'move_backward')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("move backward successful")

        if key == keyboard.Key.left:
            cli = self.node.create_client(Move, 'move_left')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("move left successful")

        if key == keyboard.Key.right:
            cli = self.node.create_client(Move, 'move_right')
            req = Move.Request()
            req.distance = 50
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("move right successful")

        if key == keyboard.Key.f1:
            cli = self.node.create_client(Empty, 'flip_forward')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("flip_forward successful")
        
        if key == keyboard.Key.f1:
            cli = self.node.create_client(Empty, 'flip_forward')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("flip_forward successful")
                
        if key == keyboard.Key.f2:
            cli = self.node.create_client(Empty, 'flip_backward')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("flip_backward successful")
                
        if key == keyboard.Key.f3:
            cli = self.node.create_client(Empty, 'flip_left')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("flip_left successful")
                
        if key == keyboard.Key.f4:
            cli = self.node.create_client(Empty, 'flip_right')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("flip_right successful")
                
        if key == keyboard.Key.f5:
            cli = self.node.create_client(Output, 'battery')
            req = Output.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
                print(result)
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("battery successful")

        if key == keyboard.Key.backspace:
            cli = self.node.create_client(Empty, 'land')
            req = Empty.Request()
            while not cli.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('service not available, waiting again...')

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, future)

            try:
                result = future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
            else:
                self.node.get_logger().info("land successful")

        if key == keyboard.Key.esc:
            return False

    def __init__(self):
        super().__init__('KeyboardController')

        self.node = rclpy.create_node('drone_service_client')
        
        print("ok")

        timer_period = 2 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Collect events until released
        with keyboard.Listener(
                on_release=self.on_release) as listener:
            listener.join()
        listener.start()

    def timer_callback(self):
        print("hello")
        global timer

        now = time.time()

        if (now - time > 15.0):
            self.command()
            
        print(timer, now)
        timer = time.time()
        
    def command(self):
        cli = self.node.create_client(Empty, 'command')
        req = Empty.Request()
        while not cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().info('Service call failed %r' % (e,))
        else:
            self.node.get_logger().info("command successful")
            
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        print("hello")

def main(args=None):
    rclpy.init(args=args)

    controller = KeyboardController()
    
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
