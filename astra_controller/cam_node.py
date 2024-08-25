import threading
import time
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.action

import sensor_msgs.msg

import cv2

def feed(
    node, 
    pub, 
    device, 
    image_width, image_height, framerate, 
    resize_image_width, resize_image_height
):
    cam = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 10)

    fourcc_value = cv2.VideoWriter_fourcc(*'MJPG')

    cam.set(cv2.CAP_PROP_FOURCC, fourcc_value)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
    cam.set(cv2.CAP_PROP_FPS, framerate)
    
    assert cam.isOpened()
    
    start_time = time.perf_counter()
    
    while True:
        ret, image = cam.read()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
        msg = sensor_msgs.msg.Image()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "default_cam" # odom is fixed to map, cam_link is reletive to odom
        
        if resize_image_width:
            image = cv2.resize(image, (resize_image_width, resize_image_height))
            msg.width = resize_image_width
            msg.height = resize_image_height
        else:
            msg.width = image_width
            msg.height = image_height
        
        msg.encoding = "rgb8"
        msg.data.frombytes(image.tobytes())

        pub.publish(msg)
        
        print(1/ (time.perf_counter() - start_time))
        start_time = time.perf_counter()

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node('cam_node')

    node.declare_parameter('device', '/dev/video_head')
    node.declare_parameter('image_width', 640)
    node.declare_parameter('image_height', 360)
    node.declare_parameter('framerate', 30.0)
    node.declare_parameter('resize_image_width', 0)
    node.declare_parameter('resize_image_height', 0)

    device = node.get_parameter('device').value
    image_width = node.get_parameter('image_width').value
    image_height = node.get_parameter('image_height').value
    framerate = node.get_parameter('framerate').value
    resize_image_width = node.get_parameter('resize_image_width').value
    resize_image_height = node.get_parameter('resize_image_height').value

    pub = node.create_publisher(sensor_msgs.msg.Image, "image_raw", 10)

    threading.Thread(target=feed, args=(
        node, 
        pub, 
        device, 
        image_width, image_height, framerate, 
        resize_image_width, resize_image_height
    ), daemon=True).start()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt as err:
        raise err

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
