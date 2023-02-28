import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, TransformBroadcaster
from tf2_msgs.msg import TFMessage

class TFmerger(Node):

    def __init__(self):
        super().__init__('tf_combiner')
        self.tf_listener = TransformListener(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self.timer_callback) # set the rate to 10 Hz

    def timer_callback(self):
        try:
            # Get the list of transforms for the given namespace
            transforms = self.tf_listener.getFrameStrings()
            namespace = "/barista_0"
            transforms = [t for t in transforms if t.startswith(namespace)]
            # Combine the transforms into a single message
            combined_msg = TFMessage()
            for transform in transforms:
                # Get the transform data for the given transform
                tf_data = self.tf_listener.lookup_transform("world", transform, rclpy.time.Time())
                # Update the frame IDs to include the namespace
                tf_data.header.frame_id = namespace + tf_data.header.frame_id
                tf_data.child_frame_id = namespace + tf_data.child_frame_id
                # Add the transform data to the combined message
                combined_msg.transforms.append(tf_data)
            # Broadcast the combined message on the '/tf' topic
            self.tf_broadcaster.sendTransform(combined_msg.transforms)
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    tf_combiner = TFmerger()
    rclpy.spin(tf_combiner)
    tf_combiner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()