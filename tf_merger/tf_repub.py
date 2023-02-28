import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.executors import SingleThreadedExecutor

class TfRepublisher(Node):

    def __init__(self, namespace, agent):
        super().__init__('tf_republisher_'+str(agent))
        self.namespace=namespace
        self.agent=agent
        tf_topic=str(self.namespace)+'_'+str(self.agent)+'/tf'
        self.subscription = self.create_subscription(
            TFMessage,
            tf_topic,
            self.tf_callback,
            10
        )
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )

    def tf_callback(self, msg):
        tf_prefix=str(self.namespace)+'_'+str(self.agent)+'/'
        for transform in msg.transforms:
            if transform.header.frame_id=='odom' and transform.child_frame_id=='base_link':
                # Update the frame IDs to remove the barista_1 prefix
                transform.header.frame_id = transform.header.frame_id.replace('odom', tf_prefix+'odom')
                transform.child_frame_id = transform.child_frame_id.replace('base_link', tf_prefix+'base_link')
            else:
                # Update the frame IDs to remove the barista_1 prefix
                transform.header.frame_id = transform.header.frame_id.replace('odom', tf_prefix+'odom')
                transform.child_frame_id = transform.child_frame_id.replace('base_footprint', tf_prefix+'base_footprint')
                           
        # Publish the transformed message on the '/tf' topic
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    agents=4
    tf_repubs=[]
    try:  
        for i in range(agents):   
            tf_repub=TfRepublisher(namespace='barista', agent=i)
            tf_repubs.append(tf_repub)
        for tf_repub in tf_repubs:
            executor.add_node(tf_repub)
        try:
            executor.spin()
        finally:
            executor.shutdown()
        
    except KeyboardInterrupt:
        executor.shutdown()


if __name__ == '__main__':
    main()