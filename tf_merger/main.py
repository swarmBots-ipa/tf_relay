import rclpy
from tf_merger.tf_relay import TFRelay
from tf_merger.tf_static_relay import TFStaticRelay
from rclpy.executors import SingleThreadedExecutor

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    agents = 4
    tf_relays = []
    tf_static_relays = []

    try:
        for i in range(agents):
            tf_relay = TFRelay(namespace='agent', agent=i)
            tf_static_relay = TFStaticRelay(namespace='agent', agent=i)
            tf_relays.append(tf_relay)
            tf_static_relays.append(tf_static_relay)

        for tf_relay in tf_relays:
            executor.add_node(tf_relay)
        
        for tf_static_relay in tf_static_relays:
            executor.add_node(tf_static_relay)

        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        executor.shutdown()


if __name__ == '__main__':
    main()
