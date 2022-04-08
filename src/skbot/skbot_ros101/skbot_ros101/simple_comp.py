import sys

from .simple_pub import Talker
from .simple_sub import Listener
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    rclpy.init(args=args)
    try:
        talker = Talker()
        listener = Listener()

        # Runs all callbacks in the main thread
        executor = SingleThreadedExecutor()
        # Add imported nodes to this executor
        executor.add_node(talker)
        executor.add_node(listener)

        try:
            # Execute callbacks for both nodes as they become ready
            executor.spin()
        finally:
            executor.shutdown()
            listener.destroy_node()
            talker.destroy_node()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()