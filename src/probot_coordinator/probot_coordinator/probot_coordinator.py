import os
import rclpy
import rclpy.logging
from rclpy.node import Node
from .probot_coordinator_llm import ProbotLLMCoordinator

def main(args=None):
    main_logger = rclpy.logging.get_logger(__name__)
    
    rclpy.init(args=args)
    
    probot_coordinator = ProbotLLMCoordinator()
    
    try:
        rclpy.spin(probot_coordinator)
    except KeyboardInterrupt:
        main_logger.info('Terminating coordinator')
    except BaseException as e:
        main_logger.error(f'Exception in coordinator: {e}')
    finally:
        probot_coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()