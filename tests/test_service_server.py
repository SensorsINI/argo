#!/usr/bin/env python3
"""
ROS2 Service Server Test
Responds to Trigger service calls and returns a success response.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import logging
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('test_service_server')

class TestServiceServer(Node):
    def __init__(self):
        super().__init__('test_service_server')
        
        # Create service
        self.srv = self.create_service(
            Trigger,
            'test_trigger_service',
            self.trigger_callback
        )
        
        logger.info('Test service server started, waiting for requests...')
        logger.info(f'Service name: test_trigger_service')
        logger.info(f'Node name: {self.get_name()}')
    
    def trigger_callback(self, request, response):
        """Handle trigger service requests"""
        logger.info('Received trigger request')
        
        # Set success response
        response.success = True
        response.message = 'Test service server responded successfully!'
        
        logger.info(f'Responding with success: {response.success}, message: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        server = TestServiceServer()
        
        logger.info('Service server is running. Press Ctrl+C to stop.')
        rclpy.spin(server)
        
    except KeyboardInterrupt:
        logger.info('Received keyboard interrupt, shutting down...')
    except Exception as e:
        logger.error(f'Error in service server: {e}')
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        logger.info('Service server shutdown complete')

if __name__ == '__main__':
    main()
