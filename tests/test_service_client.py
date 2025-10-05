#!/usr/bin/env python3
"""
ROS2 Service Client Test
Calls the test_trigger_service and waits for a response.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import logging
import sys
import time

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('test_service_client')

class TestServiceClient(Node):
    def __init__(self):
        super().__init__('test_service_client')
        
        # Create service client
        self.cli = self.create_client(Trigger, 'test_trigger_service')
        
        # Wait for service to be available
        logger.info('Waiting for test_trigger_service to be available...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            logger.info('Service not available, waiting...')
    
    def call_trigger_service(self):
        """Call the trigger service and return the response"""
        request = Trigger.Request()
        
        logger.info('Calling test_trigger_service...')
        
        try:
            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            response = future.result()
            
            if response is not None:
                logger.info(f'Service call successful!')
                logger.info(f'Success: {response.success}')
                logger.info(f'Message: {response.message}')
                return response
            else:
                logger.error('Service call failed - no response received')
                return None
                
        except Exception as e:
            logger.error(f'Error calling service: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    
    try:
        client = TestServiceClient()
        
        # Call the service
        response = client.call_trigger_service()
        
        if response and response.success:
            logger.info('Test completed successfully!')
            sys.exit(0)
        else:
            logger.error('Test failed!')
            sys.exit(1)
            
    except KeyboardInterrupt:
        logger.info('Received keyboard interrupt, shutting down...')
    except Exception as e:
        logger.error(f'Error in service client: {e}')
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        logger.info('Service client shutdown complete')

if __name__ == '__main__':
    main()
