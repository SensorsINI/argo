#!/usr/bin/env python3
"""
Safe Publishing Utilities for ROS2 Nodes

This module provides utilities to prevent "publisher's context is invalid" errors
by checking ROS2 context validity before publishing messages.
"""

import rclpy
from typing import Any, Optional


def safe_publish(publisher, message: Any, node=None) -> bool:
    """
    Safely publish a message, checking ROS2 context validity first.
    
    Args:
        publisher: ROS2 publisher object
        message: Message to publish
        node: Optional node object for additional context checks
        
    Returns:
        bool: True if message was published successfully, False otherwise
    """
    try:
        # Check if ROS2 context is still valid
        if not rclpy.ok():
            return False
            
        # Check node context if provided
        if node is not None and hasattr(node, 'context') and not node.context.ok():
            return False
            
        # Check if publisher is still valid
        if not hasattr(publisher, 'publish') or publisher is None:
            return False
            
        # Publish the message
        publisher.publish(message)
        return True
        
    except Exception:
        # Silently fail - this is expected during shutdown
        return False


def safe_log(node, level: str, message: str) -> bool:
    """
    Safely log a message, checking ROS2 context validity first.
    
    Args:
        node: ROS2 node object
        level: Log level ('info', 'warn', 'error', 'debug')
        message: Message to log
        
    Returns:
        bool: True if message was logged successfully, False otherwise
    """
    try:
        # Check if ROS2 context is still valid
        if not rclpy.ok():
            return False
            
        # Check node context
        if not hasattr(node, 'context') or not node.context.ok():
            return False
            
        # Check if logger is still valid
        if not hasattr(node, 'get_logger'):
            return False
            
        # Log the message
        logger = node.get_logger()
        if level == 'info':
            logger.info(message)
        elif level == 'warn':
            logger.warn(message)
        elif level == 'error':
            logger.error(message)
        elif level == 'debug':
            logger.debug(message)
        else:
            logger.info(message)  # Default to info
            
        return True
        
    except Exception:
        # Silently fail - this is expected during shutdown
        return False


def is_context_valid(node=None) -> bool:
    """
    Check if ROS2 context is still valid.
    
    Args:
        node: Optional node object for additional context checks
        
    Returns:
        bool: True if context is valid, False otherwise
    """
    try:
        # Check global ROS2 context
        if not rclpy.ok():
            return False
            
        # Check node context if provided
        if node is not None and hasattr(node, 'context') and not node.context.ok():
            return False
            
        return True
        
    except Exception:
        return False
