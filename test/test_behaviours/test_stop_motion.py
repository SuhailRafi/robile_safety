# test/test_behaviours/test_stop_motion.py - UPDATED VERSION

import pytest
from unittest.mock import Mock
import py_trees as pt
from geometry_msgs.msg import Twist

from robile_safety.safety_bt import StopMotion

class TestStopMotionBehaviour:
    """Test suite for StopMotion behaviour"""
    
    def test_stop_motion_initialization(self):
        """Test StopMotion behaviour initialization"""
        stop = StopMotion(topic_name="/emergency_stop")
        assert stop.topic_name == "/emergency_stop"
        assert stop.name == "stop motion"
        
        # Test default topic name
        stop_default = StopMotion()
        assert stop_default.topic_name == "/cmd_vel"
    
    def test_stop_motion_setup(self):
        """Test StopMotion behaviour setup"""
        stop = StopMotion()
        
        # Mock node
        mock_node = Mock()
        mock_publisher = Mock()
        mock_node.create_publisher.return_value = mock_publisher
        
        # Call setup
        result = stop.setup(node=mock_node)
        
        assert result is True
        mock_node.create_publisher.assert_called_once_with(Twist, "/cmd_vel", 10)
        assert stop.publisher == mock_publisher
        assert stop.node == mock_node
    
    def test_stop_motion_setup_missing_node(self):
        """Test StopMotion behaviour setup without node raises error"""
        stop = StopMotion()
        
        with pytest.raises(KeyError) as exc_info:
            stop.setup()  # No node provided
        
        # The error message should mention missing 'node'
        error_msg = str(exc_info.value)
        assert "node" in error_msg.lower() or "didn't find 'node'" in error_msg
    
    def test_stop_motion_update(self):
        """Test StopMotion behaviour update method"""
        stop = StopMotion()
        stop.publisher = Mock()
        
        status = stop.update()
        
        assert status == pt.common.Status.SUCCESS
        assert stop.publisher.publish.called
        
        # Check all velocities are zero
        twist_msg = stop.publisher.publish.call_args[0][0]
        assert twist_msg.linear.x == 0.0
        assert twist_msg.linear.y == 0.0
        assert twist_msg.linear.z == 0.0
        assert twist_msg.angular.x == 0.0
        assert twist_msg.angular.y == 0.0
        assert twist_msg.angular.z == 0.0
    
    def test_stop_motion_terminate(self):
        """Test StopMotion behaviour termination"""
        stop = StopMotion()
        stop.publisher = Mock()
        
        # Call terminate
        stop.terminate(pt.common.Status.SUCCESS)
        
        # Verify no extra publish calls
        assert stop.publisher.publish.call_count == 0
    
    def test_stop_motion_terminate_no_publisher(self):
        """Test StopMotion terminate when publisher not initialized"""
        stop = StopMotion()
        stop.publisher = None
        
        # Should not crash when publisher is None
        stop.terminate(pt.common.Status.SUCCESS)
        
        # Verify no exception was raised
        assert True
    
    def test_stop_motion_with_custom_topic(self):
        """Test StopMotion with custom topic name"""
        stop = StopMotion(topic_name="/emergency_cmd_vel")
        
        # Mock node
        mock_node = Mock()
        mock_publisher = Mock()
        mock_node.create_publisher.return_value = mock_publisher
        
        # Setup with custom topic
        stop.setup(node=mock_node)
        
        mock_node.create_publisher.assert_called_once_with(Twist, "/emergency_cmd_vel", 10)
        assert stop.publisher == mock_publisher
    
    def test_stop_motion_message_content(self):
        """Test that stop message has all zero velocities"""
        stop = StopMotion()
        stop.publisher = Mock()
        
        stop.update()
        
        twist_msg = stop.publisher.publish.call_args[0][0]
        
        # Verify all fields are explicitly set to 0.0
        assert twist_msg.linear.x == 0.0
        assert twist_msg.linear.y == 0.0
        assert twist_msg.linear.z == 0.0
        assert twist_msg.angular.x == 0.0
        assert twist_msg.angular.y == 0.0
        assert twist_msg.angular.z == 0.0
        
        # Also verify it's a proper Twist message
        assert isinstance(twist_msg, Twist)