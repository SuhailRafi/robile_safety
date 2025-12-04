# test/test_behaviours/test_battery_status.py
import pytest
from unittest.mock import patch
import py_trees as pt

from robile_safety.safety_bt import BatteryStatus2bb

class TestBatteryStatus2bb:
    """Test suite for BatteryStatus2bb behaviour"""
    
    @pytest.fixture
    def battery_behaviour(self):
        """Fixture to create BatteryStatus2bb instance"""
        return BatteryStatus2bb(threshold=30.0)
    
    def test_battery_initialization(self, battery_behaviour):
        """Test BatteryStatus2bb initialization"""
        assert battery_behaviour.threshold == 30.0
        
        # Check blackboard variables initialized
        assert battery_behaviour.blackboard.battery == 100.0
        assert battery_behaviour.blackboard.battery_low_warning is False
        
        # Test with custom threshold
        battery_custom = BatteryStatus2bb(threshold=40.0)
        assert battery_custom.threshold == 40.0
    
    def test_battery_topic_name(self, battery_behaviour):
        """Test battery topic name configuration"""
        battery_custom = BatteryStatus2bb(
            battery_voltage_topic_name="/custom_battery",
            threshold=25.0
        )
        assert battery_custom.topic_name == "/custom_battery"
        assert battery_custom.threshold == 25.0
    
    @pytest.mark.parametrize("battery_level,expected_warning", [
        (25.0, True),   # Below threshold
        (30.0, False),  # At threshold (should not warn)
        (50.0, False),  # Above threshold
        (0.0, True),    # Minimum
        (100.0, False), # Maximum
        (29.9, True),   # Just below threshold
        (30.1, False),  # Just above threshold
    ])
    def test_battery_warning_logic(self, battery_behaviour, battery_level, expected_warning):
        """Test battery warning logic with different levels"""
        # Mock the parent update to return SUCCESS
        with patch.object(battery_behaviour.__class__.__bases__[0], 'update') as mock_parent_update:
            mock_parent_update.return_value = pt.common.Status.SUCCESS
            
            # Set battery level
            battery_behaviour.blackboard.battery = battery_level
            
            # Call update
            status = battery_behaviour.update()
            
            assert status == pt.common.Status.SUCCESS
            assert battery_behaviour.blackboard.battery_low_warning == expected_warning
    
    def test_battery_update_running(self, battery_behaviour):
        """Test battery update when parent returns RUNNING"""
        with patch.object(battery_behaviour.__class__.__bases__[0], 'update') as mock_parent_update:
            mock_parent_update.return_value = pt.common.Status.RUNNING
            
            status = battery_behaviour.update()
            
            assert status == pt.common.Status.RUNNING
    
    def test_battery_update_failure(self, battery_behaviour):
        """Test battery update when parent returns FAILURE"""
        with patch.object(battery_behaviour.__class__.__bases__[0], 'update') as mock_parent_update:
            mock_parent_update.return_value = pt.common.Status.FAILURE
            
            status = battery_behaviour.update()
            
            # Should still process battery logic even on parent FAILURE
            assert status == pt.common.Status.SUCCESS
    
    def test_battery_blackboard_persistence(self, battery_behaviour):
        """Test that battery warning persists on blackboard"""
        with patch.object(battery_behaviour.__class__.__bases__[0], 'update') as mock_parent_update:
            mock_parent_update.return_value = pt.common.Status.SUCCESS
            
            # Set low battery
            battery_behaviour.blackboard.battery = 20.0
            battery_behaviour.update()
            
            assert battery_behaviour.blackboard.battery_low_warning is True
            
            # Set high battery
            battery_behaviour.blackboard.battery = 80.0
            battery_behaviour.update()
            
            # Warning should clear
            assert battery_behaviour.blackboard.battery_low_warning is False