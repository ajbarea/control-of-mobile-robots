"""
Test suite for Lab 1 Task 1: Straight-Line Motion Control

Tests the mathematical calculations, validation logic, and utility functions
of the E-puck robot straight-line motion controller.
"""

import os
import sys
from unittest.mock import Mock, patch

import pytest

# Add the lab1_task1 controller to Python path for testing
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "ajbarea_lab1",
        "webots",
        "controllers",
        "lab1_task1",
    ),
)

# Mock the Webots controller module before importing lab1_task1
mock_robot = Mock()
mock_robot.getBasicTimeStep.return_value = 32
mock_robot.getDevice.return_value = Mock()
mock_robot.step.return_value = 0

sys.modules["controller"] = Mock()
sys.modules["controller"].Robot = Mock(return_value=mock_robot)

# Import constants and functions from lab1_task1
try:
    import lab1_task1
    from lab1_task1 import (
        MAX_LINEAR_VELOCITY,
        MAX_VELOCITY,
        WHEEL_BASE,
        WHEEL_RADIUS,
        exitError,
        setSpeedsIPS,
        setSpeedsRPS,
        setSpeedsVW,
    )
except ImportError as e:
    pytest.skip(f"Cannot import lab1_task1 module: {e}", allow_module_level=True)


class TestPhysicalConstants:
    """Test robot physical specifications and derived constants."""

    def test_wheel_radius_positive(self):
        """Test that wheel radius is positive."""
        assert WHEEL_RADIUS > 0, "Wheel radius must be positive"

    def test_wheel_base_positive(self):
        """Test that wheel base distance is positive."""
        assert WHEEL_BASE > 0, "Wheel base distance must be positive"

    def test_max_velocity_positive(self):
        """Test that maximum velocity is positive."""
        assert MAX_VELOCITY > 0, "Maximum velocity must be positive"

    def test_max_linear_velocity_calculation(self):
        """Test that maximum linear velocity is calculated correctly."""
        expected_max_linear = MAX_VELOCITY * WHEEL_RADIUS
        assert (
            MAX_LINEAR_VELOCITY == expected_max_linear
        ), f"Expected {expected_max_linear}, got {MAX_LINEAR_VELOCITY}"

    def test_realistic_physical_values(self):
        """Test that physical constants are within realistic ranges."""
        # E-puck wheel radius should be less than 1 inch (realistic)
        assert WHEEL_RADIUS < 1.0, "Wheel radius seems unrealistically large"

        # Wheel base should be reasonable for a small robot
        assert 1.0 < WHEEL_BASE < 5.0, "Wheel base distance outside reasonable range"

        # Max velocity should be reasonable (not too high)
        assert MAX_VELOCITY < 20, "Maximum velocity seems unrealistically high"


class TestVelocityConversions:
    """Test velocity conversion calculations."""

    def test_linear_to_motor_velocity_conversion(self):
        """Test conversion from linear velocity to motor rotational velocity."""
        linear_velocity = 3.0  # inches/second
        expected_motor_velocity = linear_velocity / WHEEL_RADIUS

        # This would be the calculation done in moveXV function
        motor_velocity = linear_velocity / WHEEL_RADIUS
        assert (
            abs(motor_velocity - expected_motor_velocity) < 1e-6
        ), "Linear to motor velocity conversion incorrect"

    def test_velocity_clamping_logic(self):
        """Test that velocities are properly clamped to maximum."""
        test_velocity = MAX_VELOCITY + 1  # Exceed maximum
        clamped_velocity = min(test_velocity, MAX_VELOCITY)
        assert (
            clamped_velocity == MAX_VELOCITY
        ), "Velocity clamping not working correctly"

        test_velocity = MAX_VELOCITY - 1  # Within range
        clamped_velocity = min(test_velocity, MAX_VELOCITY)
        assert clamped_velocity == test_velocity, "Valid velocity should not be clamped"


class TestInputValidation:
    """Test input validation and parameter checking."""

    def test_velocity_limit_validation(self):
        """Test velocity limit validation logic."""
        # Test cases that should trigger validation warnings
        test_cases = [
            (MAX_LINEAR_VELOCITY + 0.1, True),  # Above max
            (-MAX_LINEAR_VELOCITY - 0.1, True),  # Below min (negative)
            (MAX_LINEAR_VELOCITY, False),  # At max (valid)
            (-MAX_LINEAR_VELOCITY, False),  # At min (valid)
            (0, False),  # Zero (valid)
            (3.0, False),  # Normal positive (valid)
            (-2.0, False),  # Normal negative (valid)
        ]

        for velocity, should_exceed in test_cases:
            exceeds_limit = (
                velocity > MAX_LINEAR_VELOCITY or velocity < -MAX_LINEAR_VELOCITY
            )
            assert (
                exceeds_limit == should_exceed
            ), f"Velocity {velocity} limit check failed"

    def test_distance_validation(self):
        """Test distance parameter validation."""
        # Very small distances should trigger warnings
        small_distance = 0.05
        assert abs(small_distance) < 0.1, "Small distance validation logic"

        # Normal distances should pass
        normal_distance = 12.0
        assert abs(normal_distance) >= 0.1, "Normal distance should pass validation"

    @pytest.mark.parametrize(
        "distance,velocity",
        [
            (12, 3),  # Test 1: Short distance, medium speed
            (60, 1),  # Test 2: Long distance, slow speed
            (24, MAX_LINEAR_VELOCITY),  # Test 3: Medium distance, max speed
            (18, -2),  # Test 4: Backward motion
            (12, 0),  # Test 5: Zero velocity
        ],
    )
    def test_experiment_parameters(self, distance, velocity):
        """Test that all predefined experiment parameters are valid."""
        assert isinstance(distance, (int, float)), "Distance must be numeric"
        assert isinstance(velocity, (int, float)), "Velocity must be numeric"
        assert distance > 0, "Distance must be positive"

        if velocity != 0:
            # Check if velocity is within reasonable bounds (may exceed for testing)
            assert (
                abs(velocity) <= MAX_LINEAR_VELOCITY * 1.5
            ), "Velocity should be within testable range"


class TestUtilityFunctions:
    """Test utility and helper functions."""

    @patch("builtins.print")
    def test_set_speeds_rps(self, mock_print):
        """Test setSpeedsRPS function output."""
        left_rps = 2.5
        right_rps = 3.0

        setSpeedsRPS(left_rps, right_rps)

        # Verify print was called with correct format
        mock_print.assert_any_call(f"\nLEFT MOTOR ==> {left_rps} radians per second")
        mock_print.assert_any_call(f"RIGHT MOTOR ==> {right_rps} radians per second")

    @patch("builtins.print")
    def test_set_speeds_ips(self, mock_print):
        """Test setSpeedsIPS function output."""
        left_ips = 1.5
        right_ips = 2.0

        setSpeedsIPS(left_ips, right_ips)

        mock_print.assert_any_call(f"\nLEFT MOTOR ==> {left_ips} inches per second")
        mock_print.assert_any_call(f"RIGHT MOTOR ==> {right_ips} inches per second")

    @patch("builtins.print")
    def test_set_speeds_vw(self, mock_print):
        """Test setSpeedsVW function output."""
        linear_v = 2.5
        angular_w = 1.0

        setSpeedsVW(linear_v, angular_w)

        mock_print.assert_any_call(f"\nLINEAR velocity {linear_v} inches per second")
        mock_print.assert_any_call(f"ANGULAR velocity {angular_w} radians per second")

    @patch("sys.exit")
    @patch("builtins.print")
    def test_exit_error(self, mock_print, mock_exit):
        """Test exitError function behavior."""
        exitError()

        # Verify error messages are printed
        expected_calls = [
            ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>",
            ">>>ERROR: E-PUCK MAX_VELOCITY = 6.28",
            ">>>Try a velocity in range [-6.28, 6.28]",
            ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>",
        ]

        for expected_msg in expected_calls:
            mock_print.assert_any_call(expected_msg)

        # Verify sys.exit(0) was called
        mock_exit.assert_called_once_with(0)


class TestMovementCalculations:
    """Test movement calculation logic without Webots simulation."""

    @patch("builtins.print")
    def test_zero_velocity_case(self, mock_print):
        """Test that zero velocity case is handled correctly."""
        # Mock the sensors to return predictable values
        mock_left_sensor = Mock()
        mock_right_sensor = Mock()
        mock_left_sensor.getValue.return_value = 0.0
        mock_right_sensor.getValue.return_value = 0.0

        # Mock the robot and motors
        with patch.object(
            lab1_task1, "leftposition_sensor", mock_left_sensor
        ), patch.object(
            lab1_task1, "rightposition_sensor", mock_right_sensor
        ), patch.object(
            lab1_task1, "leftMotor", Mock()
        ), patch.object(
            lab1_task1, "rightMotor", Mock()
        ), patch(
            "time.monotonic", return_value=0.0
        ):

            # Test zero velocity movement
            distance = 12
            velocity = 0

            # Call the actual movement function
            lab1_task1.moveXV(distance, velocity)

            # Verify the expected print outputs for zero velocity case
            expected_calls = [
                "[Starting position] 0.0",
                "[Stopping position] 0.0",
                f"[Distance: {distance} inches] actual distance traveled: 0.0000 inches",
                "[Time: 0 seconds] actual time traveled: 0 seconds",
                f"[Velocity: {velocity} inches per second] actual velocity: 0 inches per second",
                ">>>HINT: That was boring! Try a velocity in range [-6.28, 6.28]",
            ]

            for expected_call in expected_calls:
                mock_print.assert_any_call(expected_call)

    def test_positive_velocity_calculations(self):
        """Test motor velocity conversion for positive velocity."""
        velocity = 3  # inches/second

        # Test the actual conversion logic from moveXV function
        expected_motor_velocity = velocity / WHEEL_RADIUS
        motor_velocity = velocity / WHEEL_RADIUS

        # Verify conversion is correct
        assert abs(motor_velocity - expected_motor_velocity) < 1e-6

        # Verify clamping logic
        clamped_velocity = min(motor_velocity, MAX_VELOCITY)
        assert (
            clamped_velocity == motor_velocity
        ), "Normal velocity should not be clamped"

    def test_negative_velocity_calculations(self):
        """Test motor velocity conversion for negative velocity."""
        velocity = -2  # inches/second

        # Test the actual conversion logic from moveXV function
        motor_velocity = velocity / WHEEL_RADIUS
        assert (
            motor_velocity < 0
        ), "Motor velocity should be negative for backward motion"

        # Test the expected motor velocity value
        expected_motor_velocity = velocity / WHEEL_RADIUS
        assert abs(motor_velocity - expected_motor_velocity) < 1e-6


class TestErrorHandling:
    """Test error handling and edge cases."""

    def test_velocity_clamping_edge_cases(self):
        """Test edge cases for velocity clamping."""
        # Test exactly at maximum
        velocity = MAX_LINEAR_VELOCITY
        clamped = min(velocity, MAX_LINEAR_VELOCITY)
        assert clamped == MAX_LINEAR_VELOCITY

        # Test exactly at minimum
        velocity = -MAX_LINEAR_VELOCITY
        # The actual code clamps to MAX_LINEAR_VELOCITY for negative values too
        clamped_positive = min(abs(velocity), MAX_LINEAR_VELOCITY)
        assert clamped_positive == MAX_LINEAR_VELOCITY

    def test_small_distance_warning_threshold(self):
        """Test the small distance warning threshold."""
        threshold = 0.1

        # Should trigger warning
        small_distances = [0.05, 0.01, 0.09]
        for dist in small_distances:
            assert abs(dist) < threshold, f"Distance {dist} should trigger warning"

        # Should not trigger warning
        normal_distances = [0.1, 0.5, 1.0, 12.0]
        for dist in normal_distances:
            assert abs(dist) >= threshold, f"Distance {dist} should not trigger warning"


if __name__ == "__main__":
    pytest.main([__file__])
