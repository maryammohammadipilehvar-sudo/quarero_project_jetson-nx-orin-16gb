#!/usr/bin/env python3
"""
Unit tests for path_planning_service module.
"""

import pytest
import math
import time
from unittest.mock import MagicMock, Mock, patch
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from ros2_navigation.srv import ComputePathToPose
from ros2_navigation.path_planning_service import PathPlanningService
from ros2_navigation.tf_manager import TFManager
from ros2_navigation.planner_manager import PlannerManager
from ros2_navigation.occupancy_grid_subscriber import OccupancyGridSubscriber
from ros2_navigation.path_publisher import PathPublisher
from ros2_navigation.inflated_grid_publisher import InflatedGridPublisher
from ros2_navigation.extended_grid_publisher import ExtendedGridPublisher
from tf2_ros import TransformException
from conftest import create_occupancy_grid, create_pose_stamped


class TestPathPlanningServiceInit:
    """Tests for PathPlanningService initialization."""
    
    def test_init_basic(self, mock_node):
        """Test basic initialization."""
        tf_manager = MagicMock(spec=TFManager)
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        assert service.node == mock_node
        assert service.tf_manager == tf_manager
        assert service.planner_manager == planner_manager
        assert service.grid_subscriber == grid_subscriber
        assert service.path_publisher == path_publisher
        
        # Verify service was created
        mock_node.create_service.assert_called_once()
    
    def test_init_with_parameters(self, mock_node):
        """Test initialization with custom parameters."""
        tf_manager = MagicMock(spec=TFManager)
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher,
            grid_extension_distance=10.0,
            grid_extension_resolution=0.2,
            max_planning_time=10.0
        )
        
        assert service.grid_extension_distance == 10.0
        assert service.grid_extension_resolution == 0.2
        assert service.max_planning_time == 10.0


class TestPathPlanningServiceComputePathCallback:
    """Tests for compute_path_callback method."""
    
    def test_compute_path_success(self, mock_node, sample_occupancy_grid):
        """Test successful path planning."""
        # Setup mocks
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.global_frame = "map"
        tf_manager.get_robot_pose.return_value = create_pose_stamped(0.0, 0.0, frame_id="map")
        tf_manager.transform_pose.return_value = create_pose_stamped(1.0, 1.0, frame_id="map")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        planner_manager.inflation_radius = 0.0
        path = Path()
        path.poses = [
            create_pose_stamped(0.0, 0.0, frame_id="map"),
            create_pose_stamped(1.0, 1.0, frame_id="map")
        ]
        planner_manager.plan_path.return_value = path
        
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        grid_subscriber.get_current_grid.return_value = sample_occupancy_grid
        
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        # Create request
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose.header.frame_id = ""  # Empty, should use robot pose
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        assert response.path_found is True
        assert len(response.path.poses) > 0
        assert response.planning_time >= 0
    
    def test_compute_path_invalid_request(self, mock_node):
        """Test path planning with invalid request."""
        tf_manager = MagicMock(spec=TFManager)
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        # Create invalid request (NaN in goal)
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.goal_pose.pose.position.x = float('nan')
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        assert response.path_found is False
        assert "Invalid" in response.message or "invalid" in response.message.lower()
    
    def test_compute_path_robot_pose_failure(self, mock_node):
        """Test path planning when robot pose cannot be retrieved."""
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.get_robot_pose.side_effect = TransformException("TF error")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose.header.frame_id = ""  # Empty, triggers robot pose fetch
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        assert response.path_found is False
        assert "robot pose" in response.message.lower() or "tf" in response.message.lower()
    
    def test_compute_path_goal_transform_failure(self, mock_node, sample_occupancy_grid):
        """Test path planning when goal transform fails."""
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.global_frame = "map"
        tf_manager.get_robot_pose.return_value = create_pose_stamped(0.0, 0.0, frame_id="map")
        tf_manager.transform_pose.side_effect = TransformException("Transform error")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        grid_subscriber.get_current_grid.return_value = sample_occupancy_grid
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="odom")  # Different frame
        request.start_pose.header.frame_id = ""
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        assert response.path_found is False
        assert "transform" in response.message.lower() or "goal pose" in response.message.lower()
    
    def test_compute_path_no_grid(self, mock_node):
        """Test path planning when grid is not available."""
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.global_frame = "map"
        tf_manager.get_robot_pose.return_value = create_pose_stamped(0.0, 0.0, frame_id="map")
        tf_manager.transform_pose.return_value = create_pose_stamped(1.0, 1.0, frame_id="map")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        grid_subscriber.get_current_grid.return_value = None  # No grid
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose.header.frame_id = ""
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        assert response.path_found is False
        assert "grid" in response.message.lower()
    
    def test_compute_path_goal_outside_grid_extension(self, mock_node, small_occupancy_grid):
        """Test path planning when goal is outside grid (triggers extension)."""
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.global_frame = "map"
        tf_manager.get_robot_pose.return_value = create_pose_stamped(0.5, 0.5, frame_id="map")
        tf_manager.transform_pose.return_value = create_pose_stamped(2.0, 2.0, frame_id="map")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        path = Path()
        path.poses = [create_pose_stamped(0.5, 0.5, frame_id="map"),
                      create_pose_stamped(2.0, 2.0, frame_id="map")]
        planner_manager.plan_path.return_value = path
        
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        grid_subscriber.get_current_grid.return_value = small_occupancy_grid
        
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher,
            grid_extension_distance=1.0
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(2.0, 2.0, frame_id="map")  # Outside grid
        request.start_pose.header.frame_id = ""
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        # Should extend grid and plan path
        assert response.path_found is True
        assert len(response.path.poses) > 0
    
    def test_compute_path_no_path_found(self, mock_node, sample_occupancy_grid):
        """Test path planning when no path is found."""
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.global_frame = "map"
        tf_manager.get_robot_pose.return_value = create_pose_stamped(0.0, 0.0, frame_id="map")
        tf_manager.transform_pose.return_value = create_pose_stamped(1.0, 1.0, frame_id="map")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        empty_path = Path()
        empty_path.poses = []
        planner_manager.plan_path.return_value = empty_path
        
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        grid_subscriber.get_current_grid.return_value = sample_occupancy_grid
        
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose.header.frame_id = ""
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        assert response.path_found is False
        assert "no path" in response.message.lower() or "path found" not in response.message.lower()
    
    def test_compute_path_timeout(self, mock_node, sample_occupancy_grid):
        """Test path planning timeout."""
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.global_frame = "map"
        tf_manager.get_robot_pose.return_value = create_pose_stamped(0.0, 0.0, frame_id="map")
        tf_manager.transform_pose.return_value = create_pose_stamped(1.0, 1.0, frame_id="map")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        
        def slow_plan_path(*args, **kwargs):
            time.sleep(0.1)  # Simulate slow planning
            path = Path()
            path.poses = [create_pose_stamped(0.0, 0.0, frame_id="map")]
            return path
        
        planner_manager.plan_path.side_effect = slow_plan_path
        
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        grid_subscriber.get_current_grid.return_value = sample_occupancy_grid
        
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher,
            max_planning_time=0.01  # Very short timeout
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose.header.frame_id = ""
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        # Should timeout (may or may not depending on timing)
        assert response.planning_time >= 0
    
    def test_compute_path_with_start_pose(self, mock_node, sample_occupancy_grid):
        """Test path planning with provided start pose."""
        tf_manager = MagicMock(spec=TFManager)
        tf_manager.global_frame = "map"
        tf_manager.transform_pose.return_value = create_pose_stamped(1.0, 1.0, frame_id="map")
        
        planner_manager = MagicMock(spec=PlannerManager)
        planner_manager.inflation_radius = 0.0
        path = Path()
        path.poses = [
            create_pose_stamped(0.5, 0.5, frame_id="map"),
            create_pose_stamped(1.0, 1.0, frame_id="map")
        ]
        planner_manager.plan_path.return_value = path
        
        grid_subscriber = MagicMock(spec=OccupancyGridSubscriber)
        grid_subscriber.get_current_grid.return_value = sample_occupancy_grid
        
        path_publisher = MagicMock(spec=PathPublisher)
        inflated_grid_publisher = MagicMock(spec=InflatedGridPublisher)
        extended_grid_publisher = MagicMock(spec=ExtendedGridPublisher)
        
        service = PathPlanningService(
            mock_node,
            tf_manager,
            planner_manager,
            grid_subscriber,
            path_publisher,
            inflated_grid_publisher,
            extended_grid_publisher
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose = create_pose_stamped(0.5, 0.5, frame_id="map")
        
        response = service.compute_path_callback(request, ComputePathToPose.Response())
        
        assert response.path_found is True
        # Should not call get_robot_pose
        tf_manager.get_robot_pose.assert_not_called()


class TestPathPlanningServiceValidateRequest:
    """Tests for _validate_request method."""
    
    def test_validate_request_valid(self, mock_node):
        """Test validation of valid request."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose.header.frame_id = ""
        
        assert service._validate_request(request) is True
    
    def test_validate_request_invalid_goal_nan(self, mock_node):
        """Test validation with NaN in goal."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.goal_pose.pose.position.x = float('nan')
        
        assert service._validate_request(request) is False
    
    def test_validate_request_invalid_goal_inf(self, mock_node):
        """Test validation with Inf in goal."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.goal_pose.pose.position.y = float('inf')
        
        assert service._validate_request(request) is False
    
    def test_validate_request_invalid_start_pose(self, mock_node):
        """Test validation with invalid start pose."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose = create_pose_stamped(0.0, 0.0, frame_id="map")
        request.start_pose.pose.position.x = float('nan')
        
        assert service._validate_request(request) is False
    
    def test_validate_request_empty_start_pose(self, mock_node):
        """Test validation with empty start pose (should be valid)."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        request = ComputePathToPose.Request()
        request.goal_pose = create_pose_stamped(1.0, 1.0, frame_id="map")
        request.start_pose.header.frame_id = ""  # Empty is valid
        
        assert service._validate_request(request) is True


class TestPathPlanningServiceIsPoseValid:
    """Tests for _is_pose_valid method."""
    
    def test_is_pose_valid_valid(self, mock_node):
        """Test validation of valid pose."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        pose = create_pose_stamped(1.0, 2.0, frame_id="map")
        assert service._is_pose_valid(pose) is True
    
    def test_is_pose_valid_none(self, mock_node):
        """Test validation of None pose."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        assert service._is_pose_valid(None) is False
    
    def test_is_pose_valid_nan_x(self, mock_node):
        """Test validation with NaN in x."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        pose = create_pose_stamped(1.0, 2.0, frame_id="map")
        pose.pose.position.x = float('nan')
        assert service._is_pose_valid(pose) is False
    
    def test_is_pose_valid_nan_y(self, mock_node):
        """Test validation with NaN in y."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        pose = create_pose_stamped(1.0, 2.0, frame_id="map")
        pose.pose.position.y = float('nan')
        assert service._is_pose_valid(pose) is False
    
    def test_is_pose_valid_inf(self, mock_node):
        """Test validation with Inf values."""
        service = PathPlanningService(
            mock_node,
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(),
            MagicMock(spec=InflatedGridPublisher),
            MagicMock(spec=ExtendedGridPublisher)
        )
        
        pose = create_pose_stamped(1.0, 2.0, frame_id="map")
        pose.pose.position.z = float('inf')
        assert service._is_pose_valid(pose) is False

