"""Node that publishes intelligent obstacle sectors from OccupancyGrid."""

import traceback
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from interfaces.msg import ObstacleSectors, SectorInfo
from std_msgs.msg import Header

from .sector_analyzer import SectorAnalyzer
from .sector_config_loader import SectorConfigLoader


class SectorPublisherNode(Node):
    """Publishes intelligent obstacle sectors analyzed from OccupancyGrid."""
    
    def __init__(self):
        super().__init__('sector_publisher')
        
        # Sector analysis parameters
        self.declare_parameter('sector_analysis.max_range', 3.0)
        self.declare_parameter('sector_analysis.config_path', '')  # Path to sector_config.yaml, empty = auto-detect
        
        # Topics
        self.declare_parameter('topics.obstacles_input', '/obstacles/lidar')
        self.declare_parameter('topics.sectors_output', '/obstacles/sectors')
        
        # Load parameters
        max_range = float(self.get_parameter('sector_analysis.max_range').value)
        
        obstacles_topic = str(self.get_parameter('topics.obstacles_input').value)
        sectors_topic = str(self.get_parameter('topics.sectors_output').value)
        
        # Load sectors from config file
        config_path = str(self.get_parameter('sector_analysis.config_path').value)
        if not config_path:
            config_path = None  # Use auto-detection
        
        try:
            sectors_list = SectorConfigLoader.load_sectors(config_path)
            self.get_logger().info(f"Loaded {len(sectors_list)} sectors from config")
        except Exception as e:
            self.get_logger().error(f"Failed to load sectors from config: {e}\n{traceback.format_exc()}")
            raise ValueError(f"Failed to load sectors: {e}")
        
        # Initialize sector analyzer
        self._sector_analyzer = SectorAnalyzer(
            sectors_config=sectors_list,
            max_range=max_range
        )
        
        # Subscriber and publisher
        self.obstacles_sub = self.create_subscription(
            OccupancyGrid,
            obstacles_topic,
            self.obstacles_callback,
            10
        )
        
        self.sectors_pub = self.create_publisher(
            ObstacleSectors,
            sectors_topic,
            10
        )
        
        self.get_logger().info(
            f"Sector publisher started: "
            f"subscribing to {obstacles_topic}, "
            f"publishing to {sectors_topic}, "
            f"num_sectors={len(sectors_list)}, max_range={max_range}m"
        )
    
    def obstacles_callback(self, msg: OccupancyGrid):
        """Process OccupancyGrid and publish sectors.
        
        Args:
            msg: OccupancyGrid message from obstacle detection
        """
        try:
            # Analyze sectors using sector analyzer
            sectors = self._sector_analyzer.analyze(msg)
            
            # Create ObstacleSectors message
            sectors_msg = ObstacleSectors()
            sectors_msg.header = msg.header
            sectors_msg.sectors = sectors
            
            # Publish
            self.sectors_pub.publish(sectors_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing OccupancyGrid: {e}\n{traceback.format_exc()}")


def main(args=None):
    rclpy.init(args=args)
    node = SectorPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

