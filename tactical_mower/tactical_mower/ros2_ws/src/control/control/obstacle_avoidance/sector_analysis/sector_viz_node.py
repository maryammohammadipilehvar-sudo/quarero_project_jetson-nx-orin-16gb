"""RViz visualization node for obstacle sectors using MarkerArray."""

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from visualization_msgs.msg import Marker, MarkerArray
from interfaces.msg import ObstacleSectors
from std_msgs.msg import ColorRGBA

from .sector_config_loader import SectorConfigLoader


class SectorVizNode(Node):
    """Publishes sector visualization as MarkerArray for RViz."""
    
    def __init__(self):
        super().__init__('sector_viz')
        
        # Parameters
        self.declare_parameter('sector_analysis.config_path', '')
        self.declare_parameter('topics.sectors_input', '/obstacles/sectors')
        self.declare_parameter('topics.markers_output', '/obstacles/sectors_viz')
        self.declare_parameter('viz.frame_id', 'base_link')
        self.declare_parameter('viz.height', 0.1)  # Height of sector boxes [m]
        self.declare_parameter('viz.alpha_blocked', 0.7)  # Opacity when blocked
        self.declare_parameter('viz.alpha_free', 0.2)  # Opacity when free
        
        # Load parameters
        config_path = str(self.get_parameter('sector_analysis.config_path').value)
        if not config_path:
            config_path = None
        
        sectors_topic = str(self.get_parameter('topics.sectors_input').value)
        markers_topic = str(self.get_parameter('topics.markers_output').value)
        self.frame_id = str(self.get_parameter('viz.frame_id').value)
        self.box_height = float(self.get_parameter('viz.height').value)
        self.alpha_blocked = float(self.get_parameter('viz.alpha_blocked').value)
        self.alpha_free = float(self.get_parameter('viz.alpha_free').value)
        
        # Load sector definitions for geometry
        try:
            self.sectors_config = SectorConfigLoader.load_sectors(config_path)
            self.get_logger().info(f"Loaded {len(self.sectors_config)} sectors for visualization")
        except Exception as e:
            self.get_logger().error(f"Failed to load sectors config: {e}")
            raise
        
        # Create mapping from sector_id to config (sector_id = index in config)
        self.sector_config_map = {
            idx: sector for idx, sector in enumerate(self.sectors_config)
        }
        
        # Subscriber and publisher
        self.sectors_sub = self.create_subscription(
            ObstacleSectors,
            sectors_topic,
            self.sectors_callback,
            10
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray,
            markers_topic,
            10
        )
        
        self.get_logger().info(
            f"Sector visualization started: "
            f"subscribing to {sectors_topic}, "
            f"publishing to {markers_topic}, "
            f"frame={self.frame_id}"
        )
    
    def _get_color_for_sector(self, sector_type: str) -> ColorRGBA:
        """Get color based on sector type.
        
        Args:
            sector_type: "STOP" or "SLOW"
            
        Returns:
            ColorRGBA message
        """
        color = ColorRGBA()
        sector_type_upper = sector_type.upper()
        
        if sector_type_upper == "STOP":
            color.r = 1.0  # Red
            color.g = 0.0
            color.b = 0.0
        elif sector_type_upper == "SLOW":
            color.r = 1.0  # Yellow/Orange
            color.g = 0.7
            color.b = 0.0
        else:
            # Default: gray
            color.r = 0.5
            color.g = 0.5
            color.b = 0.5
        
        return color
    
    def sectors_callback(self, msg: ObstacleSectors):
        """Process ObstacleSectors and publish MarkerArray.
        
        Args:
            msg: ObstacleSectors message
        """
        markers = MarkerArray()
        
        # Use current time minus a small offset to avoid TF timing issues
        # This ensures the timestamp is available in the TF tree when RViz looks it up
        # Subtracting 0.1 seconds ensures the transform is always available
        now = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        
        # Delete all previous markers first (using DELETEALL)
        delete_marker = Marker()
        delete_marker.header.frame_id = self.frame_id
        delete_marker.header.stamp = now
        delete_marker.action = Marker.DELETEALL
        delete_marker.id = len(msg.sectors) + 1
        markers.markers.append(delete_marker)
        
        # Create marker for each sector
        for sector_info in msg.sectors:
            sector_id = sector_info.sector_id
            
            # Get sector geometry from config
            if sector_id not in self.sector_config_map:
                self.get_logger().warn(f"Sector ID {sector_id} not found in config, skipping")
                continue
            
            sector_config = self.sector_config_map[sector_id]
            
            # Create CUBE marker
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = "sectors"
            marker.id = sector_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position (center of sector)
            x_center = (sector_config['x_min'] + sector_config['x_max']) / 2.0
            y_center = (sector_config['y_min'] + sector_config['y_max']) / 2.0
            marker.pose.position.x = x_center
            marker.pose.position.y = y_center
            marker.pose.position.z = self.box_height / 2.0
            marker.pose.orientation.w = 1.0  # No rotation
            
            # Set scale (size of box)
            marker.scale.x = sector_config['x_max'] - sector_config['x_min']
            marker.scale.y = sector_config['y_max'] - sector_config['y_min']
            marker.scale.z = self.box_height
            
            # Set color based on sector type and blocked status
            color = self._get_color_for_sector(sector_config['sector_type'])
            if sector_info.blocked:
                color.a = self.alpha_blocked
            else:
                color.a = self.alpha_free
            marker.color = color
            
            # Set lifetime (0 = infinite)
            marker.lifetime.sec = 0
            
            markers.markers.append(marker)
        
        # Publish
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SectorVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
