In tactical sheduler ll 637:
    ```python
        first_wp = route_data['waypoints'][0]
        if not self.home_position:
            self.home_position = (
                first_wp['latitude'],
                first_wp['longitude'],
                first_wp.get('altitude', 0.0)
            )
            self.get_logger().info(
                f"Home position set from first waypoint: {self.home_position}"
            )
    ```
