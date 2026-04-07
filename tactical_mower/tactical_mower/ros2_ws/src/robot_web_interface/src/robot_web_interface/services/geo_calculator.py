"""Geographic calculations for home/charge points"""
import math


def calculate_home_point_from_charge(charge_lat: float, charge_lon: float, yaw_deg: float, distance_m: float = 2.0) -> tuple:
    """
    Compute the home point `distance_m` meters behind the mower.

    The input yaw is the ROS ENU yaw (0° = East, 90° = North). We move backwards
    along that heading and convert the offset to latitude/longitude.

    Args:
        charge_lat: Latitude of the charge point
        charge_lon: Longitude of the charge point
        yaw_deg: Robot yaw in degrees (ROS ENU convention)
        distance_m: Distance to move backwards (default: 2.0 m)

    Returns:
        (home_lat, home_lon): Coordinates of the computed home point
    """
    # Convert yaw to radians; ROS ENU: 0° = East, CCW positive
    yaw_rad = math.radians(yaw_deg)

    # Backwards vector in ENU coordinates
    delta_east = -distance_m * math.cos(yaw_rad)
    delta_north = -distance_m * math.sin(yaw_rad)

    # Earth radius in meters
    R = 6378137.0

    # Convert ENU offsets to latitude/longitude deltas
    lat_offset = (delta_north / R) * (180.0 / math.pi)
    lon_offset = (delta_east / (R * math.cos(math.radians(charge_lat)))) * (180.0 / math.pi)

    home_lat = charge_lat + lat_offset
    home_lon = charge_lon + lon_offset

    return home_lat, home_lon

