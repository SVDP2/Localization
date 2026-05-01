from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass(frozen=True)
class Wgs84Origin:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    @property
    def latitude_rad(self) -> float:
        return math.radians(self.latitude_deg)

    @property
    def longitude_rad(self) -> float:
        return math.radians(self.longitude_deg)


def lla_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
    lat = math.radians(float(latitude_deg))
    lon = math.radians(float(longitude_deg))
    alt = float(altitude_m)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    prime_vertical_radius = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (prime_vertical_radius + alt) * cos_lat * cos_lon
    y = (prime_vertical_radius + alt) * cos_lat * sin_lon
    z = (prime_vertical_radius * (1.0 - WGS84_E2) + alt) * sin_lat
    return np.array([x, y, z], dtype=float)


def ecef_to_enu_rotation(origin: Wgs84Origin) -> np.ndarray:
    lat = origin.latitude_rad
    lon = origin.longitude_rad
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    return np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )


def lla_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    origin: Wgs84Origin,
) -> np.ndarray:
    origin_ecef = lla_to_ecef(origin.latitude_deg, origin.longitude_deg, origin.altitude_m)
    point_ecef = lla_to_ecef(latitude_deg, longitude_deg, altitude_m)
    return ecef_to_enu_rotation(origin) @ (point_ecef - origin_ecef)
