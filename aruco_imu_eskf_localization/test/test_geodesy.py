import unittest

import numpy as np

from aruco_imu_eskf_localization.common.geodesy import Wgs84Origin, lla_to_enu


class TestGeodesy(unittest.TestCase):
    def test_origin_projects_to_zero(self):
        origin = Wgs84Origin(
            latitude_deg=37.5418,
            longitude_deg=127.0794,
            altitude_m=15.0,
        )

        enu = lla_to_enu(
            origin.latitude_deg,
            origin.longitude_deg,
            origin.altitude_m,
            origin,
        )

        self.assertTrue(np.allclose(enu, np.zeros(3, dtype=float), atol=1.0e-6))

    def test_small_lat_lon_offset_projects_east_north_up(self):
        origin = Wgs84Origin(
            latitude_deg=37.5418,
            longitude_deg=127.0794,
            altitude_m=15.0,
        )

        east = lla_to_enu(origin.latitude_deg, origin.longitude_deg + 1.0e-5, 15.0, origin)
        north = lla_to_enu(origin.latitude_deg + 1.0e-5, origin.longitude_deg, 15.0, origin)
        up = lla_to_enu(origin.latitude_deg, origin.longitude_deg, 16.0, origin)

        self.assertGreater(east[0], 0.0)
        self.assertLess(abs(east[1]), 0.05)
        self.assertGreater(north[1], 0.0)
        self.assertLess(abs(north[0]), 0.05)
        self.assertAlmostEqual(up[2], 1.0, delta=1.0e-6)


if __name__ == '__main__':
    unittest.main()
