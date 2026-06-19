"""Test offline construction of flexivrdk.Robot"""

import unittest
import flexivrdk


class RobotConstructorTest(unittest.TestCase):
    def assert_robot_constructor_failure(
        self, robot_sn, expected_exception, message_fragment
    ):
        with self.assertRaises(expected_exception) as context:
            flexivrdk.Robot(robot_sn, False)

        self.assertIn(message_fragment, str(context.exception))

    def test_invalid_format(self):
        self.assert_robot_constructor_failure(
            "Enlight412345", ValueError, "serial number is invalid"
        )

    def test_unsupported_model(self):
        for robot_sn in ("Enlight-X-123456", "Enlight X-123456"):
            with self.subTest(robot_sn=robot_sn):
                self.assert_robot_constructor_failure(
                    robot_sn, RuntimeError, "is not supported"
                )

    def test_supported_model_without_server(self):
        for robot_sn in ("Enlight-L-123456", "Enlight L-123456"):
            with self.subTest(robot_sn=robot_sn):
                self.assert_robot_constructor_failure(
                    robot_sn, RuntimeError, "Failed to connect"
                )


if __name__ == "__main__":
    unittest.main()
