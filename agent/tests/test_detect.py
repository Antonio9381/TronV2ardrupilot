import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("cv2", reason="OpenCV not importable in this Python; camera tests skipped")

from tron.detect import ColourDetector, Locator, pixel_to_body


def test_colour_detector_and_geometry():
    frame = np.zeros((480, 640, 3), np.uint8)
    frame[:] = (40, 120, 40)
    frame[100:160, 480:540] = (0, 0, 220)
    blob = ColourDetector("red").find(frame)
    assert blob.nx == pytest.approx(0.6, abs=0.02)
    assert blob.ny == pytest.approx(-0.45, abs=0.02)
    fwd, right = pixel_to_body(blob, 10.0, 90.0, 640 / 480)
    assert right == pytest.approx(6.0, abs=0.2)
    assert fwd == pytest.approx(0.45 * 7.5, abs=0.2)
    assert ColourDetector("blue").find(frame) is None


def test_locator_with_camera():
    class Cam:
        def read(self):
            f = np.zeros((480, 640, 3), np.uint8)
            f[230:250, 310:330] = (220, 60, 0)
            return f

        def close(self):
            pass

    fwd, right = Locator(Cam(), ColourDetector("blue", 0.0005), 62.2).locate(10)
    assert abs(fwd) < 0.3 and abs(right) < 0.3
