import unittest

import cv2
import numpy as np
from spectra.detector import detect_gate, detect_finals


class DetectorTest(unittest.TestCase):
    def test_pair_has_box_and_confidence(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(image, (60, 30), (75, 210), (0, 128, 255), -1)
        cv2.rectangle(image, (240, 30), (255, 210), (0, 128, 255), -1)
        x, y, width, height, confidence = detect_gate(image)
        self.assertAlmostEqual(x, 158.0)
        self.assertGreater(width, 180)
        self.assertGreater(confidence, 0.9)
        # PNG roundtrip exercises a recorded image, not only an in-memory array.
        ok, encoded = cv2.imencode('.png', image)
        self.assertTrue(ok)
        self.assertEqual(detect_gate(cv2.imdecode(encoded, cv2.IMREAD_COLOR)), detect_gate(image))

    def test_blank_single_post_and_wrong_color_rejected(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        self.assertIsNone(detect_gate(image))
        cv2.rectangle(image, (60, 30), (75, 210), (0, 128, 255), -1)
        self.assertIsNone(detect_gate(image))
        cv2.rectangle(image, (240, 30), (255, 210), (255, 0, 0), -1)
        self.assertIsNone(detect_gate(image))

    def test_invalid_array_rejected(self):
        with self.assertRaises(ValueError):
            detect_gate(np.zeros((10, 10)))


class FinalsDetectorTest(unittest.TestCase):
    def detect(self, image):
        return detect_finals(
            image,
            [90, 120, 80],
            [130, 255, 255],
            [
                ('flare_red', [0, 150, 80], [8, 255, 255]),
                ('flare_green', [40, 120, 80], [85, 255, 255]),
                ('flare_yellow', [26, 120, 80], [38, 255, 255]),
            ],
        )

    def test_drum_and_colored_flares(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (120, 240), 50, (255, 0, 0), -1)
        for x, color in [(350, (0, 0, 255)), (450, (0, 255, 0)), (550, (0, 255, 255))]:
            cv2.rectangle(image, (x, 100), (x + 20, 380), color, -1)
        results = dict(self.detect(image))
        self.assertEqual(set(results), {'drum', 'flare_red', 'flare_green', 'flare_yellow'})
        self.assertLess(abs(results['drum'][0] - 120), 1)
        self.assertTrue(all(0.65 <= box[-1] <= 1 for box in results.values()))

    def test_wrong_shapes_and_blank_are_rejected(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.assertEqual(self.detect(image), [])
        cv2.rectangle(image, (20, 20), (220, 40), (0, 0, 255), -1)
        cv2.rectangle(image, (320, 20), (340, 400), (255, 0, 0), -1)
        self.assertEqual(self.detect(image), [])
