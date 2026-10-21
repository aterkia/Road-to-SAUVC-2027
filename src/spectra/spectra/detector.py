"""Calibratable two-post gate detector; confidence is a geometric quality score."""
from itertools import combinations

import cv2
import numpy as np


def detect_gate(image, lower=(0, 120, 80), upper=(25, 255, 255), min_area=80,
                min_aspect=2.0):
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('expected uint8 BGR image')
    if image.size == 0:
        return None
    mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV),
                       np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    posts = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) >= min_area and h / w >= min_aspect:
            posts.append((x, y, w, h))
    best = None
    for a, b in combinations(posts, 2):
        if a[0] > b[0]:
            a, b = b, a
        overlap = min(a[1]+a[3], b[1]+b[3]) - max(a[1], b[1])
        similarity = min(a[3], b[3]) / max(a[3], b[3])
        separation = b[0] - (a[0]+a[2])
        if separation <= max(a[2], b[2]) or overlap <= 0:
            continue
        score = min(overlap / max(a[3], b[3]), similarity)
        if score < 0.6:
            continue
        x, y = a[0], min(a[1], b[1])
        w, h = b[0]+b[2]-x, max(a[1]+a[3], b[1]+b[3])-y
        if best is None or score > best[4]:
            best = (x+w/2, y+h/2, float(w), float(h), float(score))
    return best


def detect_finals(image, drum_lower, drum_upper, flare_ranges, min_area=200):
    """Lab baselines only: circular drum and tall colored flares, no payload logic."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    detections = []
    targets = [('drum', drum_lower, drum_upper)] + flare_ranges
    for label, lower, upper in targets:
        mask = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if label == 'drum':
                perimeter = cv2.arcLength(contour, True)
                score = 4*np.pi*area/perimeter**2 if perimeter else 0
                if not 0.7 <= w/h <= 1.4 or score < 0.65:
                    continue
            else:
                if h/w < 3:
                    continue
                score = area/(w*h)
            detections.append((label, (x+w/2, y+h/2, float(w), float(h), min(1.0, float(score)))))
    return detections
