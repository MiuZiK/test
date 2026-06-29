import numpy as np
import cv2
from typing import List, Tuple


def compute_change_score(frame1: np.ndarray, frame2: np.ndarray) -> float:
    if frame1.shape != frame2.shape:
        h = min(frame1.shape[0], frame2.shape[0])
        w = min(frame1.shape[1], frame2.shape[1])
        frame1 = cv2.resize(frame1, (w, h))
        frame2 = cv2.resize(frame2, (w, h))

    if len(frame1.shape) == 2:
        gray1, gray2 = frame1, frame2
    else:
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    mean1 = np.mean(gray1)

    if mean1 < 5 or mean1 > 250:
        return -1.0

    std1, std2 = np.std(gray1), np.std(gray2)
    if std1 < 2 or std2 < 2:
        return 0.0

    diff_threshold = max(10, np.std(gray1) * 0.5)

    sobel1_x = cv2.Sobel(gray1, cv2.CV_64F, 1, 0, ksize=3)
    sobel1_y = cv2.Sobel(gray1, cv2.CV_64F, 0, 1, ksize=3)
    edge_density1 = np.mean(np.sqrt(sobel1_x**2 + sobel1_y**2))

    diff = cv2.absdiff(gray1.astype(np.float64), gray2.astype(np.float64))
    mean_diff = np.mean(diff)

    _, thresh = cv2.threshold(diff, diff_threshold, 255, cv2.THRESH_BINARY)
    changed_pixels = np.count_nonzero(thresh) / thresh.size * 100

    if mean_diff > 100:
        return 100.0

    score = (mean_diff / 50 * 30) + (changed_pixels / 100 * 40) + (edge_density1 / 200 * 30)
    return float(min(100, max(0, score)))


def detect_interest_regions(frame1: np.ndarray, frame2: np.ndarray,
                            min_area: int = 500) -> List[Tuple[int, int, int, int]]:
    if frame1.shape != frame2.shape:
        h = min(frame1.shape[0], frame2.shape[0])
        w = min(frame1.shape[1], frame2.shape[1])
        frame1 = cv2.resize(frame1, (w, h))
        frame2 = cv2.resize(frame2, (w, h))

    if len(frame1.shape) == 2:
        gray1, gray2 = frame1, frame2
    else:
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    diff = cv2.absdiff(gray1, gray2)
    _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    regions = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_area:
            x, y, w, h = cv2.boundingRect(contour)
            regions.append((x, y, x + w, y + h))

    return regions
