import cv2
import numpy as np
from typing import Tuple


def estimate_flow_translation(img1: np.ndarray, img2: np.ndarray) -> Tuple[float, float, float, str]:
    h, w = img1.shape[:2]

    if len(img1.shape) == 2:
        gray1 = img1
    else:
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

    if len(img2.shape) == 2:
        gray2 = img2
    else:
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    try:
        warp_matrix = np.eye(2, 3, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-6)
        _, ecc_status = cv2.findTransformECC(
            gray1, gray2, warp_matrix, cv2.MOTION_TRANSLATION, criteria
        )
        if ecc_status > 0:
            dx, dy = warp_matrix[0, 2], warp_matrix[1, 2]
            return dx, dy, ecc_status * 100, 'ecc'
    except Exception:
        pass

    try:
        flow = cv2.calcOpticalFlowFarneback(
            gray1, gray2, None, 0.5, 3, 15, 3, 5, 1.2, 0
        )
        margin_h, margin_w = h // 4, w // 4
        center_flow = flow[margin_h:h-margin_h, margin_w:w-margin_w]
        dx = np.median(center_flow[:, :, 0])
        dy = np.median(center_flow[:, :, 1])

        flow_mag = np.sqrt(dx**2 + dy**2)
        return dx, dy, flow_mag * 10, 'farneback'
    except Exception:
        pass

    try:
        orb = cv2.ORB_create(nfeatures=1000)
        kp1, des1 = orb.detectAndCompute(gray1, None)
        kp2, des2 = orb.detectAndCompute(gray2, None)
        if des1 is not None and len(kp1) >= 10:
            bf = cv2.BFMatcher(cv2.NORM_HAMMING)
            matches = bf.knnMatch(des1, des2, k=2)
            good = [m[0] for m in matches if len(m)==2 and m[0].distance < 0.75*m[1].distance]
            if len(good) >= 15:
                src = np.float32([kp1[m.queryIdx].pt for m in good])
                dst = np.float32([kp2[m.trainIdx].pt for m in good])
                M, _ = cv2.estimateAffinePartial2D(src, dst)
                if M is not None:
                    return M[0,2], M[1,2], len(good), 'orb'
    except Exception:
        pass

    return 0.0, 0.0, 0.0, 'none'


def crop_black(img: np.ndarray, threshold: int = 5) -> np.ndarray:
    if img is None or img.size == 0:
        return img

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    coords = cv2.findNonZero(thresh)
    if coords is None:
        return img

    x, y, w, h = cv2.boundingRect(coords)
    return img[max(0, y-3):min(img.shape[0], y+h+3), max(0, x-3):min(img.shape[1], x+w+3)].copy()
