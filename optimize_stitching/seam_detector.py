import numpy as np
import cv2


def find_seam_line(overlap_region: np.ndarray, fallback_to_center: bool = True) -> np.ndarray:
    if overlap_region is None or overlap_region.size == 0:
        raise ValueError("overlap_region is empty")

    h, w = overlap_region.shape[:2]

    if h < 3 or w < 3:
        return np.full(h, w // 2, dtype=np.int32)

    if len(overlap_region.shape) == 2:
        gray = overlap_region
    else:
        gray = cv2.cvtColor(overlap_region, cv2.COLOR_BGR2GRAY)

    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    energy = np.sqrt(sobel_x**2 + sobel_y**2)

    if np.sum(energy) < 1e-6:
        if fallback_to_center:
            return np.full(h, w // 2, dtype=np.int32)
        else:
            raise ValueError("Cannot find seam in low-texture region")

    dp = energy.copy()
    path = np.zeros(h, dtype=np.int32)

    try:
        for y in range(1, h):
            prev_row = dp[y-1, :]
            dp[y, 1:-1] += np.minimum(prev_row[1:-1],
                                       np.minimum(prev_row[:-2], prev_row[2:]))
            dp[y, 0] += min(prev_row[0], prev_row[1])
            dp[y, -1] += min(prev_row[-1], prev_row[-2])

        path[h-1] = int(np.argmin(dp[h-1, :]))
        for y in range(h-2, -1, -1):
            x = path[y+1]
            candidates = []
            if x > 0:
                candidates.append((dp[y, x-1], x-1))
            candidates.append((dp[y, x], x))
            if x < w-1:
                candidates.append((dp[y, x+1], x+1))
            path[y] = min(candidates)[1]

    except Exception:
        return np.full(h, w // 2, dtype=np.int32)

    return path
