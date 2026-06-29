import numpy as np
import cv2


def blend_with_seam(frame1: np.ndarray, frame2: np.ndarray,
                    seam_line: np.ndarray, overlap_width: int = None) -> np.ndarray:
    if frame1 is None or frame2 is None:
        raise ValueError("frame1 or frame2 is None")
    if seam_line is None or len(seam_line) == 0:
        raise ValueError("seam_line is empty")

    h1, w1 = frame1.shape[:2]
    h2, w2 = frame2.shape[:2]

    if h1 != h2:
        h = min(h1, h2)
        frame1 = frame1[:h, :, :]
        frame2 = frame2[:h, :, :]
        seam_line = seam_line[:h]
    else:
        h = h1

    if len(seam_line) != h:
        seam_line = np.resize(seam_line, h)

    if overlap_width is None:
        overlap_width = min(w1, w2) // 4
    overlap_width = max(1, min(overlap_width, min(w1, w2) // 2))

    result = np.zeros_like(frame1)

    for y in range(h):
        x_seam = int(seam_line[y])
        x_seam = max(0, min(x_seam, w1 - 1))

        blend_half = max(1, overlap_width // 2)

        for x in range(w1):
            if x < x_seam - blend_half:
                result[y, x] = frame1[y, x]
            elif x < x_seam + blend_half:
                dist = abs(x - x_seam)
                alpha = 1.0 - (dist / blend_half)
                alpha = alpha * alpha * (3 - 2 * alpha)

                x2 = x - (w1 - overlap_width)
                x2 = max(0, min(x2, w2 - 1))

                if x2 >= 0 and x2 < w2:
                    blended = cv2.addWeighted(
                        frame1[y, x], 1 - alpha,
                        frame2[y, x2], alpha, 0
                    )
                    result[y, x] = blended.squeeze()
                else:
                    result[y, x] = frame1[y, x]
            else:
                x2 = x - (w1 - overlap_width)
                x2 = max(0, min(x2, w2 - 1))
                if x2 >= 0 and x2 < w2:
                    result[y, x] = frame2[y, x2]
                else:
                    result[y, x] = frame1[y, x]

    return result


def simple_alpha_blend(frame1: np.ndarray, frame2: np.ndarray,
                       overlap_width: int) -> np.ndarray:
    h1, w1 = frame1.shape[:2]
    h2, w2 = frame2.shape[:2]
    h = min(h1, h2)

    if len(frame1.shape) == 3:
        frame1 = frame1[:h, :, :]
        frame2 = frame2[:h, :, :]
    else:
        frame1 = frame1[:h, :]
        frame2 = frame2[:h, :]

    overlap_width = max(1, min(overlap_width, w1, w2))

    if len(frame1.shape) == 2:
        result = np.zeros((h, w1), dtype=np.uint8)
    else:
        result = np.zeros((h, w1, 3), dtype=np.uint8)

    result[:, :w1 - overlap_width] = frame1[:, :w1 - overlap_width]

    for x in range(overlap_width):
        alpha = x / overlap_width
        x1 = w1 - overlap_width + x
        x2 = x
        blended = cv2.addWeighted(frame1[:, x1], 1 - alpha, frame2[:, x2], alpha, 0)
        result[:, x1] = blended.squeeze()

    return result
