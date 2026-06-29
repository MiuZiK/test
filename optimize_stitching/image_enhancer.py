import numpy as np
import cv2
from typing import Tuple
from .config import StitcherConfig


def multiscale_sharpen(image: np.ndarray, amount: float = 1.5,
                       radius: float = 2.5) -> np.ndarray:
    if image is None or image.size == 0:
        return image

    if len(image.shape) == 2:
        gray = image.copy()
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (0, 0), radius)
    sharpened = cv2.addWeighted(gray, 1 + amount, blurred, -amount, 0)

    if len(image.shape) == 2:
        return sharpened

    result = image.copy()
    result[:, :, 0] = cv2.addWeighted(image[:, :, 0], 1 + amount, blurred, -amount, 0)
    result[:, :, 1] = cv2.addWeighted(image[:, :, 1], 1 + amount, blurred, -amount, 0)
    result[:, :, 2] = cv2.addWeighted(image[:, :, 2], 1 + amount, blurred, -amount, 0)

    return result


def optimize_contrast(image: np.ndarray, clip_limit: float = 3.5,
                      tile_grid_size: Tuple[int, int] = (8, 8)) -> np.ndarray:
    if image is None or image.size == 0:
        return image

    if len(image.shape) == 2:
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
        return clahe.apply(image)

    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    l_clahe = clahe.apply(l)

    lab_clahe = cv2.merge((l_clahe, a, b))
    return cv2.cvtColor(lab_clahe, cv2.COLOR_LAB2BGR)


def enhance(image: np.ndarray, config: StitcherConfig, should_sharpen: bool = True) -> np.ndarray:
    if image is None or image.size == 0:
        return image

    result = image.copy()

    if config.enhance_contrast:
        result = optimize_contrast(result, clip_limit=config.clahe_clip)

    if should_sharpen:
        result = multiscale_sharpen(result, amount=config.sharpen_amount)

    return result
