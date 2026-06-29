import cv2
import os
from typing import List
import numpy as np
from .exceptions import OutputWriteError
from .logger import log_error, log_info, log_warning


def write_segments(images: List[np.ndarray], output_dir: str,
                   prefix: str = "output", quality: int = 97) -> List[str]:
    os.makedirs(output_dir, exist_ok=True)

    saved_files = []
    segment_index = 1

    for img in images:
        if img is None or img.size == 0:
            log_warning("write_segments", "Skipping empty image")
            continue

        filename = f"{prefix}_{segment_index:03d}.jpg"
        filepath = os.path.join(output_dir, filename)

        try:
            success = cv2.imwrite(filepath, img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if success:
                saved_files.append(filepath)
                log_info("write_segments", f"Saved: {filepath}")
                segment_index += 1
            else:
                raise OutputWriteError(f"Failed to write {filepath}")
        except Exception as e:
            log_error("write_segments", e)

    return saved_files


def save_full_image(image: np.ndarray, filepath: str, quality: int = 97) -> bool:
    directory = os.path.dirname(filepath)
    if directory:
        os.makedirs(directory, exist_ok=True)

    try:
        success = cv2.imwrite(filepath, image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if success:
            log_info("save_full_image", f"Saved: {filepath}")
            return True
        else:
            raise OutputWriteError(f"Failed to write {filepath}")
    except Exception as e:
        log_error("save_full_image", e)
        return False
