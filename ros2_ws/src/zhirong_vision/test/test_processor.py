import cv2
import numpy as np
from pathlib import Path

from zhirong_vision.processor import VisionProcessor


def test_detects_blue_rectangle():
    image = np.full((240, 320, 3), 235, dtype=np.uint8)
    cv2.rectangle(image, (80, 45), (245, 205), (220, 70, 20), -1)

    result = VisionProcessor(min_color_area_ratio=0.02).process(image)

    assert result.color is not None
    assert result.color.label == "blue"
    assert result.color.area_ratio > 0.25
    assert result.color.confidence > 0.8


def test_ignores_tiny_color_noise():
    image = np.full((240, 320, 3), 235, dtype=np.uint8)
    cv2.rectangle(image, (5, 5), (12, 12), (255, 0, 0), -1)

    result = VisionProcessor(min_color_area_ratio=0.02).process(image)

    assert result.color is None


def test_decodes_navigation_qr_asset():
    asset_path = Path(__file__).parent / "assets" / "nav_home.png"
    image = cv2.imread(str(asset_path), cv2.IMREAD_COLOR)

    result = VisionProcessor(min_color_area_ratio=0.02).process(image)

    assert result.qr is not None
    assert result.qr.data == "NAV:HOME"
