from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    from pyzbar.pyzbar import ZBarSymbol, decode as zbar_decode
except ImportError:
    ZBarSymbol = None
    zbar_decode = None


@dataclass
class ColorDetection:
    label: str
    confidence: float
    area_ratio: float
    bbox: Tuple[int, int, int, int]


@dataclass
class QrDetection:
    data: str
    bbox: List[Tuple[int, int]]


@dataclass
class DetectionResult:
    color: Optional[ColorDetection]
    qr: Optional[QrDetection]

    def to_dict(self) -> Dict:
        return {
            "color": asdict(self.color) if self.color else None,
            "qr": asdict(self.qr) if self.qr else None,
        }


class VisionProcessor:
    """Pure OpenCV processor kept separate from ROS for deterministic tests."""

    COLOR_RANGES: Dict[str, Sequence[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = {
        "red": (
            ((0, 110, 70), (10, 255, 255)),
            ((170, 110, 70), (180, 255, 255)),
        ),
        "orange": (((11, 100, 80), (28, 255, 255)),),
        "green": (((35, 70, 55), (88, 255, 255)),),
        "blue": (((90, 75, 55), (138, 255, 255)),),
    }

    DRAW_COLORS = {
        "red": (0, 0, 255),
        "orange": (0, 140, 255),
        "green": (0, 220, 0),
        "blue": (255, 100, 0),
    }

    def __init__(self, min_color_area_ratio: float = 0.025):
        self.min_color_area_ratio = min_color_area_ratio
        self.qr_detector = cv2.QRCodeDetector()
        self._kernel = np.ones((5, 5), dtype=np.uint8)

    def process(self, image_bgr: np.ndarray) -> DetectionResult:
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("image_bgr must contain image data")

        return DetectionResult(
            color=self._detect_color(image_bgr),
            qr=self._detect_qr(image_bgr),
        )

    def annotate(
        self,
        image_bgr: np.ndarray,
        result: DetectionResult,
    ) -> np.ndarray:
        annotated = image_bgr.copy()
        if result.color:
            x, y, width, height = result.color.bbox
            draw_color = self.DRAW_COLORS.get(result.color.label, (255, 255, 255))
            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                draw_color,
                2,
            )
            cv2.putText(
                annotated,
                f"{result.color.label} {result.color.confidence:.2f}",
                (x, max(18, y - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                draw_color,
                2,
                cv2.LINE_AA,
            )

        if result.qr:
            points = np.array(result.qr.bbox, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated, [points], True, (255, 0, 255), 2)
            label_origin = tuple(points[0, 0])
            cv2.putText(
                annotated,
                result.qr.data[:48],
                (int(label_origin[0]), max(18, int(label_origin[1]) - 7)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

        return annotated

    def _detect_color(self, image_bgr: np.ndarray) -> Optional[ColorDetection]:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        image_area = float(image_bgr.shape[0] * image_bgr.shape[1])
        best: Optional[ColorDetection] = None

        for label, ranges in self.COLOR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(
                        hsv,
                        np.array(lower, dtype=np.uint8),
                        np.array(upper, dtype=np.uint8),
                    ),
                )

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)
            area_ratio = float(cv2.contourArea(contour)) / image_area
            if area_ratio < self.min_color_area_ratio:
                continue

            x, y, width, height = cv2.boundingRect(contour)
            fill_ratio = min(
                1.0,
                float(cv2.contourArea(contour)) / max(1.0, width * height),
            )
            confidence = min(
                1.0,
                0.55 * fill_ratio
                + 0.45 * min(1.0, area_ratio / self.min_color_area_ratio),
            )
            candidate = ColorDetection(
                label=label,
                confidence=confidence,
                area_ratio=area_ratio,
                bbox=(x, y, width, height),
            )
            if best is None or candidate.area_ratio > best.area_ratio:
                best = candidate

        return best

    def _detect_qr(self, image_bgr: np.ndarray) -> Optional[QrDetection]:
        if zbar_decode is not None:
            detections = zbar_decode(
                image_bgr,
                symbols=[ZBarSymbol.QRCODE],
            )
            if detections:
                detection = detections[0]
                data = detection.data.decode("utf-8", errors="replace").strip()
                polygon = [
                    (int(point.x), int(point.y))
                    for point in detection.polygon
                ]
                if len(polygon) < 4:
                    rect = detection.rect
                    polygon = [
                        (rect.left, rect.top),
                        (rect.left + rect.width, rect.top),
                        (rect.left + rect.width, rect.top + rect.height),
                        (rect.left, rect.top + rect.height),
                    ]
                if data:
                    return QrDetection(data=data, bbox=polygon)

        data, points, _ = self.qr_detector.detectAndDecode(image_bgr)
        if not data or points is None:
            return None

        corners = [
            (int(round(point[0])), int(round(point[1])))
            for point in points.reshape((-1, 2))
        ]
        return QrDetection(data=data.strip(), bbox=corners)
