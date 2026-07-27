"""Clean YOLOv8 object-detection baseline for the Yahboom CanMV K230.

Run this file manually from CanMV IDE.  It intentionally contains no UART
code and must not be saved as /sdcard/main.py while it is being evaluated.

Required files:
  /sdcard/libs/
  /sdcard/kmodel/steel_ball_v6_320.kmodel
"""

from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *

from media.media import *
import aidemo
import gc
import nncase_runtime as nn
import os
import sys
import ulab.numpy as np


DISPLAY_MODE = "lcd"
DISPLAY_SIZE = [640, 480]
RGB888P_SIZE = [320, 320]
MODEL_INPUT_SIZE = [320, 320]
KMODEL_PATH = "/sdcard/kmodel/steel_ball_v6_320.kmodel"
CONFIDENCE_THRESHOLD = 0.15
NMS_THRESHOLD = 0.40
MAX_BOXES_NUM = 30
DEBUG_MODE = 0

LABELS = [
    "steel ball"
]


class ObjectDetectionApp(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                 confidence_threshold=0.5, nms_threshold=0.2,
                 rgb888p_size=(224, 224), display_size=(640, 480),
                 debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.labels = labels
        self.model_input_size = list(model_input_size)
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]
        self.debug_mode = debug_mode
        self.colors = get_colors(len(labels))

        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size or self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )
            self.ai2d.pad(
                [0, 0, 0, 0, top, bottom, left, right],
                0,
                [128, 128, 128],
            )
            self.ai2d.resize(
                nn.interp_method.tf_bilinear,
                nn.interp_mode.half_pixel,
            )
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def preprocess(self, input_np):
        with ScopedTiming("preprocess", self.debug_mode > 0):
            return [nn.from_numpy(input_np)]

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            output = results[0][0].transpose()
            return aidemo.yolov8_det_postprocess(
                output.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num,
            )

    def draw_result(self, pipeline, detections):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            pipeline.osd_img.clear()
            if not detections:
                return

            boxes, class_ids, scores = detections
            for index in range(len(boxes)):
                x, y, width, height = map(
                    lambda value: int(round(value, 0)), boxes[index]
                )
                class_id = class_ids[index]
                color = self.colors[class_id]
                label = " {} {:.2f}".format(
                    self.labels[class_id], scores[index]
                )
                pipeline.osd_img.draw_rectangle(
                    x, y, width, height, color=color, thickness=4
                )
                pipeline.osd_img.draw_string_advanced(
                    x, max(0, y - 40), 32, label, color=color
                )


def require_file(path):
    try:
        os.stat(path)
    except Exception:
        raise RuntimeError("required file is missing: " + path)


def print_exception(error):
    if hasattr(sys, "print_exception"):
        sys.print_exception(error)
    else:
        print(error)


def main():
    pipeline = None
    detector = None

    try:
        require_file(KMODEL_PATH)
        print("[YOLO] clean LCD baseline starting")
        print("[YOLO] model:", KMODEL_PATH)

        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            display_mode=DISPLAY_MODE,
        )
        pipeline.create()

        detector = ObjectDetectionApp(
            KMODEL_PATH,
            labels=LABELS,
            model_input_size=MODEL_INPUT_SIZE,
            max_boxes_num=MAX_BOXES_NUM,
            confidence_threshold=CONFIDENCE_THRESHOLD,
            nms_threshold=NMS_THRESHOLD,
            rgb888p_size=RGB888P_SIZE,
            display_size=DISPLAY_SIZE,
            debug_mode=DEBUG_MODE,
        )
        detector.config_preprocess()

        while True:
            with ScopedTiming("total", DEBUG_MODE > 0):
                frame = pipeline.get_frame()
                detections = detector.run(frame)
                detector.draw_result(pipeline, detections)
                pipeline.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("[YOLO] stopped by IDE")
    except Exception as error:
        # CanMV IDE reports its Stop button as a generic "IDE interrupt"
        # exception on some firmware versions.  Treat that as a normal stop so
        # cleanup still runs without showing a misleading failure popup.
        if "IDE interrupt" in str(error):
            print("[YOLO] stopped by IDE")
        else:
            print("[YOLO] fatal error")
            print_exception(error)
            raise
    finally:
        if detector is not None:
            try:
                detector.deinit()
            except Exception as error:
                print("[YOLO] detector cleanup failed:", error)
        if pipeline is not None:
            try:
                pipeline.destroy()
            except Exception as error:
                print("[YOLO] pipeline cleanup failed:", error)
        gc.collect()
        print("[YOLO] resources released")


if __name__ == "__main__":
    main()
