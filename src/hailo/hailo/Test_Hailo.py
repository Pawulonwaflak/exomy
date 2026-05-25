"""
YOLOv8 + Hailo-8 — inferencja na żywo z kamery ROS2
=====================================================
HailoRT 4.23 | Ubuntu 24.04 | RPi5 | ROS2
Model z wbudowanym NMS: yolov8s/yolov8_nms_postprocess

"""

import argparse
import threading
import time
import numpy as np
import cv2
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                        QoSReliabilityPolicy, QoSHistoryPolicy)

from hailo_platform import (
    HEF, VDevice, HailoStreamInterface,
    InferVStreams, ConfigureParams,
    InputVStreamParams, OutputVStreamParams,
    FormatType,
)

# ──────────────────────────────────────────────────────────────
# COCO labels
# ──────────────────────────────────────────────────────────────
COCO_LABELS = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush",
]
COLORS = np.random.default_rng(42).integers(0, 255, (80, 3), dtype=np.uint8)


# ──────────────────────────────────────────────────────────────
# QoS matching (identyczny jak w Test_Server.py)
# ──────────────────────────────────────────────────────────────
def get_matched_qos(node: Node, topic: str, timeout_sec: float = 10.0) -> QoSProfile:
    deadline = node.get_clock().now().nanoseconds / 1e9 + timeout_sec
    while node.get_clock().now().nanoseconds / 1e9 < deadline:
        publishers = node.get_publishers_info_by_topic(topic)
        if publishers:
            pub_qos = publishers[0].qos_profile
            depth = pub_qos.depth if pub_qos.depth > 0 else 1
            node.get_logger().info(
                f"[QoS] reliability={pub_qos.reliability}, "
                f"durability={pub_qos.durability}, depth={depth}"
            )
            return QoSProfile(
                reliability=pub_qos.reliability,
                durability=pub_qos.durability,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=depth,
            )
        time.sleep(0.5)
    node.get_logger().warn(f"[QoS] Brak publishera. Fallback RELIABLE.")
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


# ──────────────────────────────────────────────────────────────
# Thread-safe bufor
# ──────────────────────────────────────────────────────────────
class FrameBuffer:
    def __init__(self):
        self._frame = None
        self._lock  = threading.Lock()

    def put(self, frame: np.ndarray):
        with self._lock:
            self._frame = frame

    def get(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


# ──────────────────────────────────────────────────────────────
# ROS2 node
# ──────────────────────────────────────────────────────────────
TOPIC_COMPRESSED = '/camera/image_raw/compressed'
TOPIC_RAW        = '/camera/image_raw'

class CameraSubscriber(Node):
    def __init__(self, buf: FrameBuffer):
        super().__init__('hailo_yolo_bridge')
        self.buf    = buf
        self._first = False
        self._keepalive = self.create_subscription(
            Image, TOPIC_RAW, lambda msg: None, 1)
        self.get_logger().info("[Keepalive] /camera/image_raw aktywna.")
        qos = get_matched_qos(self, TOPIC_COMPRESSED)
        self._sub = self.create_subscription(
            CompressedImage, TOPIC_COMPRESSED, self._cb, qos)
        self.get_logger().info("Subskrypcja /compressed aktywna.")

    def _cb(self, msg: CompressedImage):
        data  = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is not None:
            self.buf.put(frame)
            if not self._first:
                self.get_logger().info("[OK] Pierwsza klatka odebrana!")
                self._first = True

def ros2_spin(buf: FrameBuffer):
    rclpy.init()
    node = CameraSubscriber(buf)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ──────────────────────────────────────────────────────────────
# Post-processing — format NMS (yolov8_nms_postprocess)
#
# Hailo NMS zwraca listę per obraz, każdy element to lista detekcji.
# Każda detekcja: [y_min, x_min, y_max, x_max, score, class_id]
# Współrzędne są znormalizowane 0..1 względem rozmiaru wejścia modelu.
# ──────────────────────────────────────────────────────────────
def decode_nms_output(raw_outputs, orig_w, orig_h, conf_thresh=0.40):
    results = []
    for out_name, batched in raw_outputs.items():
        # batched[0] = lista 80 klas, każda to ndarray (N, 5)
        # kolumny: [y1, x1, y2, x2, score]  — znormalizowane 0..1
        per_class = batched[0]
        for cls_id, dets in enumerate(per_class):
            if dets is None or len(dets) == 0:
                continue
            for det in dets:
                score = float(det[4])
                if score < conf_thresh:
                    continue
                y1 = int(det[0] * orig_h)
                x1 = int(det[1] * orig_w)
                y2 = int(det[2] * orig_h)
                x2 = int(det[3] * orig_w)
                results.append({
                    "box":   [x1, y1, x2, y2],
                    "score": score,
                    "class": cls_id,
                    "label": COCO_LABELS[cls_id] if cls_id < len(COCO_LABELS) else f"cls{cls_id}",
                })
    return results

# ──────────────────────────────────────────────────────────────
# Rysowanie
# ──────────────────────────────────────────────────────────────
def draw_detections(frame: np.ndarray, detections: list) -> np.ndarray:
    for d in detections:
        x1, y1, x2, y2 = d["box"]
        cls = d["class"]
        col = tuple(int(c) for c in COLORS[cls % 80])
        lbl = f"{d['label']} {d['score']:.0%}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), col, -1)
        cv2.putText(frame, lbl, (x1+2, y1-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return frame


# ──────────────────────────────────────────────────────────────
# Hailo — inicjalizacja (HailoRT 4.23)
# ──────────────────────────────────────────────────────────────
def build_hailo_pipeline(hef_path: str):
    hef    = HEF(hef_path)
    target = VDevice()
    cfg    = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    ngs    = target.configure(hef, cfg)
    ng     = ngs[0]
    ng_p   = ng.create_params()
    in_vsp  = InputVStreamParams.make(ng,  format_type=FormatType.FLOAT32)
    out_vsp = OutputVStreamParams.make(ng, format_type=FormatType.FLOAT32)

    in_info   = hef.get_input_vstream_infos()[0]
    out_infos = hef.get_output_vstream_infos()
    shape     = in_info.shape
    model_h   = shape[0] if len(shape) == 3 else shape[1]
    model_w   = shape[1] if len(shape) == 3 else shape[2]

    print(f"[Hailo] Input  : {in_info.name}  shape={shape}")
    print(f"[Hailo] Outputs: {[o.name for o in out_infos]}")
    print(f"[Hailo] Rozdzielczość modelu: {model_w}x{model_h}")

    return target, ng, ng_p, in_vsp, out_vsp, in_info.name, model_w, model_h


# ──────────────────────────────────────────────────────────────
# Główna pętla
# ──────────────────────────────────────────────────────────────
def run_inference(hef_path: str, buf: FrameBuffer, conf: float, **_):
    target, ng, ng_p, in_vsp, out_vsp, in_name, mw, mh = \
        build_hailo_pipeline(hef_path)
    wh = (mw, mh)

    with InferVStreams(ng, in_vsp, out_vsp) as pipeline:
        with ng.activate(ng_p):
            print("[Hailo] Pipeline aktywny. Naciśnij Q aby wyjść.")
            fps_t0 = time.time(); fps_n = 0

            while True:
                frame = buf.get()
                if frame is None:
                    time.sleep(0.01); continue

                oh, ow = frame.shape[:2]

                # Preprocessing
                inp = cv2.resize(frame, wh)
                inp = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB)  # uint8, zakres 0-255
                inp = np.expand_dims(inp, 0)                 # (1, 640, 640, 3) uint8

                # Inferencja
                raw = pipeline.infer({in_name: inp})

                if not hasattr(run_inference, '_dbg'):
                    run_inference._dbg = True
                    per_class = list(raw.values())[0][0]   # lista 80 klas
                    for cls_id, dets in enumerate(per_class):
                        if len(dets) > 0:
                            print(f"[DBG] cls={cls_id} ({COCO_LABELS[cls_id]})  n={len(dets)}  sample={dets[0]}")
                    total = sum(len(d) for d in per_class)
                    print(f"[DBG] Łącznie detekcji na tej klatce: {total}")
                                

                # Post-processing — model ma wbudowany NMS
                dets = decode_nms_output(raw, orig_w=ow, orig_h=oh, conf_thresh=conf)

                # FPS
                fps_n += 1
                el = time.time() - fps_t0
                if el >= 1.0:
                    fps = fps_n/el; fps_n = 0; fps_t0 = time.time()
                else:
                    fps = fps_n / max(el, 1e-6)

                # Wyświetl
                vis = draw_detections(frame.copy(), dets)
                cv2.putText(vis, f"FPS:{fps:.1f} Det:{len(dets)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                            (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("YOLOv8 + Hailo-8", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[Exit] Q naciśnięte.")
                    break

    cv2.destroyAllWindows()
    target.release()


# ──────────────────────────────────────────────────────────────
# Entrypoint
# ──────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="YOLOv8 + Hailo-8 | ROS2 → cv2.imshow  [HailoRT 4.23 NMS]")
    ap.add_argument("--hef", default=os.path.join(os.path.dirname("/home/exomy/ros2_ws/src/hailo/hailo/Test_Hailo.py"), "yolov8s.hef"))
    ap.add_argument("--conf", type=float, default=0.40)
    ap.add_argument("--input-size", type=int, default=640)
    args = ap.parse_args()

    buf = FrameBuffer()
    threading.Thread(target=ros2_spin, args=(buf,), daemon=True).start()

    print(f"[Start] Model: {args.hef}")
    print("[Start] Czekam na pierwszą klatkę z ROS2 (max 30s)...")
    for _ in range(300):
        if buf.get() is not None:
            break
        time.sleep(0.1)
    else:
        print("[BŁĄD] Brak klatek po 30s.")
        print(f"  Sprawdź: ros2 topic echo {TOPIC_COMPRESSED} --no-arr")
        return

    run_inference(args.hef, buf, args.conf)

if __name__ == '__main__':
    main()