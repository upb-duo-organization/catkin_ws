#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

import cv2
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import time

ENGINE_PATH  = "/home/sleepy/yolo_shoes/yolo_shoes/person_detector.engine"
INPUT_W      = 640
INPUT_H      = 640
CONF_THRESH  = 0.45
NMS_THRESH   = 0.45

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
    "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]
FILTER_CLASSES = {"person"}

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def load_engine(path):
    trt.init_libnvinfer_plugins(TRT_LOGGER, "")
    with open(path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def allocate_buffers(engine):
    inputs, outputs, bindings = [], [], []
    stream = cuda.Stream()
    for binding in engine:
        shape    = engine.get_binding_shape(binding)
        size     = trt.volume(shape)
        dtype    = trt.nptype(engine.get_binding_dtype(binding))
        host_mem = cuda.pagelocked_empty(size, dtype)
        dev_mem  = cuda.mem_alloc(host_mem.nbytes)
        bindings.append(int(dev_mem))
        if engine.binding_is_input(binding):
            inputs.append({"host": host_mem, "device": dev_mem})
        else:
            outputs.append({"host": host_mem, "device": dev_mem})
    return inputs, outputs, bindings, stream

def do_inference(context, inputs, outputs, bindings, stream):
    for inp in inputs:
        cuda.memcpy_htod_async(inp["device"], inp["host"], stream)
    context.execute_async_v2(bindings=bindings, stream_handle=stream.handle)
    for out in outputs:
        cuda.memcpy_dtoh_async(out["host"], out["device"], stream)
    stream.synchronize()
    return [out["host"] for out in outputs]

def preprocess(frame):
    img = cv2.resize(frame, (INPUT_W, INPUT_H))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    return np.ascontiguousarray(img)

def postprocess(raw_output, orig_w, orig_h):
    num_classes   = len(COCO_CLASSES)
    expected_cols = 4 + num_classes
    raw = raw_output.reshape(-1, expected_cols)
    if raw.shape[1] != expected_cols:
        raw = raw_output.reshape(expected_cols, -1).T

    boxes, scores, class_ids = [], [], []
    for det in raw:
        class_scores = det[4:]
        cls_id       = int(np.argmax(class_scores))
        score        = float(class_scores[cls_id])
        if score < CONF_THRESH:
            continue
        if COCO_CLASSES[cls_id] not in FILTER_CLASSES:
            continue
        cx, cy, w, h = det[:4]
        x1 = int((cx - w / 2) * orig_w / INPUT_W)
        y1 = int((cy - h / 2) * orig_h / INPUT_H)
        x2 = int((cx + w / 2) * orig_w / INPUT_W)
        y2 = int((cy + h / 2) * orig_h / INPUT_H)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(orig_w, x2), min(orig_h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(score)
        class_ids.append(cls_id)

    detections = []
    if boxes:
        indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESH, NMS_THRESH)
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                detections.append((x, y, x+w, y+h, scores[i], class_ids[i]))
    return detections

def draw_detections(frame, detections):
    for (x1, y1, x2, y2, conf, cls_id) in detections:
        label = f"{COCO_CLASSES[cls_id]} {conf:.0%}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, y1-th-10), (x1+tw+6, y1), (0, 255, 255), -1)
        cv2.putText(frame, label, (x1+3, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return frame

class TRTNode:
    def __init__(self):
        rospy.init_node("yolo_trt", anonymous=False)

        rospy.loginfo("[YoloTRT] Loading engine...")
        self.engine  = load_engine(ENGINE_PATH)
        self.context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.bindings, self.stream = \
            allocate_buffers(self.engine)
        rospy.loginfo("[YoloTRT] Engine loaded.")

        self.prev_time = time.time()

        self.pub_img = rospy.Publisher(
            "/yolo/image_annotated", Image, queue_size=1)
        self.pub_det = rospy.Publisher(
            "/person_detected", String, queue_size=10)

        rospy.Subscriber(
            "/usb_cam/image_raw", Image,
            self.callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("[YoloTRT] Subscribed to /usb_cam/image_raw")
        rospy.loginfo("[YoloTRT] Ready.")
        rospy.spin()

    def callback(self, msg):
        try:
            # Manual conversion — bypasses cv_bridge Python2/3 conflict
            if msg.encoding in ("bgr8", "rgb8"):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    frame = frame[:, :, ::-1]   # RGB → BGR
            elif msg.encoding in ("yuyv", "yuv422", "YUV422"):
                yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 2)
                frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
            elif msg.encoding == "mono8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                # fallback — try bgr8
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
        except Exception as e:
            rospy.logerr(f"[YoloTRT] Image conversion error: {e}")
            rospy.logerr(f"[YoloTRT] Encoding was: {msg.encoding}")
            return

        orig_h, orig_w = frame.shape[:2]
        inp = preprocess(frame)
        np.copyto(self.inputs[0]["host"], inp.ravel())

        raw_outputs = do_inference(
            self.context, self.inputs,
            self.outputs, self.bindings, self.stream
        )

        detections = postprocess(raw_outputs[0], orig_w, orig_h)
        frame      = draw_detections(frame, detections)

        now            = time.time()
        fps            = 1.0 / (now - self.prev_time + 1e-9)
        self.prev_time = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Publish annotated image manually — also bypasses cv_bridge
        out_msg          = Image()
        out_msg.header   = msg.header
        out_msg.height   = frame.shape[0]
        out_msg.width    = frame.shape[1]
        out_msg.encoding = "bgr8"
        out_msg.step     = frame.shape[1] * 3
        out_msg.data     = frame.tobytes()
        self.pub_img.publish(out_msg)

        if detections:
            msg_out      = String()
            msg_out.data = f"person_detected: {len(detections)} person(s)"
            self.pub_det.publish(msg_out)
            rospy.loginfo_throttle(1.0,
                f"[YoloTRT] {len(detections)} person(s) detected")

if __name__ == "__main__":
    try:
        TRTNode()
    except rospy.ROSInterruptException:
        pass
