#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import torch
from ais_bench.infer.interface import InferSession
from std_msgs.msg import String

from det_utils import letterbox, nms, scale_coords


def preprocess_image(image, cfg, bgr2rgb=True):
    img, scale_ratio, pad_size = letterbox(image, new_shape=cfg["input_shape"])
    if bgr2rgb:
        img = img[:, :, ::-1]
    img = img.transpose(2, 0, 1)
    img = np.ascontiguousarray(img, dtype=np.float16) / 255.0
    return img, scale_ratio, pad_size


def get_labels_from_txt(path):
    labels = {}
    with open(path) as f:
        for class_id, label in enumerate(f.readlines()):
            labels[class_id] = label.strip()
    return labels


def infer_frame(image, model, labels, cfg):
    img, scale_ratio, pad_size = preprocess_image(image, cfg)
    output = model.infer([img])[0]
    output = torch.tensor(output)
    boxout = nms(output, conf_thres=cfg["conf_thres"], iou_thres=cfg["iou_thres"])
    pred_all = boxout[0].numpy()

    detected = ""
    if pred_all.size == 0:
        return image, detected

    scale_coords(cfg["input_shape"], pred_all[:, :4], image.shape, ratio_pad=(scale_ratio, pad_size))

    best_conf = 0.0
    for bbox in pred_all:
        conf = float(bbox[4])
        if conf < cfg["publish_thres"] or conf < best_conf:
            continue
        class_id = int(bbox[5])
        detected = labels.get(class_id, "")
        best_conf = conf

        x1, y1, x2, y2 = [int(v) for v in bbox[:4]]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, f"{detected} {conf:.2f}", (x1, y1 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return image, detected


def find_camera_index(max_index=10):
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        ok, _ = cap.read()
        cap.release()
        if ok:
            return index
    raise RuntimeError("No camera found")


def main():
    rospy.init_node("traffic_light")

    model_path = rospy.get_param("~model_path", "yolov5s_bs1.om")
    label_path = rospy.get_param("~label_path", "./coco_names_light.txt")
    camera_index = rospy.get_param("~camera_index", -1)
    show_window = rospy.get_param("~show_window", False)

    cfg = {
        "conf_thres": rospy.get_param("~conf_thres", 0.6),
        "publish_thres": rospy.get_param("~publish_thres", 0.6),
        "iou_thres": rospy.get_param("~iou_thres", 0.5),
        "input_shape": [640, 640],
    }

    if camera_index < 0:
        camera_index = find_camera_index()

    model = InferSession(0, model_path)
    labels = get_labels_from_txt(label_path)
    pub = rospy.Publisher("/light", String, queue_size=10)

    cap = cv2.VideoCapture(camera_index)
    rate = rospy.Rate(10)
    last_detected = ""

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok:
            rate.sleep()
            continue

        frame, detected = infer_frame(frame, model, labels, cfg)
        if detected:
            last_detected = detected
            pub.publish(detected)
        elif last_detected:
            pub.publish(last_detected)

        if show_window:
            cv2.imshow("traffic_light", frame)
            cv2.waitKey(1)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
