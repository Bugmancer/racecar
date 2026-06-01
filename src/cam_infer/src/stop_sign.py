import cv2
import numpy as np
import ipywidgets as widgets
from IPython.display import display
import torch
from skvideo.io import vreader, FFmpegWriter
import IPython.display
from ais_bench.infer.interface import InferSession
from det_utils import letterbox, scale_coords, nms

import rospy
from std_msgs.msg import String

light_color = 0

def preprocess_image(image, cfg, bgr2rgb=True):
    """图片预处理"""
    img, scale_ratio, pad_size = letterbox(image, new_shape=cfg['input_shape'])
    if bgr2rgb:
        img = img[:, :, ::-1]
    img = img.transpose(2, 0, 1)  # HWC2CHW
    img = np.ascontiguousarray(img, dtype=np.float16) / 255.0
    return img, scale_ratio, pad_size


def draw_bbox(bbox, img0, color, wt, names):
    """在图片上画预测框"""
    det_result_str = ''
    for idx, class_id in enumerate(bbox[:, 5]):
        if float(bbox[idx][4] < float(0.05)):
            continue
        print(names[int(class_id)])
        light_color = names[int(class_id)]
        img0 = cv2.rectangle(img0, (int(bbox[idx][0]), int(bbox[idx][1])), (int(bbox[idx][2]), int(bbox[idx][3])),
                             color, wt)
        img0 = cv2.putText(img0, str(idx) + ' ' + names[int(class_id)], (int(bbox[idx][0]), int(bbox[idx][1] + 16)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        img0 = cv2.putText(img0, '{:.4f}'.format(bbox[idx][4]), (int(bbox[idx][0]), int(bbox[idx][1] + 32)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        det_result_str += '{} {} {} {} {} {}\n'.format(
            names[bbox[idx][5]], str(bbox[idx][4]), bbox[idx][0], bbox[idx][1], bbox[idx][2], bbox[idx][3])
    return img0


def get_labels_from_txt(path):
    """从txt文件获取图片标签"""
    labels_dict = dict()
    with open(path) as f:
        for cat_id, label in enumerate(f.readlines()):
            labels_dict[cat_id] = label.strip()
    return labels_dict


def draw_prediction(pred, image, labels):
    """在图片上画出预测框并进行可视化展示"""
    imgbox = widgets.Image(format='jpg', height=720, width=1280)
    img_dw = draw_bbox(pred, image, (0, 255, 0), 2, labels)
    imgbox.value = cv2.imencode('.jpg', img_dw)[1].tobytes()
    display(imgbox)


def infer_image(img_path, model, class_names, cfg):
    """图片推理"""
    # 图片载入
    image = cv2.imread(img_path)
    # 数据预处理
    img, scale_ratio, pad_size = preprocess_image(image, cfg)
    # 模型推理
    output = model.infer([img])[0]

    output = torch.tensor(output)
    # 非极大值抑制后处理
    boxout = nms(output, conf_thres=cfg["conf_thres"], iou_thres=cfg["iou_thres"])
    pred_all = boxout[0].numpy()
    # 预测坐标转换
    scale_coords(cfg['input_shape'], pred_all[:, :4], image.shape, ratio_pad=(scale_ratio, pad_size))
    # 图片预测结果可视化
    draw_prediction(pred_all, image, class_names)


def infer_frame_with_vis(image, model, labels_dict, cfg, bgr2rgb=True):
    # 数据预处理
    img, scale_ratio, pad_size = preprocess_image(image, cfg, bgr2rgb)
    # 模型推理
    output = model.infer([img])[0]

    output = torch.tensor(output)
    # 非极大值抑制后处理
    boxout = nms(output, conf_thres=cfg["conf_thres"], iou_thres=cfg["iou_thres"])
    pred_all = boxout[0].numpy()
    # 预测坐标转换
    scale_coords(cfg['input_shape'], pred_all[:, :4], image.shape, ratio_pad=(scale_ratio, pad_size))

    # 图片预测结果可视化
    img_vis = draw_bbox(pred_all, image, (0, 255, 0), 2, labels_dict)
    return img_vis


def img2bytes(image):
    """将图片转换为字节码"""
    return bytes(cv2.imencode('.jpg', image)[1])



def infer_camera(model, labels_dict, cfg):
    """外设摄像头实时推理"""
    def find_camera_index():
        max_index_to_check = 10  # Maximum index to check for camera

        for index in range(max_index_to_check):
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                cap.release()
                return index

        # If no camera is found
        raise ValueError("No camera found.")

    # 获取摄像头
    camera_index = find_camera_index()
    cap = cv2.VideoCapture(camera_index)
    # 初始化可视化对象
    image_widget = widgets.Image(format='jpeg', width=1280, height=720)
    display(image_widget)
    while True:
        # 对摄像头每一帧进行推理和可视化
        _, img_frame = cap.read()

        image_pred = infer_frame_with_vis(img_frame, model, labels_dict, cfg)
        image_widget.value = img2bytes(image_pred)


cfg = {
    'conf_thres': 0.6,  # 模型置信度阈值，阈值越低，得到的预测框越多
    'iou_thres': 0.5,  # IOU阈值，高于这个阈值的重叠预测框会被过滤掉
    'input_shape': [640, 640],  # 模型输入尺寸
}

def talker():
     pub = rospy.Publisher('chatter', String, queue_size=10)
     rospy.init_node('sign_talker', anonymous=True)
     rate = rospy.Rate(10) # 10hz
     while not rospy.is_shutdown():
        if(light_color == 'A'):
            hello_str = "stop"
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()    
        

if __name__ == '__main__':
    

    model_path = 'yolov5s_bs1.om'
    label_path = './coco_names_sign.txt'
    # 初始化推理模型
    model = InferSession(0, model_path)
    labels_dict = get_labels_from_txt(label_path)

    infer_mode = 'camera'
    infer_camera(model, labels_dict, cfg)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass