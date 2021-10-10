#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ava_perception.msg import ImageDetect, ImageTrack
from torchvision.models import detection
import cv2
import torch
import numpy as np
import math
bridge = CvBridge()



# Tracker initializations
tracking = rospy.get_param('~tracking', True)
track_publisher = rospy.Publisher('tracked_objects', ImageTrack, queue_size=10)
track_dist_threshold = 20
prev_detections = [] # Previous frame's frame_detections array for tracking

# Model and publisher initializations.
publisher = rospy.Publisher('object_cords', ImageDetect, queue_size=10)
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = detection.fasterrcnn_mobilenet_v3_large_fpn(pretrained=True).to(DEVICE)
model.eval()
rospy.loginfo("Model Initialized")


COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

def subcallback(data):
    global prev_detections
    image = rosToCV(data)
    image = CVToTorch(image)

    image = image.to(DEVICE)
    detections = model(image)[0]

    #  rospy.loginfo("Tracking set to " + str(tracking))

    rospy.loginfo("Detected " + str(len(detections["boxes"])) +" objects")

    frame_detections = filterDetections(detections, 0) # Array of ImageDetect objects of a single frame
    
    if tracking == False:
        for img_obj_msg in frame_detections:
            publisher.publish(img_obj_msg)
            rospy.loginfo("Published object")   
    else:
        if len(prev_detections) > 0:
            tracked_objects = trackObjects(prev_detections, frame_detections)
            for tracked_object_msg in tracked_objects:
                track_publisher.publish(tracked_object_msg)
                rospy.loginfo("Published Tracked Object")
        prev_detections = frame_detections
            


    return None

def rosToCV(ros_img):
    return bridge.imgmsg_to_cv2(ros_img, desired_encoding='passthrough')

def CVToTorch(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.transpose((2,0,1))
    
    image = np.expand_dims(image, axis=0)
    image = image / 255.0
    image = torch.FloatTensor(image)
    return image

def filterDetections(detections, frame_id):
    filtered_detections = []
    for i in range(len(detections["boxes"])):
        confidence = detections["scores"][i]

        if confidence > 0.5:
            type_idx = int(detections["labels"][i])
            box  = detections["boxes"][i].detach().cpu().numpy()
            (sX, sY, eX, eY) = box.astype("uint")

            msg = ImageDetect()
            msg.frame_id = str(frame_id)
            msg.object_type = COCO_INSTANCE_CATEGORY_NAMES[type_idx]
            msg.x = int((sX+eX)/2)
            msg.y = int((sY+eY)/2)
            filtered_detections.append(msg)
    return filtered_detections


def trackObjects(prev_detections, cur_detections):
    tracked_objects = []
    for cur_obj in cur_detections:
        for prev_obj in prev_detections:
            if cur_obj.object_type == prev_obj.object_type:
                cur_coord = [cur_obj.x, cur_obj.y]
                prev_coord = [prev_obj.x, prev_obj.y]
                if math.dist(cur_coord, prev_coord) <= track_dist_threshold:
                    msg = ImageTrack()
                    msg.frame_id = cur_obj.frame_id
                    msg.object_type = cur_obj.object_type
                    msg.cur_x = cur_obj.x
                    msg.cur_y = cur_obj.y
                    msg.prev_x = prev_obj.x
                    msg.prev_y = prev_obj.y
                    tracked_objects.append(msg)

    return tracked_objects


def main(args):
    rospy.init_node('image_detector')
    rospy.Subscriber("videofile/image_raw", Image, subcallback)

    rospy.spin()

if __name__ == '__main__':
    main(0)
