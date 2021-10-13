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
tracked_objects = []
track_count = 0


# Model and publisher initializations.
frame_count = 0
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

img_obj_keys = ['frame_id','object_id','object_type','cur_x','cur_y','prev_x','prev_y']

def subcallback(data):
    global prev_detections, frame_count
    image = rosToCV(data)
    image = CVToTorch(image)

    image = image.to(DEVICE)
    detections = model(image)[0]

    #  rospy.loginfo("Tracking set to " + str(tracking))

    rospy.loginfo("Detected " + str(len(detections["boxes"])) +" objects")

    frame_detections = filterDetections(detections, frame_count) # Array of ImageDetect objects of a single frame
    frame_count+=1
    
    for img_obj_msg in frame_detections:
        publisher.publish(objToDetectMsg(img_obj_msg))
        rospy.loginfo("Published detected object")   
    if tracking:
        trackObjects(frame_detections)
        for track_obj in tracked_objects:
            track_publisher.publish(objToTrackMsg(track_obj))
            rospy.loginfo("Published Tracked Object")       

            



def rosToCV(ros_img):
    return bridge.imgmsg_to_cv2(ros_img, desired_encoding='passthrough')

def CVToTorch(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.transpose((2,0,1))
    
    image = np.expand_dims(image, axis=0)
    image = image / 255.0
    image = torch.FloatTensor(image)
    return image

def objToDetectMsg(img_obj):
    msg = ImageDetect()
    msg.header.frame_id = img_obj['frame_id']
    msg.object_type = img_obj['object_type']
    msg.x = img_obj['cur_x']
    msg.y = img_obj['cur_y']

    return msg

def objToTrackMsg(img_obj):
    msg = ImageTrack()
    msg.header.frame_id = img_obj["frame_id"]
    msg.object_id = str(img_obj["object_id"])
    msg.cur_x = img_obj["cur_x"]
    msg.cur_y = img_obj["cur_y"]
    msg.prev_x = img_obj["prev_x"] if isinstance(img_obj["prev_x"], int) else 0
    msg.prev_y = img_obj["prev_y"] if isinstance(img_obj["prev_y"], int) else 0
 
    return msg


def filterDetections(detections, frame_id):
    filtered_detections = []
    for i in range(len(detections["boxes"])):
        confidence = detections["scores"][i]

        if confidence > 0.5:
            type_idx = int(detections["labels"][i])
            box  = detections["boxes"][i].detach().cpu().numpy()
            (sX, sY, eX, eY) = box.astype("uint")
            
            msg = dict.fromkeys(img_obj_keys)
            msg["frame_id"] = str(frame_id)
            msg["object_type"] = COCO_INSTANCE_CATEGORY_NAMES[type_idx]
            msg["cur_x"] = int((sX+eX)/2)
            
            msg["cur_y"] = int((sY+eY)/2)

            filtered_detections.append(msg)
    return filtered_detections


def trackObjects(cur_detections):
    global tracked_objects, prev_detections
    if len(tracked_objects) == 0:
        for cur_obj in cur_detections:
            AddToTracked(cur_obj)
    else:
        track_established = [False for i in range(len(cur_detections))]
        for tracked in tracked_objects:
            isTracked = False
            for idx,cur in enumerate(cur_detections):
                if tracked["object_type"] == cur["object_type"]:
                    cur_coord = [cur["cur_x"], cur["cur_y"]]
                    prev_coord = [tracked["cur_x"], tracked["cur_y"]]
                    if math.dist(cur_coord, prev_coord) <= track_dist_threshold:
                        tracked["prev_x"] = tracked["cur_x"]
                        tracked["prev_y"] = tracked["cur_y"]
                        tracked["cur_x"] = cur["cur_x"]
                        tracked["cur_y"] = cur["cur_y"]
                        track_established[idx] = True
                        isTracked = True
                        break
            if isTracked is False:
                tracked_objects.remove(tracked)
        #If object detected in current frame was not tracked previously, add it to tracked_objects
        for idx,isTracked in enumerate(track_established):
            if isTracked is False:
                AddToTracked(cur_detections[idx])

                

                    

def AddToTracked(cur_obj):
    global tracked_objects, track_count
    track_obj = dict.fromkeys(img_obj_keys)
    track_obj["frame_id"] = cur_obj["frame_id"]
    track_obj["object_type"] = cur_obj["object_type"]
    track_obj["object_id"] = track_count
    track_obj["cur_x"]= cur_obj["cur_x"]
    track_obj["cur_y"] = cur_obj["cur_x"]
    tracked_objects.append(track_obj)
    track_count +=1







def trackObjectsBtnFrames(prev_detections, cur_detections):
    interframe_objects = []
    for cur_obj in cur_detections:
        for prev_obj in prev_detections:
            if cur_obj.object_type == prev_obj.object_type:
                cur_coord = [cur_obj.x, cur_obj.y]
                prev_coord = [prev_obj.x, prev_obj.y]
                if math.dist(cur_coord, prev_coord) <= track_dist_threshold:
                    msg = dict.fromkeys(img_obj_keys)
                    msg["frame_id"] = cur_obj["frame_id"]
                    msg["cur_x"]= cur_obj["cur_x"]
                    msg["cur_y"] = cur_obj["cur_x"]
                    msg["prev_x"] = prev_obj["cur_x"]
                    msg["prev_y"] = prev_obj["cur_x"]
                    tracked_objects.append(msg)

    return interframe_objects


def main(args):
    rospy.init_node('image_detector')
    rospy.Subscriber("videofile/image_raw", Image, subcallback)

    rospy.spin()

if __name__ == '__main__':
    main(0)
