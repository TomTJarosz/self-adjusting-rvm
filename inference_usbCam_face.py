#!/usr/bin/python
# -*- coding: utf-8 -*-
# pylint: disable=C0103
# pylint: disable=E1101

import sys
import time
import numpy as np
import tensorflow as tf
import cv2
from threading import Thread
from picamera import PiCamera
from picamera.array import PiRGBArray
from utils import label_map_util
from utils import visualization_utils_color as vis_util
cam=PiCamera()
cam.resolution=(1280,720)
raw=PiRGBArray(cam)
cam.capture(raw,format='rgb')
img=raw.array
[h, w] = img.shape[:2]
#h=2*h/3
#w=2*w/3
img=None
raw=None
global image
image=np.empty((4,h/2,w/4,3),dtype=np.uint8)
global out_imgs
out_imgs=np.empty((4,h/2,w/4,3),dtype=np.uint8)
global processed
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = './model/frozen_inference_graph_face.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = './protos/face_label_map.pbtxt'

NUM_CLASSES = 2

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

class get_imgs(Thread):
    def run(self):
        i=0
        while 1:
            raw=PiRGBArray(cam)
            cam.capture(raw,format='rgb')
            ar=cv2.resize(np.array(raw.array),(w/2,h/2))[:,w/4:w/2]
            ar = cv2.cvtColor(ar, cv2.COLOR_BGR2RGB)
            ar=cv2.addWeighted(ar,2,ar,0,20)
            image[i]=ar
            
            ar=None
            raw=None
            print "1 lap of get_imgs"
            time.sleep(.6)
            i=(i+1)%4
            

class classify(Thread):
    def run(self):
        while 1:
            t1=time.time()
            out_imgs=np.copy(image)
            inp=np.empty((h,w/2,3),dtype=np.uint8)
            inp[0:h/2,0:w/4]=out_imgs[0]
            inp[0:h/2,w/4:]=out_imgs[1]
            inp[h/2:,0:w/4]=out_imgs[2]
            inp[h/2:,w/4:]=out_imgs[3]
            (boxes, scores, classes, num_detections) = tDetector.run(inp)
            
            
            vis_util.visualize_boxes_and_labels_on_image_array(
                [inp],
                [np.squeeze(boxes)],
                [np.squeeze(classes).astype(np.int32)],
                [np.squeeze(scores)],
                category_index,
                use_normalized_coordinates=True,
                line_thickness=4)
            imgdic[0]=inp
            print "time per image:"+str((time.time()-t1)/4)
            imgdic['p']=True
            

class TensoflowFaceDector(object):
    def __init__(self, PATH_TO_CKPT):
        """Tensorflow detector
        """

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')


        with self.detection_graph.as_default():
            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            self.sess = tf.Session(graph=self.detection_graph, config=config)
            self.windowNotSet = True


    def run(self, image_np):
        """image: rgb image array
        return (boxes, scores, classes, num_detections)
        """

        image_np=np.expand_dims(image_np,axis=0)

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        # Actual detection.
        start_time = time.time()
        (boxes, scores, classes, num_detections) = self.sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np})
        elapsed_time = time.time() - start_time
        print('inference time cost: {}'.format(elapsed_time))

        return (boxes, scores, classes, num_detections)


if __name__ == "__main__":
    tDetector = TensoflowFaceDector(PATH_TO_CKPT)
    imgdic={'p':False}
    #cap = cv2.VideoCapture(camID)
    print 'starting to get images'
    get_imgs().start()
    time.sleep(4.5)
    print 'starting to classify images'
    classify().start()
    time.sleep(1)
    print 'starting to display images'
    count=0
    cv2.namedWindow("tensorflow based (%d, %d)" % (w/2, h), cv2.WINDOW_NORMAL)
    while imgdic['p']==False:
        time.sleep(2)
    while 1:
        imgdic['p']==False
        cv2.imshow("tensorflow based (%d, %d)" % (w/2, h), imgdic[0])#cv2.resize(imgdic[0][count],(w/2,h)))
        k = cv2.waitKey(1) & 0xff
        if k == ord('q') or k == 27:
            break
        while imgdic['p']==False:
            print 'waiting for images to be processed'
            time.sleep(.5)
    '''
    while True:
        
        ret, image = cap.read()
        if ret == 0:
            break

        [h, w] = image.shape[:2]
        print (h, w)
        image = cv2.flip(image, 1)
        
        raw=PiRGBArray(cam)
        cam.capture(raw,format='rgb')
        image1=raw.array
        image=np.copy(image1)
        [h1, w1] = image1.shape[:2]
        image=np.copy(image1[:,0:(w1/2)])
        [h, w] = image.shape[:2]
        
        t1=time.time()
        for x in xrange(2):
            for y in xrange(2):
                raw=PiRGBArray(cam)
                cam.capture(raw,format='rgb')
                ar=cv2.resize(np.array(raw.array),(w/2,h/2))
                image[h*x/2:h*(x+1)/2,w*y/4:w*(y+1)/4]=ar[:,w/4:w/2]
        print 'total time per image='+str((time.time()-t1)/4)
        print(h, w)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        (boxes, scores, classes, num_detections) = tDetector.run(image)
        out_imgs=np.copy(images)
        vis_util.visualize_boxes_and_labels_on_image_array(
            out_imgs,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=4)

        if windowNotSet is True:
            cv2.namedWindow("tensorflow based (%d, %d)" % (w, h), cv2.WINDOW_NORMAL)
            windowNotSet = False

        cv2.imshow("tensorflow based (%d, %d)" % (w, h), image)
        k = cv2.waitKey(1) & 0xff
        if k == ord('q') or k == 27:
            break

    #cap.release()
    '''