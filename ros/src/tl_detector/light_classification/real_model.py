import tensorflow as tf
import os
from styx_msgs.msg import TrafficLight
import numpy as np


class RealModel(object):
    def __init__(self, path):
        self.path = path
        self.prob_thr = 0.5
        self.RED = 2
        self.YELLOW = 7
        self.GREEN = 1
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(graph=self.detection_graph, config=config)

    def predict(self, img):
        detection = TrafficLight.UNKNOWN
        with self.detection_graph.as_default():
            img_expanded = np.expand_dims(img, axis=0)
            (boxes, scores, classes, num) = self.sess.run(
                [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                feed_dict={self.image_tensor: img_expanded})
            scores = scores.squeeze()
            classes = classes.squeeze()
            best = 0
            for i in range(boxes.shape[0]):
                if scores[i] > self.prob_thr:
                    if scores[i] > best:
                        sign = classes[i]
                        best = scores[i]
            if sign == self.GREEN:
                detection = TrafficLight.GREEN
            elif sign == self.RED:
                detection = TrafficLight.RED
            elif sign == self.YELLOW:
                detection = TrafficLight.YELLOW
            return detection



