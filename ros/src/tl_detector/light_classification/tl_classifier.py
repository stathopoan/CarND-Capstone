from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
from real_model import RealModel

class TLClassifier(object):
    def __init__(self):
        self.PATH_TO_MODEL = 'models/frozen_inference_graph.pb'
        # TODO Aruul fix error in line below
        # self.model = RealModel(self.PATH_TO_MODEL)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # DONE implement light color prediction
        # return self.model.predict(image)

        return TrafficLight.RED  # hard-wired to return red
