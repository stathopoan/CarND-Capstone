import os
from real_model import RealModel


class TLClassifier(object):
    def __init__(self):
        this_file_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.PATH_TO_MODEL = this_file_dir_path+'/models/frozen_inference_graph.pb'
        self.model = RealModel(self.PATH_TO_MODEL)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # DONE implement light color prediction
        # return TrafficLight.RED  # hard-wired to return red
        return self.model.predict(image)


