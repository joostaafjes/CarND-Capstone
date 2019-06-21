
from styx_msgs.msg import TrafficLight

# removed all Joosts LENEt style code

class TLClassifier(object):
    def __init__(self):
       #TODO load classifier
       pass


        
    def get_classification(self, image): # image): #changed for Joost code
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        return TrafficLight.UNKNOWN
   
 