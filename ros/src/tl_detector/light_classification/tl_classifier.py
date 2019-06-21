from styx_msgs.msg import TrafficLight

# from Joost
import numpy as np
from keras.models import load_model
from keras.preprocessing.image import img_to_array, load_img
import time
## Joost above

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
       # pass
        #class_model = load_model('model'+ '/model.h5')#Joosts # maybe self was probelm Joosts did not have self
        #self.class_model = load_model('model/model.h5')
        #self.class_model = load_model('/model/model.h5')
        class_model = load_model('/model.h5')

    def get_classification(self, image_name): # image): #changed for Joost code
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        #return TrafficLight.UNKNOWN
        
        #def predict(image_name):
        img = load_img(image_name, False, target_size=(32, 32), color_mode='rgb')
        x = img_to_array(img)
        x = np.expand_dims(x, axis=0)
        preds = class_model.predict_classes(x)# I added self then removed it
        #prob = class_model.predict_proba(x) #do we need this? I think state 3 threshold os enough ? John
        #print(preds, prob)
        
        return preds

    
'''
below was in Joosts classify.py file and used for testing only?
start = time.time()
predict('images-resized/086-0-yellow.png.jpeg')
elapsed = time.time() - start
print('elapsed time:', elapsed, ' ms')

predict('images-resized/067-2-green.png.jpeg')
predict('images-resized/019-0-red.png.jpeg')
'''
