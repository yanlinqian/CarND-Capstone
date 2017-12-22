from styx_msgs.msg import TrafficLight
import rospy
import tensorflow as tf
import numpy as np
import cv2
CNN_MODEL_PATH='light_classification/ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb'
#'ssd_mobilenet_v1_coco_2017_11_17' or 'faster_rcnn_resnet101_coco_2017_11_08' downloaded from
#https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md

def tfgraph_load(graph_path):
    """
    load a tensorflow model
    """
    graph = tf.Graph()
    with graph.as_default():
        graph_def = tf.GraphDef()
        with tf.gfile.GFile(CNN_MODEL_PATH, 'rb') as fid:
            string_graph = fid.read()
            graph_def.ParseFromString(string_graph)
            tf.import_graph_def(graph_def, name='')
    return graph


try:
    cnn = tfgraph_load(CNN_MODEL_PATH)
    image_tensor = cnn.get_tensor_by_name('image_tensor:0')
    bboxs = cnn.get_tensor_by_name('detection_boxes:0')
    scores = cnn.get_tensor_by_name('detection_scores:0')
    classes = cnn.get_tensor_by_name('detection_classes:0')
    sess = tf.Session(graph=cnn)
    rospy.logerr('tl_classifier, load graph and make session')
except Exception as e:
    rospy.logerr('tl_classifier, error in load graph and make session')

def general_detect(sess,image):
    """use pretrained faster_rcnn resnet101 to get general detection results"""
    img=np.expand_dims(np.asarray(image,dtype=np.uint8),0)
    (img_boxes,img_scores,img_classes)=sess.run([bboxs,scores,classes],
                                                feed_dict={image_tensor:img})
    return np.squeeze(img_boxes),np.squeeze(img_scores),np.squeeze(img_classes)

def threshold_boxes(threshold,boxes,scores,classes):
    """get bounding boxes over threshold"""
    id_over_threshold=[]
    for i in range(len(classes)):
        if scores[i]>=threshold and classes[i]==10:
            id_over_threshold.append(i)
    return boxes[id_over_threshold,...]

def get_image_incets(im,boxes):
    """use boxes to get incets in im"""
    images=[]
    w,h,_=im.shape
    for box in boxes:
        x1,y1,x2,y2=np.uint16(np.round(h*box[1])),np.uint16(np.round(w*box[0])),\
                    np.uint16(np.round(h*box[3])),np.uint16(np.round(w*box[2]))
        images.append(im[y1:y2,x1:x2])
    return images

def light_detect_incet(image):
    """detect one light according to a small image which likely covers only a traffic light"""
    light_color=[]
    h,w,_=image.shape

    red_thresh=cv2.inRange(image[0:np.int16(h/3),:,:],np.array([20,20,240]),np.array([60,60,255]))
    yellow_thresh=cv2.inRange(image[np.int16(h/3):np.int16(2*h/3),:,:],np.array([20,200,200]),np.array([60,255,255]))
    green_thresh=cv2.inRange(image[np.int16(2*h/3):h,:,:],np.array([40,215,20]),np.array([100,255,60]))

    light_color.append(np.sum(red_thresh[:]))
    light_color.append(np.sum(yellow_thresh[:]))
    light_color.append(np.sum(green_thresh[:]))

    # cv2.imwrite('red_thresh.png',red_thresh)
    # cv2.imwrite('yellow_thresh.png', yellow_thresh)
    # cv2.imwrite('green_thresh.png', green_thresh)

    if np.max(light_color)==0:
        return 'GREEN'

    light_states={0:'RED',1:'YELLOW',2:'GREEN'}
    return light_states[np.argmax(light_color)]

def light_detect_image(sess,image,threshold):
    """detect all traffic light in a full resolution image"""
    img_boxes,img_scores,img_classes=general_detect(sess, image)
    img_boxes=threshold_boxes(threshold,img_boxes,img_scores,img_classes)
    img_incets=get_image_incets(image,img_boxes)
    # i=0
    # for incet in img_incets:
    #     cv2.imwrite('firstincet%d.png'%i,incet)
    #     i=i+1
    #rospy.logerr('tl_classifer,len incets:%d'%(len(img_incets)))
    lights=[]
    for incet in img_incets:
        lights.append(light_detect_incet(incet))

    return lights

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        try:
            #cv2.imwrite('firstimage.png',image) #works
            lights=light_detect_image(sess,image,0.3)

            for l in lights:
                if l in ['RED']:  # treat yellow lights are red to be on the safe side
                    return TrafficLight.RED
        except Exception as e:
            rospy.logerr('tl_classifier, error in TLClassifier.get_classfication')
        return TrafficLight.UNKNOWN
