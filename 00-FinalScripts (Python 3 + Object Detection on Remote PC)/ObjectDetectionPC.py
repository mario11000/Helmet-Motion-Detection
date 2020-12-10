######## Image Object Detection Using Tensorflow-trained Classifier #########
#
# Author: Evan Juras
# Date: 1/15/18
# Description: 
# This program uses a TensorFlow-trained classifier to perform object detection.
# It loads the classifier uses it to perform object detection on an image.
# It draws boxes and scores around the objects of interest in the image.

## Some of the code is copied from Google's example at
## https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

## and some is copied from Dat Tran's example at
## https://github.com/datitran/object_detector_app/blob/master/object_detection_app.py

## but I changed it to make it more understandable to me.

# Import packages
import os
import cv2
import numpy as np
import tensorflow as tf
import sys
import time
import socket
from PIL import Image
from threading import Thread

global old_path
#Path to save the image
path1 = "C:\\Users\\Mariane\\Desktop\\Projectfinal\\Feed1"
path2 = "C:\\Users\\Mariane\\Desktop\\Projectfinal\\Feed2"
global outfile

#Function to delete all files in a directory
def DeleteFiles():
    global old_path
    for i in os.listdir(old_path):
         
        full_path = os.path.join(old_path, i)
        if os.path.isfile(full_path):
            os.remove(full_path)

old_path = path1
DeleteFiles()

old_path = path2
DeleteFiles()

#Defining a socket
s=socket.socket()

#Reserve a port on your computer in our case it is 1234 but it can be anything
port=8000

# Next bind to the port we have not typed any ip in the ip field 
# instead we have inputted an empty string this makes the server listen to requests coming from other computers on the network 
s.bind(('', port))         
print("socket binded to " +str(port)) 
  
# put the socket into listening mode 
s.listen(1)      
print("socket is listening")            
  
# Establish connection with client.
c, addr = s.accept()
#print('Got connection from '+str(addr))
   
# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")

# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

# Path to frozen detection graph .pb file, which contains the model that is used for object detection.
PATH_TO_CKPT = 'C:\\tensorflow1\\models\\research\\object_detection\\faster_rcnn_resnet50_coco_2018_01_28\\frozen_inference_graph.pb'

# Path to label map file
PATH_TO_LABELS = 'C:\\tensorflow1\\models\\research\\object_detection\\data\\mscoco_label_map.pbtxt'

# Number of classes the object detector can identify
NUM_CLASSES = 1

# Load the label map.
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `king`.
# Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes = NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
# Transform model to graph and give it to the session
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph = detection_graph)

# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

#get Human index box with max score greater than 60
def bScoreOnlyHumans(classes, scores):
  j = -1
  max = -1
  
  #the range is defined by the programmer (we assumed a maximum of 100 detections are possible in the image)
  for i in range(0,99):
    if classes[i] == 1 and scores[i] >= 0.6:
      if scores[i] > max: 
        max = scores[i]
        j = i            
  return j
    
#convert X coord to angles 
def getX(box):
    x = ( box[1] + box[3] ) / 2 * width
    #print(x)
    x = ( ( x - width / 2 ) / width ) * 40
    #print(x)
	#to convert np.int_16 to normal int
    return int(np.int16(x.item()))

#convert Y coord to angles 
def getY(box):
    y = ( box[0] + box[2] ) / 2 * height
    #print(y)
    y = ( ( y - height / 2 ) / height ) * 25
    #print(y)
    return int(np.int16(y.item()))

# Load image using OpenCV and
# expand image dimensions to have shape: [1, None, None, 3]
# i.e. a single-column array, where each item in the column has the pixel RGB value
i = 1
imgcount = 0
path = path1

while 1:
    #Take the size
    size=int(c.recv(1024).decode("utf-8")) 
    
	#Send GOT
    c.send("GOT".encode("utf-8"))
    imgcount+=1
    
	if(imgcount==30):
        old_path=path
		
        if(path==path1):
            path=path2
        else:
            path=path1
        imgcount=0
        
		DeleteThread=Thread(target=DeleteFiles)
        DeleteThread.start()
    
	#create empty image file with specified name in path
    outfile = path+'\\image'+str(i)+'.jpg'
    myfile = open(outfile, 'wb')
    
	amount_received = 0
    while amount_received < size:
        data = c.recv(size)
        if not data:
            break
        amount_received += len(data)
        myfile.write(data)
    myfile.close()
	
    c.send("OK".encode("utf-8"))

    PATH_TO_IMAGE = outfile
    #print(PATH_TO_IMAGE)
    exists = os.path.isfile(PATH_TO_IMAGE)
    if exists:
		#read image using openCV
        image = cv2.imread(PATH_TO_IMAGE)
        #get image meta
		height, width, channels = image.shape
        #exapand image
		image_expanded = np.expand_dims(image, axis = 0)

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_expanded})

        #cv2.imshow('Object detector', image)
		
		#squeeze reduces the dimension of an multi-dimensional array by one
        maxIndex = bScoreOnlyHumans( np.squeeze(classes), np.squeeze(scores))
        
        """
		# Draw the results of the detection (aka 'visulaize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(image,
                                                            np.squeeze(boxes),
                                                            np.squeeze(classes).astype(np.int32),
                                                            np.squeeze(scores),
                                                            category_index,
                                                            use_normalized_coordinates=True,
                                                            line_thickness=8,
                                                            min_score_thresh=0.60)

        
        #cv2.imshow('Object detector', image)
		"""
        if maxIndex==-1:      
            print('no human')   
            x=0
            y=0
        else: 
            print('human detected')
			
			boxesqueezed = np.squeeze(boxes)    
            x = getX(boxesqueezed[maxIndex])  #calculates x coordinates for the center of the box
            y = getY(boxesqueezed[maxIndex])  #calculates y coordinates for the center of the box
      
        #cv2.destroyAllWindows()
       
        # All the results have been drawn on image. Now display the image.
        #cv2.imshow('Object detector', image)
        #print(str(x))
        #print(str(y))
        
		#Send length X
        c.send(str(len(str(x))).encode("utf-8"))
        #Send X
        #print(str(x))
        c.send(str(x).encode("utf-8"))

        #Send length Y
        #print(len(str(y)))
        c.send(str(len(str(y))).encode("utf-8"))
        #print(str(y))
        #Send Y
        c.send(str(y).encode("utf-8")) 



        i=i+1
    else:
        time.sleep(2)

# Close the connection with the client 
c.close()
# Clean up
cv2.destroyAllWindows()