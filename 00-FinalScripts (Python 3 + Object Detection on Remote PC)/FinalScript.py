import pigpio
import uuid

import bluetooth
import codecs
import os
from threading import Thread

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from firebase_admin import storage

from picamera import PiCamera
import socket

""" FIREBASE CONNECTION """

##Use a service account
##Use private key from file .json
cred = credentials.Certificate('/home/pi/Desktop/FinalScripts/helmetmotionfina-1554984473719-firebase-adminsdk-bsc1o-559e01b76c.json')

##Initialize firebase_admin & get access to the storage bucket
firebase_admin.initialize_app(cred, {
    'storageBucket': 'helmetmotionfina-1554984473719.appspot.com'
})

##Get access to the database
db = firestore.client()

##Get access to the storage
#'bucket' is an object defined in the google-cloud-storage Python library.
bucket = storage.bucket()


""" CONSTANTS INITIALIZATION """

##Specifying resolution for the stream
S_Width = 640
S_Height = 480

##Get access to the camera
camera = PiCamera()
camera.resolution = (S_Width, S_Height)
camera.framerate = 80

##Give permission to variable pi to get access to the local GPIO pins
pi = pigpio.pi()

##Servo Connections:
##Horizontal: Physical pin #11 => GPIO pin #17
##Vertical: Physical pin #13 => GPIO pin #27
pinX = 17
pinY = 27

##Motion Modes
mode_manual = 0
mode_auto = 1

##X and Y mode automatic to caculate the new cummulative angle of the camera since it is moving
oldx = 0
oldy = 0

## outfile stores the path to the latest image captured
global outfile
outfile = ""

##tempImageID stores the unique id of the image
global tempImageID
tempImageID = ""

#function to delete all files from the specified path
def clear_folder(path_to_folder):
    cmd = 'rm ' + path_to_folder + '*'
    os.popen(cmd)
    
##Clear the live feed folder (this could later be used to save the feed history in a database)
feedLocation1 = '/home/pi/Desktop/FinalScripts/Feed1/'
feedLocation2 = '/home/pi/Desktop/FinalScripts/Feed2/'

clear_folder(feedLocation1)
clear_folder(feedLocation2)

"""Image capture"""

#capture images continuously while alternating between two folders
def imageCapture():
    global outfile
    global path
    
    imgcount=0
    path=feedLocation1
    uniqueID = uuid.uuid4()
    
    while True:
        for i, filename in enumerate(camera.capture_continuous(path+str(uniqueID) + "-{counter}.jpg", use_video_port=True)):
            outfile=filename
            
            imgcount+=1
            if (imgcount>=200):
                old_path=path
                if (path==feedLocation1):
                    print('changing to feed 2')
                    path = feedLocation2
                else:
                    print('changing to feed 1')
                    path = feedLocation1
				
				#clear outfile since the image may get deleted before transmission
                outfile=""
                clear_folder(old_path)
                
				imgcount = 0
                #break so that the changed path can take effect in the for loop
				break

""" INITIALIZE VARIABLES """

##Specifying initial lat, lon & mode
LatValInit = -1
LonValInit = -1
ModeInit = -1

""" LIVE STREAM SECTION """

##Update image url in firebase for stream
def SaveImageToStorage(url):
    print("-------------------------------")
    print(" Uploading Image: ", url)
    
    try:
        ##Update the URL in the db to chnage the image in the web
        ##Referencing the URL document for the update ('u' is to transform the string to frpm bytes to unicode)
        doc_url = db.collection(u'Stream').document(u'feed')
    
        ##Updating the field
        doc_url.set({
            u'url':url,
        })
    
        #print(" Upload Done ")
    except:
        ##In case of error
        print(" Upload Error for: ", url)
    
    print("-------------------------------")

##Start the live stream
def LiveStream():
    global outfile
    
    it = 0

    ##Variables to create a unique filename
    while True:
        if (outfile!=""):
            tempImage = outfile
             
            ##Get access to the element in firebase storage
            blob = bucket.blob(tempImage)
            
            ##Uploading the image and getting the image url after making the blob public to use the url in a remote computer
            blob.upload_from_filename(tempImage)        
            blob.make_public()
            url = blob.public_url
            
            #Creating thread to make stream faster
            UploadImThread = Thread(target=SaveImageToStorage, args=(url, ))
            UploadImThread.start()
     
CaptureThread = Thread(target=imageCapture)
CaptureThread.start()

UploadStreamThread = Thread(target=LiveStream)
UploadStreamThread.start()
print('finished capture and live stream')
    

""" BLUETOOTH + SOCKET REQUIREMENTS INITIALIZATION """

# Define the port on which you want to connect 
portPC = 8000
#IP_PC = "192.168.1.4"
IP_PC = "172.20.10.2"

##Mac Address of HC-06 module in order to only connect to it
bd_addr = "00:18:E4:40:00:06"
portBLE = 1

##Open and connect a scoket for communication
##Loop until connection is established
sockBLE = None
sockPC = None

#reset connection if already connected, or initialize if connecting for first time
def reset_ble(connected=False):
    global sockBLE
    
    if connected:
        sockBLE.close()
    
    connectedBLE = False
    ##Connect to arduino
    while(not connectedBLE):
        # Create a socket object
        sockBLE = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
        ##sock.connect_ex returns 0 is connectoin is successful or the errnum in case of failure
        c = sockBLE.connect_ex((bd_addr, portBLE))
        if(c == 0 ):
            connectedBLE = True   
    print(" Waiting to connect bluetooth. Status: ", c)

def reset_pc(connected=False):
    global sockPC
    
    if connected:
        sockBLE.close()
        
    connectedPC = False
    while(not connectedPC):
        # Create a socket object 
        sockPC = socket.socket()
        
        # connect to the server on local computer 
        t = sockPC.connect_ex((IP_PC, portPC))
        
        if(t==0):
            connectedPC = True
            
        print(" Waiting to connect socket. Status: ", t)
		
print("Connecting to bluetooth and socket....")
reset_pc()
reset_ble()


""" SERVO MOTION SECTION """    

##Function to convert x-angle to an appropriate format for the motion of the servo responsible for the horizontal motion
##Rounding occurs from angle to pulse (in macroseconds) (The range is 500 to 2500)
# x belongs to [-90, 90]
def MoveServoXAngle(angleX):
    global oldx
    oldx = angleX
    
    pulseX = round(((angleX+90)*(100/9))+500,1)
    
    print(" Raw X: ",angleX)
    
    if pulseX>2450:
        pulseX=2450
    
    if pulseX<550:
        pulseX=550
        
    ##Moving the horizontal servo by writing the converted value to the GPIO #17
    pi.set_servo_pulsewidth(pinX, pulseX)

##Function to convert y-angle to an appropriate format for the motion of the servo responsible for the vertical motion
##Rounding occurs from angle to pulse (in macroseconds) (The range is 500 to 2500)
# y belongs to [-180, 0]
def MoveServoYAngle(angleY):
    global oldy
    oldy = angleY
    
    #pulseY = round(((angleY+90)*(100/9))+500,1)
    pulseY = round((angleY*(-100/9))+500,1)
    print(" Raw Y: ",angleY)
    
    if pulseY>2450:
        pulseY=2450
    
    if pulseY<550:
        pulseY=550
        
    ##Moving the vertical servo by writing the converted value to the GPIO #27
    pi.set_servo_pulsewidth(pinY, pulseY)

##Function to get angles from coordinates and move the servos
##round(n * 2, -1)/2 => rounds n to the nearest 5

#reset servo rotation
def reset_servos():
    #initialize x and y to 0 0
    MoveServoXAngle(0)
    MoveServoYAngle(-60)

reset_servos()

""" MOTION MODE SECTION """

##Manual motion according to the helment motion
def ManualMotion():
    print(" Manual Motion ")
    
    x = ""
    xVal = 0
    i = 0
    ##Read length of x
    data = sockBLE.recv(1)
    ##Convert xlength from bytes to int
    xlen = ord(str(data))
    
    ##Get the x value
    ##Concept:
    # 1: Read every number of the x separately
    # 2: Convert the bytes to strings
    # 3: Add them to an accumulator variable which will then become the x when the proccess finishes
    while ( i < xlen ):
        data = sockBLE.recv(1)
        xSection = str(data)
        x = x + xSection
        i = i + 1
    
    y = ""
    yVal = 0
    
    j = 0
    
    ##Read length of y
    data = sockBLE.recv(1)
    ##Convert ylength from bytes to int
    ylen = ord(str(data))
    
    ##Get the lon value
    ##Same concept as lat
    while ( j < ylen ):
        data = sockBLE.recv(1)
        ySection = str(data)
        y = y + ySection
        j = j + 1
        
    try:
        xVal = float(x.replace(" ",""))
        yVal = float(y.replace(" ",""))
    
        print(" x: ",xVal)
        print(" y: ",yVal)
        print("-------------------------------")
		
        MoveServoXAngle(xVal)
        MoveServoYAngle(yVal)
    except:
        ##error occured in data transmission => to solve it: close connection and attempt to reconnect
        print("Error with x and y(x, y)")
        reset_ble(True)


#Function to send and receive image result using socket
def HumanDetection(path):    
    try:
        ##opening the image
        im = open(path, "rb")
        nbytes = im.read()
        size = len(nbytes)
        print(size)

        #send image size
        sockPC.send(str(size).encode("utf-8"))

        #recieve size confirmation
        if (sockPC.recv(4096).decode("utf-8") == "GOT"):
            print("OK")
            sockPC.send(nbytes)
            
        if(sockPC.recv(4096).decode("utf-8") == "OK"):
            print("Received")
            
        im.close()
        
        lenX=int(sockPC.recv(1).decode("utf-8"))
        X=int(sockPC.recv(lenX).decode("utf-8"))

        print("X: ", X)
        
        lenY=int(sockPC.recv(1).decode("utf-8"))
        Y=int(sockPC.recv(lenY).decode("utf-8"))
        
        print("Y: ", Y)
        
        X = oldx - X 
        Y = oldy - Y
        
        MoveServoXAngle(X)
        MoveServoYAngle(Y)
		
    except FileNotFoundError:
        print("File not found")


##Automatic motion according to the image detection
def AutomaticMotion():
    print(" Automatic Motion ")    
    
    if(outfile != ""):
        print("outfile is: "+outfile)
        HumanDetection(outfile)
    else:
        print("Empty image path.")


""" FIREBASE INTERACTION SECTION """

##Updating the motion mode in database
def SendMode(mode):
    print(" Sending Mode ")
    
    try:
        doc_ref = db.collection(u'Status').document(u'mode')
        doc_ref.set({
            u'mode': mode,
        })
    except:
        ##if error occured during upload, change the mode in order for it to be sent again later
        print(" Unable to Send Mode ")
        ModeInit = -1

##Updating the GPS coords in database
def SendGPSCoords(lat, lon):
    print(" Sending Location ")
    
    try:
        doc_ref = db.collection(u'GPS').document(u'coord')
        doc_ref.set({
            u'lat': lat,
            u'lon': lon,
        })
    except:
        ##if error occured during upload, change the location in order for it to be sent again later
        print(" Unable to Send Location ")
        LatValInit = -LatValInit
        LonValInit = -LonValInit

""" MAIN EXECUTION SECTION """

print(" !!!Initialization Completed, Starting Program!!! ")

while True:
    """ Reading Mode """
    
    ##Read 1 byte from the socket
    data = sockBLE.recv(1)
    ##Convert mode from bytes to int
    mode = int(codecs.encode(data, 'hex'), 16)

    ##If mode changed, update it and send it
    if ( ModeInit != mode ):
        print("Change Mode")
        reset_servos()
        
        ModeInit = mode
        ModeThread = Thread(target=SendMode, args=(ModeInit, ))
        ModeThread.start()
    
    print(" Mode: ", mode)
    print("")
    
    """ Moving Servos According to Mode """
    
    ##Move servos according to mode
    if ( ModeInit == mode_manual ):
        ManualMotion()
    elif ( ModeInit == mode_auto):
        sockBLE.send("a")
        AutomaticMotion()
        
    """ Reading GPS Coordinates """
    
    ##Reading lat and lon
    ##Mehtod:
    # 1: read the length
    # 2: read the value
    
    i = 0
    j = 0
    
    Lat = ""
    LatVal = 0
    
    Lon = ""
    LonVal = 0
    
    ##Read length of latitude
    data = sockBLE.recv(1)
    ##Convert latlength from bytes to int
    latl = ord(str(data))
    
    ##Get the lat value
    ##Concept:
    # 1: Read every number of the lon separately
    # 2: Convert the bytes to strings
    # 3: Add them to an accumulator variable which will then become the lon when the proccess finishes
    while ( j < latl ):
        data = sockBLE.recv(1)
        latSection = str(data)
        Lat = Lat + latSection
        j = j + 1
        
    ##Read length of longitude
    data = sockBLE.recv(1)
    ##Convert lonlength from bytes to int
    longl = ord(str(data))
    
    ##Get the lon value
    ##Same concept as lat
    while ( i < longl ):
        data = sockBLE.recv(1)
        lonSection = str(data)
        Lon = Lon + lonSection
        i = i + 1
        
    try:
        LatVal = float(Lat.replace(" ",""))
        LonVal = float(Lon.replace(" ",""))
    
        print(" Lat: ",LatVal)
        print(" Long: ",LonVal)
        print("-------------------------------")
    
        ##If coords changed update and send them
        if ( LatValInit != LatVal or LonValInit != LonVal ):
            LatValInit = LatVal
            LonValInit = LonVal
            GPSThread = Thread(target=SendGPSCoords, args=(LatValInit, LonValInit))
            GPSThread.start()
        
    except:
        ##error occured in data transmission => to solve it: close connection and attempt to reconnect
        print("Error with lat and lon (Lat, Lon)")
        reset_ble(True)


""" END OF EXECUTION """

##Close connection of the socket and with the servo pins
pi.set_servo_pulsewidth(17,0)
pi.stop()
sock.close()
s.close()
camera.close()
print(" Program Terminated. ")



