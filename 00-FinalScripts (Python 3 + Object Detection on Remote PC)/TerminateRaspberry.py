import pigpio
import uuid

import bluetooth
import codecs
import sys
import glob
import os
import subprocess
from threading import Thread

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from firebase_admin import storage

from picamera import PiCamera
from time import sleep
import socket
#camera = PiCamera()
pi = pigpio.pi()
pi.set_servo_pulsewidth(17,0)
pi.stop()

#camera.close()
print(" Program Terminated. ")