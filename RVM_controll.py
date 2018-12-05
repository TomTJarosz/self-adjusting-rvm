import numpy as np
import time
import subprocess
import scipy
import serial
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
from rvm_funcs import *
global not_moving
global in_imgs
global out_imgs


in_imgs=np.empty((h,w/2,3),dtype=np.uint8)
not_moving=True
cam=PiCamera()
cam.resolution=(1280,720)
raw=PiRGBArray(cam)
cam.capture(raw,format='rgb')
img=raw.array
[h, w] = img.shape[:2]

class get_imgs(Thread):
	def run(self):
		i=0
		cap = cv2.VideoCapture(0)
		while 1:
			if not_moving==True:
				raw=PiRGBArray(cam)
				cam.capture(raw,format='rgb')
				ar=cv2.resize(np.array(raw.array),(w/2,h/2))[:,w/4:w/2]
				ar = cv2.cvtColor(ar, cv2.COLOR_BGR2RGB)
				ar=cv2.addWeighted(ar,2,ar,0,20)
				if i==0:
					in_imgs[0:h/2,0:w/4]=ar
				if i==1:
					in_imgs[0:h/2,w/4:]=ar
				if i==2:
					in_imgs[h/2:,0:w/4]=ar
				if i==3:
					in_imgs[h/2:,w/4:]=ar
				ar=None
				raw=None
				#print "Added 1 image"
				i=(i+1)%4
			time.sleep(.6)
	

CMD_get_state=bytearray([2,0,0,0])
CMD_rotate=bytearray([1,0,0,0])
CMD_go_to_limits=bytearray([3,0,0,0])
not_moving=True
PATH_TO_CKPT = './model/frozen_inference_graph_face.pb'
get_imgs().start()

ard = serial.Serial('/dev/tty.usbserial', 115200)

#pixel_locs=np.load('cam_angles.npy')
#pixel_2_angle=np.load('pixel_2_angle.npy').item()
#pixel_locs_tree=scipy.spatial.KDTree(pixel_locs)

#load window transformation matrix
try:
	base_to_window=np.load('base_to_window.npy')
except:
	base_to_window=np.diag((1,1,1,1))
	base_to_window[0][3]=1

#get current thetas
Ts=np.zeros(3)
dTs=np.zeros(3)
#move to origin
move_to_orgin(CMD_go_to_limits,CMD_rotate,ard)

#base_to_cam=get_base_to_cam(Ts)
base_to_eye=np.array([1,0,0])
eye_directions=np.array([])
eye_times=np.array([])
eye_T=0
while 1:
	state=get_state(CMD_get_state,ard)
	#shutdown
	if state[0]==2:
		not_moving=False
		shutdown(CMD_get_state,ard,base_to_window)
		print 'shutting down'
		state=get_state(CMD_get_state,ard)
		while state[0]!=2:
			time.sleep(1)
		print 'startin up'
		not_moving=True
	#single click
	elif state[0]==3:
		state=get_state(CMD_get_state,ard)
		while state[0]!=3:
			time.sleep(1)
			state=get_state(CMD_get_state,ard)			
		inp=np.copy(in_imgs)
		rel_eye_directions, eye_times=get_rel_eye_directions(inp,True)
		not_moving=False
		#move to origin
		move_to_orgin(CMD_go_to_limits,CMD_rotate,ard)
		not_moving=True
		Ts=np.zeros(3)
		dTs=np.zeros(3)
		eye_directions=rel_to_abs(rel_eye_directions,Ts)
		base_to_eye,eye_T=get_new_base_to_eye(eye_directions)
		base_to_window=get_window_T(base_to_eye,Ts)
	#double click
	elif state[0]==1:
		not_moving=False
		#move to origin
		move_to_orgin(CMD_go_to_limits,CMD_rotate,ard)
		not_moving=True
	#normal operation
	else:
		#print 'no clicks'
		inp=np.copy(in_imgs)
		eye_directions,eye_times=add_eye_direction(inp,eye_directions,eye_times,Ts,True)
		base_to_eye,eye_T,changed=get_eye_dir(eye_directions,base_to_eye,eye_T)
		if changed:
			dTs=get_delta_theta(base_to_eye,base_to_window,Ts)
			not_moving=False
			rotret=rotate(CMD_rotate,ard,dTs)
			not_moving=True
			#if rotret!=0:
			Ts=Ts+dTs
		else:
			print 'not moving'

