import numpy as np
import time
import subprocess
import scipy
import serial
from rvm_funcs import *
from AI import TensoflowFaceDector
ard = serial.Serial('/dev/tty.usbserial', 115200)
PATH_TO_CKPT = './model/frozen_inference_graph_face.pb'
tDetector = TensoflowFaceDector(PATH_TO_CKPT)
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
_=move_to_limits(CMD_go_to_limits)
#move to origin
dTs[0]=45
dTs[1]=45
dTs[2]=45

rotret=rotate(dTs)
#if rotret!=0:
Ts=Ts+dTs
#base_to_cam=get_base_to_cam(Ts)
base_to_eye=np.array([1,0,0])
eye_directions=np.array([])
eye_times=np.array([])
CMD_get_state=bytearray([2,0,0,0])
CMD_rotate=bytearray([1,0,0,0])
CMD_go_to_limits=bytearray([3,0,0,0])
while 1:
	state=get_state(CMD_get_state,ard)
	#shutdown
	if state[2]==2:
		shutdown(CMD_get_state,ard,base_to_window)
	#single click
	else if state[2]==3:
		rel_eye_directions, eye_times=get_rel_eye_directions(tDetector)
		dTs=move_to_limits(CMD_go_to_limits,ard)
		Ts=np.zeros(3)
		rotret=rotate(dTs)
		Ts=Ts+dTs
		eye_directions=rel_to_abs(rel_eye_directions,Ts)
		base_to_eye,eye_T=get_new_base_to_eye(eye_directions)
		base_to_window=get_window_T(base_to_eye,Ts)
	#double click
	else if state[2]==1:
		_=move_to_limits(CMD_go_to_limits,ard)
		rotret=rotate(Ts+np.array([45,45,45]))
	#normal operation
	else:
		eye_directions,eye_times=add_eye_direction(eye_directions,eye_times,Ts,tDetector)
		base_to_eye,eye_T,changed=get_eye_dir(eye_directions,base_to_eye,eye_T)
		if changed:
			dT=get_delta_theta(base_to_eye,base_to_window,Ts)
			rotret=rotate(CMD_rotate,ard,dTs)
			#if rotret!=0:
			Ts=Ts+dTs


