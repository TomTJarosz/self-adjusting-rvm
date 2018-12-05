import numpy as np
import time
import subprocess
import scipy
import serial
from threading import Thread
from inference_usbCam_face import TensoflowFaceDector

PATH_TO_CKPT = './model/frozen_inference_graph_face.pb'
tDetector = TensoflowFaceDector(PATH_TO_CKPT)
##Function: get_base_to_cam(Ts)
##Use: Given the angles of the motors, computes the transformation matrix $T^{0}_{3} \in \R^{4 x 4}$
##Parameters: Ts-numpy array, size 3, values=[theta0, thetea1, theta2], the angles of the motors relative to the base frame
##Side affects: None

def get_base_to_cam(Ts):
	t0=Ts[0]*np.pi/180.0
	t1=Ts[1]*np.pi/180.0
	t2=Ts[2]*np.pi/180.0
	return np.array([[np.cos(t0)*np.cos(t2)-np.sin(t0)*np.sin(t1)*np.sin(t2),	-1*np.sin(t0)*np.cos(t1),	np.cos(t0)*np.sin(t2)+np.sin(t0)*np.sin(t1)*np.cos(t2),		0],
					[np.sin(t0)*np.cos(t2)+np.cos(t0)*np.sin(t1)*np.sin(t2),	np.cos(t0)*np.cos(t1),		np.sin(t0)*np.sin(t2)-np.cos(t0)*np.sin(t1)*np.cos(t2),		0],
					[-1*np.cos(t1)*np.sin(t2),									np.sin(t1),					np.cos(t1)*np.cos(t2),										0],
					[0,															0,							0,															1]])

##Function: get_rel_eye_directions(in_imgs,tDetector,visualize)
##Use: Given images of the cabin, this fuction returns a list of the eye locations in the image. Can also visualize the image labeling
##Parameters: 	in_imgs- numpy array shape (640,720,3); 4 most recent images of the cabin.
#				visualize- boolean parameter, if enabled, displace images with eye locations labeled
##Side affects: runs for ~4 seconds
def get_rel_eye_directions(inp,visualize):
	num_images=0
	rel_eye_directions=[]
	eye_times=[]
	while len(rel_eye_directions)<4:
		t1=time.time()
		eyes, etimes  = tDetector.get_eyes(inp,visualize)
		#print "time per image:"+str((time.time()-t1)/4)
		rel_eye_directions=rel_eye_directions+eyes
		eye_times=eye_times+etimes
	return np.array(rel_eye_directions),np.array(eye_times)

def rot_z_then_y(tz,ty):
	tz=tz*np.pi/180.0
	ty=ty*np.pi/180.0
	return np.array([[np.cos(tz),-1*np.sin(tz),0],[np.sin(tz),np.cos(tz),0],[0,0,1]]).dot(np.array([[np.cos(ty),0,np.sin(ty)],[0,1,0],[-1*np.sin(ty),0,np.cos(ty)]]))

def rel_to_abs(rel_eye_directions,Ts):
	base_to_cam=get_base_to_cam(Ts)
	eye_directions=[]
	print "rel_eye_directions="+str(rel_eye_directions)
	print 'mirror rotation matrix='+str(base_to_cam[0:3,0:3])
	for red in list(rel_eye_directions):
		#pixel=pixel_locs_tree.query(red)
		#angles=pixel_2_angle[pixel]
		angles=np.array([-26.5*(red[0]),26.5*(red[1]-360)])
		print 'eye angles='+str(angles)
		print "rel_eye_dir rot matrix="+str(rot_z_then_y(angles[0],angles[1]))
		print "eye_dir="+str(base_to_cam[0:3,0:3].dot(rot_z_then_y(angles[0],angles[1]))[:,0][0:3])
		eye_directions.append(base_to_cam[0:3,0:3].dot(rot_z_then_y(angles[0],angles[1]))[:,0][0:3])
	return np.array(eye_directions)

def get_new_base_to_eye(eye_directions):
	ret= np.mean(eye_directions,axis=0)
	ret=ret/np.linalg.norm(ret)
	return ret,time.time()

def get_window_T(base_to_eye,Ts):
	base_to_cam=get_base_to_cam(Ts)
	Ts=Ts*np.pi/180.0
	window_P=base_to_cam[0:3,0:3].dot(np.diag((1,-1,-1)).dot(base_to_cam[0:3,0:3].T.dot(base_to_eye)))
	window_T=np.arctan(-1*np.sin(Ts[1])/(np.cos(Ts[1])*np.cos(Ts[2])))
	window_R=np.array([[1,0,0],[0,np.cos(window_T),-1*np.sin(window_T)],[0,np.sin(window_T),np.cos(window_T)]])
	base_to_window=np.zeros((4,4))
	base_to_window[0:3,0:3]=window_R
	base_to_window[0][3]=window_P[0]
	base_to_window[1][3]=window_P[1]
	base_to_window[2][3]=window_P[2]
	base_to_window[3][3]=1
	return base_to_window

def add_eye_direction(inp,eye_directions,eye_times,Ts,visualize):
	t1=time.time()
	eyes, etimes  = tDetector.get_eyes(inp,visualize)
	#print "time per image:"+str((time.time()-t1)/4)
	eye_dirs=rel_to_abs(eyes,Ts)
	cur_time=time.time()
	good_indexes=[i for i in xrange(len(eye_times)) if cur_time-eye_times[i]<10]
	eye_directions=eye_directions[good_indexes]
	eye_times=eye_times[good_indexes]
	eye_directions=list(eye_directions)
	eye_times=list(eye_times)
	eye_directions=eye_directions+list(eye_dirs)
	eye_times=eye_times+list(etimes)
	return np.array(eye_directions),np.array(eye_times)

def get_eye_dir(eye_directions,base_to_eye,eye_T):
	if len(eye_directions)==0:
		print 'AAAAAAA'
		return base_to_eye,eye_T,False
	if len(eye_directions)>1:
		mean=np.mean(eye_directions,axis=0)
	else:
		mean=np.copy(eye_directions[0])
	if len(eye_directions)<4:
		print 'BBBBBBB'
		return base_to_eye,eye_T,False
	dist=np.linalg.norm(mean-base_to_eye)
	variance=np.var(np.linalg.norm(eye_directions-mean,axis=0))
	cur_time=time.time()
	d_time=np.sqrt(cur_time-eye_T)
	time_weight=5.0
	print "eye_directions="+str(eye_directions)
	print "mean="+str(mean)
	print "base_to_eye="+str(base_to_eye)
	print "dist="+str(dist)
	print "d_time="+str(d_time)
	print "variance="+str(variance)
	score=(dist*( 1/ (1 + np.exp(-d_time/time_weight))))/variance
	#.65 is arbitary threshold, and subject to change
	print "score="+str(score)
	if score>.45 and dist>.047:
		print 'CCCCCCC'
		return mean, cur_time,True
	print 'DDDDDD'
	return base_to_eye, eye_T,False


def get_delta_theta(base_to_eye,base_to_window,Ts):
	x=base_to_eye+base_to_window[:,3][0:3]
	x=x/np.linalg.norm(x)
	base_to_cam=get_base_to_cam(Ts)
	y=(np.identity(3)-np.outer(x,x)).dot(base_to_window[:,1][0:3])
	y=y/np.linalg.norm(y)
	newTs=np.zeros(3)
	newTs[1]=np.arcsin(y[2])
	newTs[2]=np.arcsin(-1*x[2]/np.cos(newTs[1]))
	newTs[0]=np.arccos(y[1]/np.cos(newTs[1]))
	newTs=newTs*180./np.pi
	if newTs[0]>0:
		newTs[0]=newTs[0]*-1
	newTs=np.rint(newTs/0.9)*0.9
	for i in xrange(3):
		if newTs[i]>45:
			newTs[i]=45
		if newTs[i]<-45:
			newTs[i]=-45
	return newTs-Ts


def get_state(CMD,ard):
	pre_state_t=time.time()
	ard.write(CMD)
	state=bytearray(ard.read(4))
	#print "time to get state from arduino="+str(time.time()-pre_state_t)
	return state	

#returns -1*dTs to reach 0s from previous oreintation
def move_to_limits(CMD,ard):
	while 1:
		ard.write(CMD)
		resp=bytearray(ard.read(4))
		if resp[0]>7:
			if resp[0]&0x07!=0:
				time.sleep(2)
				#overheating
			continue
		dTs=np.array([0.9*(resp[1]-50),0.9*(resp[2]-50),0.9*(resp[3]-50)])
		return dTs

def rotate(CMD,ard,dTs):
	steps=np.rint(dTs/0.9)
	steps=bytearray(np.array(steps,dtype="int8"))
	while 1:
		CMD[1]=np.abs(steps[0])
		CMD[2]=np.abs(steps[1])
		CMD[3]=np.abs(steps[2])
		if steps[0]<0:
			CMD[0]=CMD[0]+32 
		if steps[1]<0:
			CMD[0]=CMD[0]+64
		if steps[2]<0:
			CMD[0]=CMD[0]+128 
		ard.write(CMD)
		resp=bytearray(ard.read(4))
		if resp[0]&0x07!=0:
			time.sleep(2)
			#overheating
			continue
		return#TODO handle limit hit

def shutdown(CMD,ard,base_to_window):
	np.save('base_to_window',base_to_window)
	state=get_state(CMD,ard)
	time.sleep(4)
	while state[2]!=2:
		state=get_state(CMD,ard)
		time.sleep(4)

def get_pix_loc(thresh):
	cmd = "raspstill -o ./image.jpg"
	subprocess.call(cmd, shell=True)
	img=cv2.imread("image.jpg",cv2.IMREAD_GRAYSCALE)
	indexes=[np.array(x,y) for y in xrange(img.shape[0]) for x in xrange(img.shape[1]) if img[y][z]>thresh ]
	pix_loc=np.mean(indexes,axis=0)
	pl2=np.array([img.shape[1]-pix_loc[0],pix_loc[1]])
	return pix_loc, pl2


def center_led():
	cmd = "raspstill -o ./image.jpg"
	subprocess.call(cmd, shell=True)
	img=cv2.imread("image.jpg",cv2.IMREAD_GRAYSCALE)
	center_x=img.shape[1]/2.0
	center_y=img.shape[0]/2.0
	center=np.array([center_x,center_y])
	indexes=[np.array(x,y) for y in xrange(img.shape[0]) for x in xrange(img.shape[1]) if img[y][z]>thresh ]
	pix_loc=np.mean(indexes,axis=0)
	cv2.circle(img,(center_x,center_y),2,(0,0,255),-1)
	cv2.imshow(img)
	k = cv2.waitKey(1) & 0xff
	if k == ord('q') or k == 27:
		pass
	while raw_input("Measured distance from center is "+str(center-pix_loc)+". Should I go?")[0].lower()!="y":
		cmd = "raspstill -o ./image.jpg"
		subprocess.call(cmd, shell=True)
		img=cv2.imread("image.jpg",cv2.IMREAD_GRAYSCALE)
		cv2.circle(img,(center_x,center_y),2,(0,0,255),-1)
		indexes=[np.array(x,y) for y in xrange(img.shape[0]) for x in xrange(img.shape[1]) if img[y][z]>thresh ]
		pix_loc=np.mean(indexes,axis=0)
   		k = cv2.waitKey(1) & 0xff
		if k == ord('q') or k == 27:
			pass


def move_to_orgin(CMD_go_to_limits,CMD_rotate,ard):
	_=move_to_limits(CMD_go_to_limits,ard)
	_=rotate(CMD_rotate,ard,np.array([40*.9,75*.9,55*.9]))

