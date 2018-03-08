from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera

import cv2
import numpy as np
import time
import imutils

camera = PiCamera()
camera.resolution = (320,240)
camera.framerate = 30
rawCapture = PiRGBArray(camera,size=(320,240))
stream = camera.capture_continuous(rawCapture,format="bgr",use_video_port=True,burst=True)
camera.close()

#params for Shi-Tomasi corner detection
feature_params = dict(maxCorners=100,qualityLevel=0.5,minDistance=7,blockSize=7)

lk_params = dict( winSize=(15,15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))  

color = np.random.randint(0,255,(100,3))
#FPS start
vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()

#take first frame and find corners in it
old_frame = vs.read()
old_frame = imutils.resize(old_frame,width=80)
old_gray = cv2.cvtColor(old_frame,cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray,mask=None,**feature_params)

#create a mask  
mask = np.zeros_like(old_frame)

while(True):
	frame = vs.read()
	frame = imutils.resize(frame,width=80)
	frame_gray = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY)
	#calculate optical flow
	p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray,frame_gray,p0,None,**lk_params)	
	if(p1 is None):
		old_gray = frame_gray.copy()
		p0 = cv2.goodFeaturesToTrack(old_gray,mask=None,**feature_params)
		mask = np.zeros_link(old_frame)
		OpticalFlow_flag = 0
	else:
		#Select good points
		good_new = p1[st==1]
		good_old = p0[st==1]
		#draw the tracks
		for i,(new,old) in enumerate(zip(good_new,good_old)):
			a,b = new.ravel()
			c,d = old.ravel()
			mask = cv2.line(mask,(a,b),(c,d),color[i+1].tolist(),1)
			frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
			OpticalFlow_x = int(a * 100)
			OpticalFlow_y = int(b * 100)
			LastOpticalFlow_x = int(c * 100)
			LastOpticalFlow_y = int(d * 100)
		
		img = cv2.add(frame,mask)
		cv2.imshow("img",img)		

		#Now update the previous frame and previous points
		old_gray = frame_gray.copy()
		p0 = good_new.reshape(-1,1,2)
		
		key = cv2.waitKey(1)&0xff
		if key == ord("q"):
			break
		fps.update()

fps.stop()
print "[INFO] elasped time: {:.2f}".format(fps.elapsed())
print "[INFO] approx. FPS: {:.2F}".format(fps.fps())

cv2.destroyAllWindows()
vs.stop()
