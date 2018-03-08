from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import imutils
import time
import cv2

#initialize the camera and grab a reference to the raw cameracapture
camera = PiCamera()
camera.resolution = (320,240)
camera.framerate = 30
rawCapture = PiRGBArray(camera,size=(320,240))
stream = camera.capture_continuous(rawCapture,format="bgr",use_video=True,burst=True)
camera.close()
#FPS start
vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()
#blob
params = cv2.SimpleBlobDetector_Params()
#change thresholds
params.minThreshold = 0
params.maxThreshold = 255

#fliter by color
params.filterByColor = True
params.blobColor = 0

#Setup the detector whith default parameters
detector = cv2.SimpleBlobDetector_create(params)

while(True):
	frame = vs.read()
	frame = imutils.resize(frame,width=160)
	cv2.imshow("CameraStream",frame)
#blob 
	keypoints = detector.detect(frame)
#hough circle
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,25,param1=80,param2=35,minRadius=0,maxRadius=0)

	if(keypoints):
		for i in range (0,len(keypoints)):
			x = keypoints[i].pt[0]
			y = keypoints[i].pt[1]
			Position_x = int(x*100)
			Position_y = int(y*100)
			#Draw keypoints
    		im_keypoints = cv2.drawKeypoints(frame,keypoints,np.array([]),(0,0.255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		    #show keypoints
    		cv2.imshow("Keypoints",im_keypoints)
	else:
		Position_x = 8000
		Position_y = 6000

	if circles is not None:
		for c in circles[0,:]:
			#draw the circle
			cv2.circle(gray,(c[0],c[1]),c[2],(255,0,0),2)
			#draw the center of the circle
			cv2.circle(gray,(c[0],c[1]),2,(255,0,0),3)
			Circles_x = int(c[0]*100)
			Circles_y = int(c[1]*100)
		cv2.imshow("circles",gray)

	print "X: %.2f"%(Position_x)
	print "Y: %.2f"%(Position_y)

	key = cv2. waitKey(1)&0xFF
	if key == ord("q"):
#        #Draw keypoints
#		im_keypoints = cv2.drawKeypoints(frame,keypoints,np.array([]),(0,0.255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#        #show keypoints
#		cv2.imshow("Keypoints",im_keypoints)
		break
	fps.update()

fps.stop()
print "[INFO] elasped time: {:.2f}".format(fps.elapsed())
print "[INFO] approx. FPS: {:.2F}".format(fps.fps())

cv2.destroyAllWindows()
vs.stop()
