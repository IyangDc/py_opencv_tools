#代码来源于互联网
#2018.3.8经IyangDc整合调试，确认能够在Raspiberry+picamera+opencv3的环境下运行
#使用时需根据具体情况，调节检测器的各项参数
#用于检测图像中的斑点特征和圆特征
#加载硬件包
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
#加载软件包
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
#blob检测器参数对象生成
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
		print "X: %.2f"%(Position_x)
		print "Y: %.2f"%(Position_y)
	else:
		Position_x = 8000
		Position_y = 6000
	#hough circle
	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,25,param1=80,param2=35,minRadius=0,maxRadius=0)
	if circles is not None:
		for c in circles[0,:]:
			#draw the circle
			cv2.circle(gray,(c[0],c[1]),c[2],(255,0,0),2)
			#draw the center of the circle
			cv2.circle(gray,(c[0],c[1]),2,(255,0,0),3)
			Circles_x = int(c[0]*100)
			Circles_y = int(c[1]*100)
		cv2.imshow("circles",gray)
	key = cv2. waitKey(1)&0xFF
	if key == ord("q"):
		break
	fps.update()
fps.stop()
print "[INFO] elasped time: {:.2f}".format(fps.elapsed())
print "[INFO] approx. FPS: {:.2F}".format(fps.fps())
cv2.destroyAllWindows()
vs.stop()
