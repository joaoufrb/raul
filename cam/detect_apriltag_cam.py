import apriltag
#import argparse
import cv2

import time

# define a video capture object 
vid = cv2.VideoCapture(2) 

time.sleep(1)

# define the AprilTags detector options 
options = apriltag.DetectorOptions(families="tag36h11")
#options = apriltag.DetectorOptions(families="tag16h5, tag36h11")

while True:

    ret, frame = vid.read()

    if not ret:
        print("failed to grab frame")
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    detector = apriltag.Detector(options)
    results = detector.detect(gray)


    # loop over the AprilTag detection results
    for r in results:
    	# extract the bounding box (x, y)-coordinates for the AprilTag
    	(ptA, ptB, ptC, ptD) = r.corners
    	# and convert each of the (x, y)-coordinate pairs to integers
    	ptB = (int(ptB[0]), int(ptB[1]))
    	ptC = (int(ptC[0]), int(ptC[1]))
    	ptD = (int(ptD[0]), int(ptD[1]))
    	ptA = (int(ptA[0]), int(ptA[1]))
    	# draw the bounding box of the AprilTag detection
    	cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
    	cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
    	cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
    	cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
    	# draw the center (x, y)-coordinates of the AprilTag
    	(cX, cY) = (int(r.center[0]), int(r.center[1]))
    	cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
    	# draw the tag family on the frame
    	tagFamily = r.tag_family.decode("utf-8")
    	cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    	print("[INFO] tag family: {}".format(tagFamily))
    	print("[INFO] position (x): {}".format(cX))

    cv2.imshow("Detection", frame)

    #cv2.waitKey(1)
#    time.sleep(0.2)

    # the 'q' button is set as the 
    # quitting button you may use any 
    # desired button of your choice 
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break

##    k=cv2.waitKey(10) & 0XFF
#    k = cv2.waitKey(1)
#    if k%256 == 27:
#        # ESC pressed
#        print("Escape hit, closing...")
#        break
#    elif k%256 == 32:
#        # SPACE pressed
#        img_name = "opencv_frame_{}.png".format(img_counter)
#        cv2.imwrite(img_name, frame)
#        print("{} written!".format(img_name))
#        img_counter += 1

vid.release()
cv2.destroyAllWindows()
time.sleep(1)

