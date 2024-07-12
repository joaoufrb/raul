
import cv2
import numpy
from apriltag import apriltag
from dt_apriltags import Detector
import os
import yaml
import time
from cv2 import imshow

# define a video capture object
vid = cv2.VideoCapture(2)

time.sleep(1)

at_detector = Detector(families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

with open('cam_info.yaml', 'r') as stream:
    parameters = yaml.safe_load(stream)
#fx, fy: The camera's focal length (in pixels). For most cameras fx and fy will be equal or nearly so.
#cx, cy: The camera's focal center (in pixels). For most cameras this will be approximately the same as the image center.

while True:

    ret, frame = vid.read()

    if not ret:
        print("failed to grab frame")
        break

    #img = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    

    # K format:
    # K: [fx, ??, cx, ??, fy, cy, ??, ??, ??]
    # reshape to this:
    #   fx  ??  cx
    #   ??  fy  cy
    #   ??  ??  ??
    cameraMatrix = numpy.array(parameters['sj4000']['K']).reshape((3,3))
    
    # camera_params: ([fx, fy, cx, cy])
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

#    cv2.imshow('Original image',img)

   # camera_params and tag_size (in meters)
    tags = at_detector.detect(img, True, camera_params, parameters['sj4000']['tag_size'])
    print(tags)

    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

        cv2.putText(color_img, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255))

    cv2.imshow('Detected tags', color_img)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()
time.sleep(1)
