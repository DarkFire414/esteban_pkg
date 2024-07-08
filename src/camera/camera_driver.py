#!/usr/bin/env python3

"""
Interfaz entre Droidcam y ROS
"""

import cv2
import rospy

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from utils.image_converter import Image_converter

def main(args):
    ic = Image_converter(callback_fcn=None)
    rospy.init_node('camera_driver_node', anonymous=True)

    cameraIp = rospy.get_param('/camera/ip')

    cap = cv2.VideoCapture(cameraIp)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        # Display the resulting frame
        ic.publish_image(frame)
        #cv2.imshow('Camera Driver', frame)
        #if cv2.waitKey(1) == ord('q'):
            #break
    # When everything done, release the capture
    print('CAMERA_DRIVER: Process finished')
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
