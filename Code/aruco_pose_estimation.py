import numpy as np
import cv2 as cv
import rospy 
from sensor_msgs.msg import Image
import std_msgs
from cv_bridge import CvBridge
import yaml

# roslaunch ueye_cam rgb8.launch
# rqt
# rosrun image_view video_recorder image:=/camera/rgb/image_raw
#bringup
#teleop


# def callback(data):
#     br = CvBridge()
#     rospy.loginfo('receiving image')
#     cv.imshow("camera", br.imgmsg_to_cv2(data))
#     cv.waitKey(1)


# def listener():
#     # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
#     rospy.init_node('test', anonymous=True)
#     while not rospy.is_shutdown():#peut etre deplacer le subscriber hors de la boucle while
#         rospy.Subscriber("/t265/stereo_ir/left/fisheye_image_raw", Image, callback)
#         rospy.spin()     

#frame = CvBridge.imgmsg_to_cv2(Image.data, desired_encoding="rdg8")
cap = cv.VideoCapture('output1.avi') 

aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_ARUCO_ORIGINAL)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Read the calibration data
def load_calib(filename):
  with open(filename, "r") as file:
    try:
        doc = yaml.load(file)
    except yaml.YAMLError as exc:
        print(exc)

    dist = np.array(doc.get('distortion_coefficients').get('data'))
    cam_mtx = doc.get('camera_matrix').get('data')
    p_mtx = doc.get('projection_matrix').get('data')
    cam_mtx = np.reshape((np.array(cam_mtx)), (3,3))
    p_mtx = np.reshape((np.array(p_mtx)), (3,4))
    
    return p_mtx, cam_mtx, dist
nbr_frame = 0
while True:
    nbr_frame += 1
    ret, frame = cap.read()

    # Preprocessing
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.medianBlur(gray, 5)
    sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
    sharpen = cv.filter2D(blur, -1, sharpen_kernel)
    parameters = cv.aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if corners:
        if len(corners) == 2:
            subpix_corners2 = cv.cornerSubPix(sharpen, corners[1], (5, 5), (-1, -1), criteria)
            subpix_corners1 = cv.cornerSubPix(sharpen, corners[0], (5, 5), (-1, -1), criteria)
            
        else:
            subpix_corners1 = cv.cornerSubPix(sharpen, corners[0], (5, 5), (-1, -1), criteria)
            

    font = cv.FONT_HERSHEY_SIMPLEX
    p_mtx, cam_mtx, dist = load_calib("ost.yaml")

    if np.all(ids is not None):

        rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners, 0.14, cam_mtx, dist)
        pose= cv.aruco.estimatePoseSingleMarkers(corners, 0.14, cam_mtx, dist)
        print("Frame n0\n",nbr_frame,"\n")
        print("Rotation vector\n",rvec,"\n")
        print("Translation vector\n",tvec,"\n")
        print("Pose\n",pose,"\n")
        
        for i in range(0, ids.size):
            # draw axis and square for the aruco markers
            cv.aruco.drawAxis(frame, cam_mtx, dist, rvec[i], tvec[i], 0.1)
            cv.aruco.drawDetectedMarkers(frame, corners)

            # code to show ids of the marker found
            strg = ''
            for i in range(0, ids.size):
                strg += str(ids[i][0]) + ', '

            cv.putText(frame, "ID: " + strg, (0, 64), font, 1, (0, 255, 0), 2, cv.LINE_AA)

    # Display result
    cv.imshow('frame', frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
