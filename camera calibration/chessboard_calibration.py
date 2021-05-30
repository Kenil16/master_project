import numpy as np
import cv2
import glob

#This is based on an implementation from https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html

#Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*19,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:19].T.reshape(-1,2)

#Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#Now find all images to be used for camera calibration
images = glob.glob('data_front_chessboard2/*.png')
image = 0
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    #Here the corners of the chessboard are found
    ret, corners = cv2.findChessboardCorners(gray, (9,19),None)

    #If the corners are then add the object points and refining them. Then visualize the corners
    if ret == True:
        image += 1
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        img = cv2.drawChessboardCorners(img, (9,19), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)
        cv2.imwrite('output/img' + str(image) + '.png',img)

#Now display the camera matrix and rvec/tvec if wanted
cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print(mtx)
print(dist)

mean_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

#Now print the total estimated error for the calibration
print "total error: ", mean_error/len(objpoints)

#Get the optimal camera matrix if wanted 
img = cv2.imread('data_bottom_chessboard2/4.png')
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

