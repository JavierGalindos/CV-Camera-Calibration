#!/usr/bin/env python
# coding: utf-8

# # Chessboard

# In[29]:


"""
Sources:
https://docs.opencv.org/4.5.4/dc/dbb/tutorial_py_calibration.html
https://www.geeksforgeeks.org/camera-calibration-with-python-opencv/
"""
import numpy as np
import cv2 as cv
import glob

# Define the dimensions of checkerboard
CHECKERBOARD = (5, 7)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHECKERBOARD[0]
                      * CHECKERBOARD[1],
                      3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('Images/Chessboard/*.jpeg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (CHECKERBOARD[0],CHECKERBOARD[1]), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (CHECKERBOARD[0],CHECKERBOARD[1]), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
        filename = './Test/Image%d.jpg'%i
        i +=1
        cv.imwrite(filename,img)
cv.destroyAllWindows()




# In[30]:


# Calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


# ## Saving xml

# In[34]:


filename = 'calibration.xml'
s = cv.FileStorage(filename, cv.FileStorage_WRITE)
s.write('camera_matrix', mtx)
s.write('distortion_coefficients', dist)
s.release()


# # Circles grid

# In[55]:


# Define the dimensions of checkerboard
CHECKERBOARD = (4, 5)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((CHECKERBOARD[0]
                      * CHECKERBOARD[1],
                      3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('Images/Circles/*.jpeg')
i = 0
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findCirclesGrid(gray, (CHECKERBOARD[0],CHECKERBOARD[1]), cv.CALIB_CB_ASYMMETRIC_GRID)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (CHECKERBOARD[0],CHECKERBOARD[1]), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
        #filename = './Test/Image%d.jpg'%i
        #i +=1
        #cv.imwrite(filename,img)
cv.destroyAllWindows()




# In[56]:


# Calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)


# ## Saving xml

# In[58]:


filename = 'calibration2.xml'
s = cv.FileStorage(filename, cv.FileStorage_WRITE)
s.write('camera_matrix', mtx)
s.write('distortion_coefficients', dist)
s.release()


# ## Saving xml

# In[58]:


filename = 'calibration2.xml'
s = cv.FileStorage(filename, cv.FileStorage_WRITE)
s.write('camera_matrix', mtx)
s.write('distortion_coefficients', dist)
s.release()


# In[ ]:




