import numpy as np
import cv2
import glob
from naoqi import ALProxy

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 1)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 1)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 1)
    return img

axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)*28.22
images = glob.glob('*.jpg')
nimg = len(images)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)*28.22

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
c=0

ip_addr = ('169.254.37.37') #Ethernet
#ip_addr = ('192.168.0.109') #Wi-Fi
port_num = 9559             #int(sys.argv[2])

# get NAOqi module proxys
memProxy = ALProxy("ALMemory",ip_addr,port_num)
videoDevice = ALProxy('ALVideoDevice', ip_addr, port_num)
# subscribe top camera
AL_kTopCamera = 0
#AL_kBotCamera = 1
AL_kQVGA = 1            # 320x240
AL_kBGRColorSpace = 13
captureDevice = videoDevice.subscribeCamera(
    "test", AL_kTopCamera, AL_kQVGA, AL_kBGRColorSpace, 20)

# create image
width = 320
height = 240
image = np.zeros((height, width, 3), np.uint8)
imgraw = np.zeros((height, width, 3), np.uint8)


for fname in images:

    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #print fname
    c=c+1

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(0)
       
cv2.destroyAllWindows()
print '- - - ESTIMATING CAMERA PARAMETERS - - -'
print 'Total:', c, 'images'
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print ('Calibration Matrix: ')
print 
print mtx
np.savetxt('calibmat', mtx, delimiter="; ")
print 
print ('Distortion Coeficients: ')
print
print dist
np.savetxt('distcoef', dist, delimiter="; ")
print
print ('Rotation Vectors: ')
print
print rvecs
np.savetxt('rvecs', rvecs, delimiter="; ")
print
print ('Translation Vectors: ')
print
print tvecs
np.savetxt('tvecs', tvecs, delimiter="; ")
print

mean_error = 0
tot_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

mean_error = tot_error/len(objpoints)
print "total error: ", mean_error
np.savetxt('rmserror', (mean_error,), delimiter="; ")

print '- - - - - Pose Estimation! - - - - -'
c=0
Z = np.zeros((54))
while True:

    # get image
    result = videoDevice.getImageRemote(captureDevice);

    if result == None:
        print 'cannot capture.'
    elif result[6] == None:
        print 'no image data string.'
    else:

        # translate value to mat
        values = map(ord, list(result[6]))
        i = 0
        for y in range(0, height):
            for x in range(0, width):
                image.itemset((y, x, 0), values[i + 0])
                image.itemset((y, x, 1), values[i + 1])
                image.itemset((y, x, 2), values[i + 2])
                i += 3
        imgraw = image

        # exit by [ESC]  
        if cv2.waitKey(10) == 27:
            cv2.destroyAllWindows()
            break

        #transform to grayscale
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        #locate and save the corners positions
        ret, corners = cv2.findChessboardCorners(gray, (9,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            img = imgraw
            
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            _, rvecs, tvecs, inliers  = cv2.solvePnPRansac(objp, corners2, mtx, dist)
            imgpoints.append(corners2)
            rvecs,jacobian = cv2.Rodrigues(rvecs) #from R-vector to R-matrix

#Initialize all sensor variables
            theta1_head = 0      #HeadYaw
            theta2_head = 0      #HeadPitch
            theta1_lleg = 0      #LHipYawPitch
            theta2_lleg = 0      #LhipRoll
            theta3_lleg = 0      #LhipPitch
            theta4_lleg = 0      #LKneePitch
            theta5_lleg = 0      #LAnklePitch
            theta6_lleg = 0      #LAnkleRoll
            
#Get all sensor values for calibration
            theta1_head = memProxy.getData("Device/SubDeviceList/HeadYaw/Position/Sensor/Value")
            theta2_head = memProxy.getData("Device/SubDeviceList/HeadPitch/Position/Sensor/Value")
            theta1_lleg = memProxy.getData("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value")
            theta2_lleg = memProxy.getData("Device/SubDeviceList/LHipRoll/Position/Sensor/Value")
            theta3_lleg = memProxy.getData("Device/SubDeviceList/LHipPitch/Position/Sensor/Value")
            theta4_lleg = memProxy.getData("Device/SubDeviceList/LKneePitch/Position/Sensor/Value")
            theta5_lleg = memProxy.getData("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value")
            theta6_lleg = memProxy.getData("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value")            

            sensormatrix = np.array([[theta1_head],[theta2_head],[theta1_lleg],[theta2_lleg],[theta3_lleg],[theta4_lleg],[theta5_lleg],[theta6_lleg]])
            #print sensormatrix

            point = [corners2[6][0][0],corners2[6][0][1],1]
            print "Point 4: ", point
            center = (int(np.round(point[0])),int(np.round(point[1])))
            img = cv2.circle(img,center, 5, (0,200,0), -1)
            point = np.dot(point,rvecs) + np.matrix.transpose(tvecs)
            Z = point[0,2]
            print "Distance estimate: ", Z
 
            #center = (int(np.round(point[0,1])),int(np.round(point[0,0])))
            #img = cv2.circle(img,center, 5, (0,200,0), -1)
            #for i in range(0,54):  
            #    point = np.dot(objp[i],rvecs) + np.matrix.transpose(tvecs)
            #    Z[i] = point[0,2] #Z-value to mm
            #    center = (int(np.round(point[0,1])),int(np.round(point[0,0])))
            #    img = cv2.circle(img,center, 5, (0,100,100), -1)
            
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            #cv2.drawChessboardCorners(image, (9, 6), corners, ret)
            img = draw(img, corners2, imgpts)
            
            cv2.waitKey(10)
            cv2.imshow('img', img)
            #cv2.imshow('imgRaw', imgraw)
        elif ret == 0:
            cv2.imshow('img', image)
            #cv2.imshow('imgRaw', imgraw)
