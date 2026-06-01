#      ▄▀▄     ▄▀▄
#     ▄█░░▀▀▀▀▀░░█▄
# ▄▄  █░░░░░░░░░░░█  ▄▄
#█▄▄█ █░░▀░░┬░░▀░░█ █▄▄█

###################################
##### Authors:                #####
##### Stephane Vujasinovic    #####
##### Frederic Uhrweiller     ##### 
#####                         #####
##### Creation: 2017          #####
###################################


#***********************
#**** Main Programm ****
#***********************


# Package importation
import numpy as np
import cv2
from pathlib import Path

# Filtering
kernel= np.ones((3,3),np.uint8)

def coords_mouse_disp(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        #print x,y,disp[y,x],filteredImg[y,x]
        average=0
        for u in range (-1,2):
            for v in range (-1,2):
                average += disp[y+u,x+v]
        average=average/9
        Distance= -593.97*average**(3) + 1506.8*average**(2) - 1373.1*average + 522.06
        Distance= np.around(Distance*0.01,decimals=2)
        print('Distance: '+ str(Distance)+' m')
        
# This section has to be uncommented if you want to take mesurements and store them in the excel
##        ws.append([counterdist, average])
##        print('Measure at '+str(counterdist)+' cm, the dispasrity is ' + str(average))
##        if (counterdist <= 85):
##            counterdist += 3
##        elif(counterdist <= 120):
##            counterdist += 5
##        else:
##            counterdist += 10
##        print('Next distance to measure: '+str(counterdist)+'cm')

  

#*************************************************
#***** Parameters for Distortion Calibration *****
#*************************************************

# Termination criteria
criteria =(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (7x7 for 8x8 chessboard)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
objp *= 0.04  # 4cm squares

# Arrays to store object points and image points from all images
objpoints= []   # 3d points in real world space
imgpointsR= []   # 2d points in image plane
imgpointsL= []
ChessImaL = None
ChessImaR = None
# Start calibration from the camera
print('Starting calibration for the 2 cameras... ')
# Load saved images from pc/tools/calib_images/
calib_dir = Path(__file__).parent / "tools" / "calib_images"
image_size = None
i = 0
while True:
    t = str(i)
    ChessImaL = cv2.imread(str(calib_dir / f'chessboard-L{t}.png'), 0)
    ChessImaR = cv2.imread(str(calib_dir / f'chessboard-R{t}.png'), 0)
    if ChessImaL is None or ChessImaR is None:
        print(f'Loaded {i} total pairs, {len(objpoints)} usable')
        break
    if image_size is None:
        image_size = ChessImaL.shape[::-1]
    retL, cornersL = cv2.findChessboardCorners(ChessImaL, (7,7), None)
    retR, cornersR = cv2.findChessboardCorners(ChessImaR, (7,7), None)
    if retL and retR:
        objpoints.append(objp)
        cv2.cornerSubPix(ChessImaL, cornersL, (11,11), (-1,-1), criteria)
        cv2.cornerSubPix(ChessImaR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)
        print(f'  loaded pair {i}')
    i += 1

if len(objpoints) == 0:
    print('Error: no valid calibration pairs found')
    exit(1)

# Determine the new values for different parameters
#   Right Side
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints,
                                                        imgpointsR,
                                                        image_size, None, None)
hR, wR = image_size[1], image_size[0]
OmtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, (wR, hR), 1, (wR, hR))

#   Left Side
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints,
                                                        imgpointsL,
                                                        image_size, None, None)
hL, wL = image_size[1], image_size[0]
OmtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, (wL, hL), 1, (wL, hL))

print('Cameras Ready to use')

#********************************************
#***** Calibrate the Cameras for Stereo *****
#********************************************

# StereoCalibrate function
#flags = 0
#flags |= cv2.CALIB_FIX_INTRINSIC
#flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
#flags |= cv2.CALIB_USE_INTRINSIC_GUESS
#flags |= cv2.CALIB_FIX_FOCAL_LENGTH
#flags |= cv2.CALIB_FIX_ASPECT_RATIO
#flags |= cv2.CALIB_ZERO_TANGENT_DIST
#flags |= cv2.CALIB_RATIONAL_MODEL
#flags |= cv2.CALIB_SAME_FOCAL_LENGTH
#flags |= cv2.CALIB_FIX_K3
#flags |= cv2.CALIB_FIX_K4
#flags |= cv2.CALIB_FIX_K5
retS, MLS, dLS, MRS, dRS, R, T, E, F = cv2.stereoCalibrate(objpoints,
                                                          imgpointsL,
                                                          imgpointsR,
                                                          mtxL,
                                                          distL,
                                                          mtxR,
                                                          distR,
                                                          image_size,
                                                          criteria=criteria_stereo,
                                                          flags=cv2.CALIB_FIX_INTRINSIC)

# StereoRectify function
rectify_scale= 0 # if 0 image croped, if 1 image nor croped
RL, RR, PL, PR, Q, roiL, roiR = cv2.stereoRectify(MLS, dLS, MRS, dRS,
                                                 image_size, R, T,
                                                 rectify_scale, (0, 0))
# initUndistortRectifyMap function
Left_Stereo_Map = cv2.initUndistortRectifyMap(MLS, dLS, RL, PL,
                                             image_size, cv2.CV_16SC2)
Right_Stereo_Map = cv2.initUndistortRectifyMap(MRS, dRS, RR, PR,
                                              image_size, cv2.CV_16SC2)
#*******************************************
#***** Parameters for the StereoVision *****
#*******************************************

# Create StereoSGBM and prepare all parameters
window_size = 3
min_disp = 2
num_disp = 130-min_disp
stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
    numDisparities = num_disp,
    blockSize = window_size,
    uniquenessRatio = 10,
    speckleWindowSize = 100,
    speckleRange = 32,
    disp12MaxDiff = 5,
    P1 = 8*3*window_size**2,
    P2 = 32*3*window_size**2)

# Used for the filtered image
stereoR=cv2.ximgproc.createRightMatcher(stereo) # Create another stereo for right this time

# WLS FILTER Parameters
lmbda = 80000
sigma = 1.8
visual_multiplier = 1.0
 
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

#*************************************
#***** Starting the StereoVision *****
#*************************************

# Test on saved images instead of live cameras
test_images = sorted(calib_dir.glob('chessboard-L*.png'))
for img_path in test_images:
    frameL_path = img_path
    frameR_path = calib_dir / img_path.name.replace('chessboard-L', 'chessboard-R')

    frameL = cv2.imread(str(frameL_path))
    frameR = cv2.imread(str(frameR_path))
    if frameL is None or frameR is None:
        continue

    # Rectify the images on rotation and alignement
    Left_nice= cv2.remap(frameL,Left_Stereo_Map[0],Left_Stereo_Map[1], interpolation = cv2.INTER_LANCZOS4, borderMode = cv2.BORDER_CONSTANT)
    Right_nice= cv2.remap(frameR,Right_Stereo_Map[0],Right_Stereo_Map[1], interpolation = cv2.INTER_LANCZOS4, borderMode = cv2.BORDER_CONSTANT)

##    # Draw Red lines
##    for line in range(0, int(Right_nice.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
##        Left_nice[line*20,:]= (0,0,255)
##        Right_nice[line*20,:]= (0,0,255)
##
##    for line in range(0, int(frameR.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
##        frameL[line*20,:]= (0,255,0)
##        frameR[line*20,:]= (0,255,0)    
        
    # Show the Undistorted images
    #cv2.imshow('Both Images', np.hstack([Left_nice, Right_nice]))
    #cv2.imshow('Normal', np.hstack([frameL, frameR]))

    # Convert from color(BGR) to gray
    grayR= cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)
    grayL= cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)

    # Compute the 2 images for the Depth_image
    disp= stereo.compute(grayL,grayR)#.astype(np.float32)/ 16
    dispL= disp
    dispR= stereoR.compute(grayR,grayL)
    dispL= np.int16(dispL)
    dispR= np.int16(dispR)

    # Using the WLS filter
    filteredImg= wls_filter.filter(dispL,grayL,None,dispR)
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
    filteredImg = np.uint8(filteredImg)
    #cv2.imshow('Disparity Map', filteredImg)
    disp= ((disp.astype(np.float32)/ 16)-min_disp)/num_disp # Calculation allowing us to have 0 for the most distant object able to detect

##    # Resize the image for faster executions
##    dispR= cv2.resize(disp,None,fx=0.7, fy=0.7, interpolation = cv2.INTER_AREA)

    # Filtering the Results with a closing filter
    closing= cv2.morphologyEx(disp,cv2.MORPH_CLOSE, kernel) # Apply an morphological filter for closing little "black" holes in the picture(Remove noise) 

    # Colors map
    dispc= (closing-closing.min())*255
    dispC= dispc.astype(np.uint8)                                   # Convert the type of the matrix from float32 to uint8, this way you can show the results with the function cv2.imshow()
    disp_Color= cv2.applyColorMap(dispC,cv2.COLORMAP_OCEAN)         # Change the Color of the Picture into an Ocean Color_Map
    filt_Color= cv2.applyColorMap(filteredImg,cv2.COLORMAP_OCEAN) 

    # Show the result for the Depth_image
    #cv2.imshow('Disparity', disp)
    #cv2.imshow('Closing',closing)
    #cv2.imshow('Color Depth',disp_Color)
    cv2.imshow('Filtered Color Depth',filt_Color)

    # Mouse click
    cv2.setMouseCallback("Filtered Color Depth",coords_mouse_disp,filt_Color)
    
    # End the Programme
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break
    
cv2.destroyAllWindows()