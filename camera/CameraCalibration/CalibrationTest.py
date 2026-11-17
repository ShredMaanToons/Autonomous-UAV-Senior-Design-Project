# Import required modules
import cv2
import numpy as np
import glob

images = glob.glob('*.png')

cv2.namedWindow('img', cv2.WINDOW_NORMAL)
cv2.resizeWindow('img', 1200, 600)

# Intrinsic parameters
fx = 782.97012129 # Focal length in pixels
fy = 782.15425781 
cx = 303.37701841 # Principal point: pixel where the optical axis hits the sensor
cy = 225.68735933 
# Lens distortion parameters
k1 = -0.50139567 # Radial distortion: straight lines bowing
k2 = 0.3469868 # k = 0 for ideal camera
k3 = -0.27986178
p1 = 0.00266273 # Tangental distortion: lens not perfectly parallel to sensor
p2 = 0.00129382

K = np.array([[fx, 0, cx],
            [0, fy, cy], 
            [0,  0,  1]], dtype=np.float64)
    
dist = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

for filename in images:
    image = cv2.imread(filename)
    h, w = image.shape[:2]

    # alpha=1 keeps all source pixels (black borders); alpha=0 crops to valid pixels only
    newK, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 1, (w, h))
    trueImage = cv2.undistort(image, K, dist, None, newK)

    both = np.hstack((image, trueImage))
    cv2.imshow('img', both)
    cv2.waitKey(0)

cv2.destroyAllWindows()