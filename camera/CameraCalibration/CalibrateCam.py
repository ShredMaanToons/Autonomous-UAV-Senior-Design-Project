# Intrinsic parameters
    fx = 686 # Focal length in pixels
    fy = 697 # Replace when camera has been calibrated
    cx = 320 # Principal point: pixel where the optical axis hits the sensor
    cy = 240 # Replace when camera has been calibrated
# Lens distortion parameters
    k1 = 0 # Radial distortion: straight lines bowing
    k2 = 0 # k = 0 for ideal camera
    k3 = 0 # Replace when camera has been calibrated
    p1 = 0 # Tangental distortion: lens not perfectly parallel to sensor
    p2 = 0 # Replace when camera has been calibrated ----------------------- These vars should be found with cv2.calibrateCamera
# Undistorting the position of the detection
    K = np.array([[fx, 0, cx],
                    [0, fy, cy], # Intrinsic matrix, packs the camera calibration data for cv2.undistortPoints
                    [0,  0,  1]], dtype=np.float64)

    dist = np.array([k1, k2, p1, p2, k3], dtype=np.float64) # Packs the distortion coefficients for cv2.undistortPoints
