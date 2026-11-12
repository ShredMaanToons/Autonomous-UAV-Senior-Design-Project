# In case of using a new camera or lens the camera must be calibrated with this code
For a guide on calibration reference this website:
https://www.geeksforgeeks.org/python/camera-calibration-with-python-opencv/
> [!warning]
> There is a scam ad on this website that will change the entire screen to try to make it look like you are hacked.

**Copy the code to a .py file on your computer.

Use the camera and lens you want to calibrate to take at least 20 pictures of a checkerboard pattern from multiple angles and making sure the pattern is positioned across all edges and center space.

Add the images taken from the camera you want to calibrate as .jpg files to the same folder as your .py file. Run the code, ensure at least 20 images are reported as detected.

Copy the reported camera matrix and distortion coefficient as constants into the file where you will run cv2.undistortPoints**
(in this case camera/object_detection/object_manager.py -> _target_position)
