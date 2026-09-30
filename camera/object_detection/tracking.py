import cv2 # Open Source Computer Vision Library
# Used in "debug_vis_multi" to draw on the video feed with .circle .line .rectange .putText

def mult_detect_targets(frame, model, classes, conf=0.5):
    # Detects the target then gives the bounding box
    # Returns the box offset from the center of frame

    if frame is None or frame.size == 0:
        return None, frame # Rejects invalid images.

    # model is a YOLO object, (Frame is the input image, verbose=False supresses console output, conf is confidence, classes is the list of objects we are identifying??)
    # [0] at the end is used to change the default array output of the model into the first scalar item from the array which is put into "res".
    # res is "result", taking the result of the analized frame
    # res.boxes returns the box of the image
    # res.names[id] returns the name of the boxed item associated with the class id parameter
    res = model(frame, verbose=False, conf=conf, classes=classes)[0]

    # Finding center of the input image
    H, W = frame.shape[:2]
    frame_cx = W / 2.0
    frame_cy = H / 2.0

    targets = []

    # Loop through every box in the image, noting the 1) confidence, 2) location in frame, 3) width and hight, 4) Center, class info, and stores the info in the targets array that will be returned
    for b in res.boxes:
        # 1)
        det_conf = float(b.conf.item())
        # 2)
        x1, y1, x2, y2 = b.xyxy[0].cpu().numpy()
        # 3)
        x = int(x1)
        y = int(y1)
        w = int(x2 - x1)
        h = int(y2 - y1)
        
        if w <= 0 or h <= 0:
            continue # Skip broken boxes
        # 4)
        cx = x + w / 2.0
        cy = y + h / 2.0
        err_px = (cx - frame_cx, cy - frame_cy)

        # Get class info
        class_id = int(b.cls[0])
        class_name = res.names[class_id]

        targets.append({
            "bbox": (x,y,w,h),
            "class": class_name,
            "error": err_px,
            "conf": det_conf
        })

    return targets, frame

# Function for the user to see how the objects are being detected
def debug_vis_multi(frame, targets):
    vis = frame.copy() # Takes a copy of the frame and measures it
    H, W = vis.shape[:2]

    # Draw center once
    cv2.circle(vis, (W // 2, H // 2), 5, (255, 0, 0), -1)

    for t in targets:
        x, y, w, h = map(int, t["bbox"])
        err_px = t["error"]
        cls = t["class"] # Get the information for each target

        # Box
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Target center
        cx = int(W / 2 + err_px[0])
        cy = int(H / 2 + err_px[1])
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

        # Line to center
        cv2.line(vis, (W // 2, H // 2), (cx, cy), (255, 255, 0), 2)

        # Label
        cv2.putText(
            vis,
            f"{cls}",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

    return vis
