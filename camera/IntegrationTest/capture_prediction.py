
import cv2
import time
from picamera2 import Picamera2
from ultralytics import YOLO

# Load your trained YOLO model
model = YOLO("/home/pi/pi_code/Model_dataset_graphs/best.pt")

def capture_image(frame):
    # Generate a unique filename
    filename = f"capture_{int(time.time())}.jpg"

    # Save captured image
    cv2.imwrite(filename, frame)
    print(f"\nImage saved as {filename}")

    # Run YOLO prediction on the captured image
    results = model(filename)

    print("\nMODEL PREDICTION")

    # Access prediction results
    for result in results:

        # Get detected class names
        names = [
            result.names[cls.item()]
            for cls in result.boxes.cls.int()
        ]

        # Get confidence scores
        confs = result.boxes.conf

        if len(names) == 0:
            print("No objects detected.")

        else:
            for name, conf in zip(names, confs):
                print(f"Object detected: {name}")
                print(f"Confidence: {conf.item():.2%}")

def main():

    # Initialize Raspberry Pi camera
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={"FrameRate": 30}
    )

    picam2.configure(config)
    picam2.start()

    time.sleep(2)

    print("Press 1 to capture an image and run Model Prediction.")
    print("Press q to quit.")

    while True:

        # Capture live camera frame
        frame = picam2.capture_array()

        # Display live video
        cv2.imshow("Video", frame)

        # Read keyboard input
        key = cv2.waitKey(1) & 0xFF

        if key == ord('1'):

            # Capture image and run prediction
            capture_image(frame)

        elif key == ord('q'):

            # Exit program
            break

    # Close camera and windows
    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
