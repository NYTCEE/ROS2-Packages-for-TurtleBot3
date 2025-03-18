import cv2
import pupil_apriltags
import numpy as np

def main():
    # Initialize the AprilTag detector
    detector = pupil_apriltags.Detector()

    # Open the camera
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Could not open video device")
        return

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        results = detector.detect(gray)

        # Draw the AprilTag bounding box and ID on the frame
        for result in results:
            (ptA, ptB, ptC, ptD) = result.corners
            ptA = tuple(map(int, ptA))
            ptB = tuple(map(int, ptB))
            ptC = tuple(map(int, ptC))
            ptD = tuple(map(int, ptD))

            # Draw the bounding box
            cv2.polylines(frame, [np.array([ptA, ptB, ptC, ptD], dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)

            # Draw the tag ID
            tag_id = result.tag_id
            cv2.putText(frame, f"ID: {tag_id}", tuple(map(int, ptA)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the frame with AprilTags
        cv2.imshow("AprilTag Detection", frame)

        # Exit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture object and close any OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
