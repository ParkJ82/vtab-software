import cv2

for i in range(10):
    cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
    opened = cap.isOpened()

    if not opened:
        print(f"Camera {i}: not available")
        cap.release()
        continue

    ret, frame = cap.read()
    print(f"Camera {i}: opened={opened}, frame={ret}")

    if ret:
        cv2.imshow(f"Camera {i}", frame)
        print(f"Showing camera {i}. Press any key for next.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    cap.release()