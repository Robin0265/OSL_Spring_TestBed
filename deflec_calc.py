from picamera2 import Picamera2, Preview
import cv2

import time
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (800, 600)})
picam2.configure(config)
# picam2.start_preview(Preview.DRM)
picam2.start()

# time.sleep(2)
picam2.capture_file(file_output="main.png")

print(cv2.__version__)
png_img = cv2.imread("./main.png", cv2.IMREAD_UNCHANGED)
cv2.imwrite("./main.jpg", png_img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])