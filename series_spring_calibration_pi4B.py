import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder


PIN_START = 4
PIN_STOP = 5

GPIO.setmode(GPIO.BCM)
# GPIO.setup(pin1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(pin2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_START, GPIO.OUT)
GPIO.setup(PIN_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)

picam2 = Picamera2()
picam2.configure(
        picam2.create_video_configuration(
            raw={"size":(1640,1232)}, # raw size 
            main={"size": (640, 480)} # scaled size
            )
        )
picam2.set_controls({"FrameRate": 30})
encoder = H264Encoder()


if __name__ == "__main__": 
    
    GPIO.output(PIN_START, GPIO.LOW)
    try:
        picam2.start_recording(encoder, 'test_modified6.h264')
        while True:
            time.sleep(1/200)
            if not(GPIO.input(PIN_STOP)):
                break
    finally:
        picam2.stop_recording()
        GPIO.cleanup()
        exit()