import time
# import RPi.GPIO as GPIO
from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder
from time import strftime, time
import time

PIN_START = 4
PIN_STOP = 5

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(pin1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# GPIO.setup(pin2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# GPIO.setup(PIN_START, GPIO.OUT)
# GPIO.setup(PIN_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)



if __name__ == "__main__": 
    picam2 = Picamera2()
    picam2.configure(
            picam2.create_video_configuration(
                raw={"size":(1640,1232)}, # raw size 
                main={"size": (640, 480)} # scaled size
                )
            )
    picam2.set_controls({"FrameRate": 30})
    encoder = H264Encoder()
    
    
    # time.sleep(10)
    # GPIO.output(PIN_START, GPIO.LOW)
    picam2.start_preview(Preview.DRM)
    try:
        file_name = 'Calibration'+strftime("%y%m%d_%H%M%S")+'.h264'
        picam2.start_recording(encoder, file_name)
        while True:
            time.sleep(1/200)
            # if not(GPIO.input(PIN_STOP)):
                # break
    except KeyboardInterrupt:
        # picam2.stop_preview()
        picam2.stop_recording()
        # GPIO.cleanup()
        exit()      
    finally:
        # picam2.stop_preview()
        picam2.stop_recording()
        # GPIO.cleanup()
        exit()