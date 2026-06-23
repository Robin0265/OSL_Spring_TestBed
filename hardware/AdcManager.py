import busio
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.ads1x15 import Pin
from adafruit_ads1x15.analog_in import AnalogIn
import csv
import traceback
import time


class AdcManager(object):
    def __init__(self, csv_file_name=None, read_retries=3, retry_delay=0.002):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c, data_rate=860)
        self.chan = AnalogIn(self.ads, Pin.A0)
        self.read_retries = read_retries
        self.retry_delay = retry_delay
        self.save_csv = not (csv_file_name is None)
        self.csv_file_name = csv_file_name
        self.csv_file = None
        self.csv_writer = None
        self.volts = -42.0 # error code
        self.read_error_count = 0
        self.last_read_ok = False

    def __enter__(self):
        if self.save_csv:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time", "voltage", "test_duration"])
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)
        return self

    def __exit__(self, etype, value, tb):
        """ Closes the file properly """
        if self.save_csv:
            self.csv_file.__exit__(etype, value, tb)
        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    def update(self):
        t0=time.time()
        self.last_read_ok = False
        for attempt in range(self.read_retries):
            try:
                self.volts = self.chan.voltage
                self.last_read_ok = True
                break
            except OSError:
                self.read_error_count += 1
                if attempt == self.read_retries - 1:
                    break
                time.sleep(self.retry_delay)

        dur = time.time()-t0
        if self.save_csv:
            self.csv_writer.writerow([time.time(),self.volts, dur])
        return self.volts
