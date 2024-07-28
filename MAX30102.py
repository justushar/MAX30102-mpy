import machine
import utime
from max30102 import MAX30102, MAX30105_PULSE_AMP_MEDIUM
import math
from ssd1306 import SSD1306_I2C
import writer
import freesans20
import framebuf
import sys
from machine import Pin
import gc

# Define the I2C pins
i2c = machine.SoftI2C(sda=machine.Pin(14), scl=machine.Pin(15), freq=400000)
i2c1 = machine.SoftI2C(scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)
display = SSD1306_I2C(128, 64, i2c1)

# Button to return to main program
button_return_main=Pin(10,Pin.IN,Pin.PULL_UP)

# Custom Exception
class Return_To_Main(Exception):
    pass

# Create an instance of the MAX30102 sensor
sensor = MAX30102(i2c=i2c)

# Scan I2C bus to ensure that the sensor is connected
if sensor.i2c_address not in i2c.scan():
    print("Sensor not found.")
    sys.exit()
elif not sensor.check_part_id():
    # Check that the targeted sensor is compatible
    print("I2C device ID not corresponding to MAX30102 or MAX30105.")
    sys.exit()
else:
    print("Sensor connected and recognized.")

# Set up the sensor with default configuration
sensor.setup_sensor()
sensor.set_sample_rate(3200)
sensor.set_fifo_average(4)
sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)
sensor.set_pulse_width(215)

# Variables for beat detection
BUFFER_SIZE = 250
buffer = [0] * BUFFER_SIZE
peak_indices = []
min_distances = []
max_distances = []
peak_threshold = 38600
min_threshold = 0.2
max_threshold = 0.8
last_peak_index = -1
beat_avg = 0  # Initialize beat_avg variable

# Constants for SpO2 calculation
a = 1.5958422
b = -34.6596622
c = 112.6898759

# Low-pass filter parameters
ALPHA = 0.4

def format_reading(reading):
    if reading < 100:
        return "{:02}".format(int(reading))
    else:
        return "{:03}".format(int(reading))




def find_peaks(data):
    peaks = []
    for i in range(1, len(data) - 1):
        if data[i] > data[i-1] and data[i] > data[i+1]:
            peaks.append(i)
    return peaks

def calculate_bpm(peaks):
    if len(peaks) >= 2:
        # Calculate the time interval between consecutive peaks
        peak_intervals = [peaks[i] - peaks[i-1] for i in range(1, len(peaks))]
        
        # Compute the BPM based on the average peak interval
        avg_interval = sum(peak_intervals) / len(peak_intervals)
        sampling_rate = 3200
        bpm = (60 * sampling_rate / avg_interval) + 30000
        return bpm
    else:
        return 0

def calculate_spo2(red_ac, red_dc, ir_ac, ir_dc):
    if ir_ac != 0:
        red_absorbance = math.log10(red_dc / red_ac)
        ir_absorbance = math.log10(ir_dc / ir_ac)
        spo2 = 110 - 25 * (red_absorbance / ir_absorbance)-36.5
        return spo2
    else:
        return 0

def apply_low_pass_filter(data, prev_filtered_data):
    filtered_data = [(1 - ALPHA) * prev + ALPHA * curr for prev, curr in zip(prev_filtered_data, data)]
    return filtered_data

prev_red_filtered = [0] * BUFFER_SIZE
prev_ir_filtered = [0] * BUFFER_SIZE

while True:
    # The check() method has to be continuously polled to check for new readings
    sensor.check()
    if button_return_main.value()==0:
        sensor.shutdown()
        del button_return_main
        del sensor
        del TH
        del fb
        gc.collect()
        machine.reset()
        utime.sleep(3)
        raise Return_To_Main()
    
    
    #Display and Heart Rate icon Setup
    font_writer=writer.Writer(display,freesans20)
    TH=bytearray(b'\xff\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00`\r\x00\x01\xfc?\x80\x07\xfe\x7f\xe0\x07\xff\xff\xe0\x0f\xff\xff\xf0\x0f\xff\xff\xf0\x0f\xfe\xff\xf0\x0f\xfe\x7f\xf0\x0f\xfe\x7f\xf0\x0f\xe6\x7f\xf0\x07\xe5\x7f\xe0\x07\xe5g\xe0\x02\x99\xa2\x80\x00\xbb\xa8\x80\x01\xf9\x9f\x80\x00\xff\x9f\x00\x00\x7f\xde\x00\x00?\xfc\x00\x00\x1f\xf8\x00\x00\x07\xe0\x00\x00\x03\xc0\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
    fb = framebuf.FrameBuffer(TH,32,32, framebuf.MONO_HLSB)
    
    # Check if the storage contains available samples
    if sensor.available():
        # Access the storage FIFO and gather the readings (integers)
        red_reading = sensor.pop_red_from_storage()
        ir_reading = sensor.pop_ir_from_storage()

        # Apply low-pass filter to the red and IR readings
        red_filtered = apply_low_pass_filter([red_reading], prev_red_filtered)[-1]
        ir_filtered = apply_low_pass_filter([ir_reading], prev_ir_filtered)[-1]

        # Update the previous filtered data
        prev_red_filtered.append(red_filtered)
        prev_red_filtered = prev_red_filtered[1:]  # Shift the buffer
        prev_ir_filtered.append(ir_filtered)
        prev_ir_filtered = prev_ir_filtered[1:]  # Shift the buffer

        # Print the acquired data
        print("Red: {}, IR: {}".format(red_filtered, ir_filtered))
        
        if(ir_filtered>230):
            # Image Display
            display.fill(0)
            display.blit(fb,0,20)
            # Update the buffer with new data
            buffer.append(red_filtered)
            buffer = buffer[1:]  # Shift the buffer

            # Find peaks in the buffer
            peaks = find_peaks(buffer)

            # Update peak indices, min_distances, and max_distances
            if len(peaks) > 0:
                peak_indices.append(peaks)
                if len(peak_indices) > 1:
                    min_distances.append(peaks[0] - peak_indices[-2][-1])
                    max_distances.append(peaks[0] - peak_indices[-2][0])

            # Update peak thresholds
            if len(min_distances) > 0:
                peak_threshold = min_threshold * min(min_distances)
            if len(max_distances) > 0:
                max_threshold = max_threshold * max(max_distances)

            # Check if a peak is detected
            if len(peaks) > 0 and red_filtered > peak_threshold:
                current_peak_index = peaks[0]

                # Calculate the BPM when a new peak is detected
                if current_peak_index != last_peak_index:
                    bpm = calculate_bpm(peak_indices[-1])
                    if bpm != 0:
                        beat_avg = bpm
                        display.text("BPM: {}".format(format_reading(beat_avg)[:3]),50,20)
                        display.show()
                        print("BPM: {}".format(format_reading(beat_avg)[:3]))  # Print up to 3 digits
                    last_peak_index = current_peak_index

                    # Calculate SpO2
                    if len(peaks) > 0:
                        red_ac = max(buffer) - min(buffer)
                        ir_ac = max(peaks) - min(peaks)
                        red_dc = sum(buffer) / len(buffer)
                        ir_dc = sum(peaks) / len(peaks)
                        spo2 = calculate_spo2(red_ac, red_dc, ir_ac, ir_dc)
                    if(spo2<100):
                        display.text("SpO2: {:.2f}%".format(spo2),35,40)
                        display.show()
                        print("SpO2: {:.2f}%".format(spo2))
                    else:
                        display.text("SpO2: 99%",35,40)
                        display.show()
                        
        else:
            display.fill(0)
            font_writer.set_textpos(0,10)
            font_writer.printstring("Keep Finger   on Sensor")
            display.show()
        utime.sleep(1)  # Delay between updates

