import bitalino
import numpy
import serial

ser = serial.Serial()
ser.port = "COM3"
ser.open()


macAddress = "20:18:05:28:62:83"
   
device = bitalino.BITalino(macAddress)

srate = 1000
nframes = 100
threshold = 5

device.start(srate, [0])
print ("START")

try:
    while True:

        data = device.read(nframes)
        
        if numpy.mean(data[:, 1]) < 1: break

        EMG = data[:, -1]
        
        envelope = numpy.mean(abs(numpy.diff(EMG)))
        if envelope > threshold:
            device.trigger([0, 1])
            sig = 1
        else:
            device.trigger([0, 0])
            sig = 0
        ser.write(str(sig).encode("utf-8"))

finally:
    print ("STOP")
    device.trigger([0, 0])
    device.stop()
    device.close()
    ser.close()