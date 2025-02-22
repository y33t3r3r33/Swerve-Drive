import wpilib
from rplidar import RPLidar

class Lidar():
    def __init__(self):
        self.lidar = RPLidar('/dev/ttyUSB0')
        self.lidar.get_info()

    def Scan(self):
        scandata = self.lidar.iter_scans(max_scan_meas=500)
        for scan in scandata:
            for(_, angle, distance) in scan:
                #print(f"Angle: {angle}, Distance: {distance}")
             pass
