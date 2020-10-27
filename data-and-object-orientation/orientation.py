from constants import MPU_ACCEL_READINGSCALE_2G, MPU_GYRO_READINGSCALE_250DEG
from math import sqrt, degrees, atan2

class ComplementaryFilter:
    
    accel_x = 0
    accel_y = 0
    accel_z = 0

    def __init__(self, gyro = True, alfa = 0):
        self.gyro_enabled = gyro
        self.alfa = alfa

        self.pitch_accel = 0 
        self.roll_accel = 0

        if self.gyro_enabled:
            self.pitch_gyro = 0 
            self.roll_gyro = 0
            self.yaw_gyro = 0

        self.pitch = 0
        self.roll = 0

        self.prev_pitch = 0
        self.prev_roll = 0
        self.prev_yaw = 0
        
        self.delta_pitch = 0
        self.delta_roll = 0
        self.delta_yaw = 0

        self.update = False

    def accel_handler(self, handle, value):

        self.accel_x = self.twos_comp(value.hex()[0:4], 2)  /  MPU_ACCEL_READINGSCALE_2G
        self.accel_y = self.twos_comp(value.hex()[4:8], 2)  /  MPU_ACCEL_READINGSCALE_2G
        self.accel_z = self.twos_comp(value.hex()[8:12], 2) /  MPU_ACCEL_READINGSCALE_2G

        #print(f"Accelerometer -> X: {self.accel_x}, Y: {self.accel_y}, Z: {self.accel_z}\n")
        
        sign_z = lambda z: 1 if z >= 0 else -1

        try:
            self.prev_pitch = self.pitch
            self.prev_roll = self.roll

            if self.gyro_enabled:
                self.prev_yaw = self.yaw_gyro

            calc_num = (-1) * self.accel_x
            calc_den = sign_z(self.accel_z)*sqrt(0.1*self.accel_y*self.accel_y + self.accel_z*self.accel_z)
            self.pitch_accel = degrees(atan2(calc_num , calc_den))

            if not self.gyro_enabled:
                self.pitch = self.pitch_accel
                self.delta_pitch = self.pitch - self.prev_pitch
            
            calc_num = self.accel_y
            calc_den = sqrt(self.accel_z*self.accel_z + self.accel_x*self.accel_x)
            self.roll_accel = degrees(atan2(calc_num , calc_den))
        
            if not self.gyro_enabled:
                self.roll = self.roll_accel
                self.delta_roll = self.roll - self.prev_roll

            print(f"\nRoll: {round(self.pitch_accel, 2)}º --- Pitch: {round(self.roll_accel, 2)}º\n")

            self.update = True * (not self.gyro_enabled)
        
        except ZeroDivisionError:
            return

        if self.accel_z >= 0.5:
            print("Position -> Supine")

        elif self.accel_z <= -0.5:
            print("Position -> Prone")

        else:
            if self.accel_x >= 0.5:
                print("Position -> RLR")

            elif self.accel_x <= -0.5:
                print("Position -> LLR")

            elif self.accel_y >= 0.7:
                print("Position -> Fowler's")
            
            elif self.accel_y <= -0.5:
                print("Position -> Trendelenberg")

    def gyro_handler(self, handle, value):
        gyro_x = self.twos_comp(value.hex()[0:4], 2) / MPU_GYRO_READINGSCALE_250DEG - 6.75
        gyro_y = self.twos_comp(value.hex()[4:8], 2) / MPU_GYRO_READINGSCALE_250DEG - 2.15
        gyro_z = self.twos_comp(value.hex()[8:12], 2) / MPU_GYRO_READINGSCALE_250DEG + 0.12
        
        self.pitch_gyro = gyro_y + self.prev_pitch
        self.roll_gyro = gyro_x + self.prev_roll
        self.yaw_gyro = gyro_z + self.prev_yaw

        self.pitch = (1 - self.alfa) * self.pitch_accel + self.alfa * self.pitch_gyro
        self.roll = (1 - self.alfa) * self.roll_accel + self.alfa * self.roll_gyro

        self.delta_roll = self.roll - self.prev_roll
        self.delta_pitch = self.pitch - self.prev_pitch
        self.delta_yaw = self.yaw_gyro - self.prev_yaw

        self.update = True
        
        print(f"IMU -> X: ({round(self.pitch, 2)}º, Y: ({round(self.roll, 2)}))º, Z: ({round(self.yaw_gyro, 2)})º")

    @staticmethod
    def twos_comp(val, n_bytes):
        val = int(val, 16)
        b = val.to_bytes(n_bytes, byteorder="big")                                                          
        return int.from_bytes(b, byteorder="big", signed=True)