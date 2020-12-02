from constants import MPU_ACCEL_READINGSCALE_2G, MPU_GYRO_READINGSCALE_250DEG
from math import sqrt, degrees, atan2
import time
import csv

class ComplementaryFilter:
    
    accel_x = 0
    accel_y = 0
    accel_z = 0

    T_LOW_RES = 0.5
    T_HIGH_RES = 0.05

    def __init__(self, gyro = False, alfa = 0.75):
        self.gyro_enabled = gyro
        self.alfa = alfa
        self.accel_enabled = False

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

        self.position = False

    def accel_handler(self, handle, value):
        self.accel_enabled = True

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

            calc_num = self.accel_y
            calc_den = sqrt(self.accel_z*self.accel_z + self.accel_x*self.accel_x)
            self.pitch_accel = degrees(atan2(calc_num , calc_den))
            
            if not self.gyro_enabled:
                self.pitch = self.pitch_accel
                self.delta_pitch = self.pitch - self.prev_pitch
            
            calc_num = ( (-1) / sign_z(self.accel_z) ) * self.accel_x   \
                if self.pitch_accel > 70 or self.pitch_accel < -70      \
                else ( (-1) ) * self.accel_x   \
            
            calc_den = sqrt(0.1*self.accel_y*self.accel_y + self.accel_z*self.accel_z)  \
                if self.pitch_accel > 70 or self.pitch_accel < -70                      \
                else sign_z(self.accel_z) * sqrt(0.1*self.accel_y*self.accel_y + self.accel_z*self.accel_z)
            
            self.roll_accel = degrees(atan2(calc_num , calc_den))
            
            if not self.gyro_enabled:
                self.roll = self.roll_accel
                self.delta_roll = self.roll - self.prev_roll

            print(f"\nPitch: {round(self.pitch_accel, 2)}º --- Roll: {round(self.roll_accel, 2)}º\n")

            self.update = True * (not self.gyro_enabled)

            if not self.gyro_enabled:
                with open("data/accel.csv", "a") as accel:
                    accel = csv.writer(accel, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    accel.writerow([f'{time.time()}', f'{self.accel_x}', f'{self.accel_y}', f'{self.accel_z}', f'{self.roll}', f'{self.pitch}'])
        
        except ZeroDivisionError:
            return

    def gyro_handler(self, handle, value):
        gyro_x = self.twos_comp(value.hex()[0:4], 2) / MPU_GYRO_READINGSCALE_250DEG - 6.75
        gyro_y = self.twos_comp(value.hex()[4:8], 2) / MPU_GYRO_READINGSCALE_250DEG - 2.15
        gyro_z = self.twos_comp(value.hex()[8:12], 2) / MPU_GYRO_READINGSCALE_250DEG + 0.12
        
        self.pitch_gyro = gyro_x*(self.T_HIGH_RES if self.accel_enabled else self.T_LOW_RES) + self.prev_pitch
        self.roll_gyro = gyro_y*(self.T_HIGH_RES if self.accel_enabled else self.T_LOW_RES) + self.prev_roll
        self.yaw_gyro = gyro_z*(self.T_HIGH_RES if self.accel_enabled else self.T_LOW_RES) + self.prev_yaw

        self.pitch = (1 - self.alfa) * self.pitch_accel + self.alfa * self.pitch_gyro
        self.roll = (1 - self.alfa) * self.roll_accel + self.alfa * self.roll_gyro

        self.delta_roll = self.roll - self.prev_roll
        self.delta_pitch = self.pitch - self.prev_pitch
        self.delta_yaw = self.yaw_gyro - self.prev_yaw

        self.update = True
        
        if -45 <= self.roll <= 45 and -25 <= self.pitch <= 35:
            self.position = "Supine"
            print("Position -> Supine")

        elif (125 <= self.roll <= 180 or -180 <= self.roll <= -125) and -35 <= self.pitch <= 35:
            self.position = "Prone"
            print("Position -> Prone")

        else:
            if 45 < self.roll <= 125:
                self.position = "LLR"
                print("Position -> LLR")

            elif -125 < self.roll <= -45:
                self.position = "RLR"
                print("Position -> RLR")

            elif self.pitch > 35:
                self.position = "Fowler's"
                print("Position -> Fowler's")
            
            elif self.pitch < -25:
                self.position = "Trendelenberg"
                print("Position -> Trendelenberg")

            else:
                self.position = "Undefined"
            
        if self.accel_enabled:
            with open("data/combined.csv", "a") as gyro:
                gyro = csv.writer(gyro, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
                gyro.writerow([f'{time.time()}', f'{self.pitch_accel}', f'{self.roll_accel}', f'{self.pitch_gyro}', f'{self.roll_gyro}', f'{self.yaw_gyro}', f'{self.pitch}', f'{self.roll}', f'{self.position}'])

        else:
            with open("data/gyro.csv", "a") as gyro:
                gyro = csv.writer(gyro, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
                gyro.writerow([f'{time.time()}', f'{gyro_x}', f'{gyro_y}', f'{gyro_z}'])

        print(f"IMU -> Pitch: ({round(self.pitch, 2)}º, Roll: ({round(self.roll, 2)}))º")

    @staticmethod
    def twos_comp(val, n_bytes):
        val = int(val, 16)
        b = val.to_bytes(n_bytes, byteorder="big")                                                          
        return int.from_bytes(b, byteorder="big", signed=True)
