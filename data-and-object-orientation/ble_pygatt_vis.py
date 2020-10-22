import pygatt
from bleak import BleakScanner
import asyncio
import binascii
import ctypes as ct
from math import sqrt, atan, degrees, atan2

from constants import *

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

delta_pitch = 0
pitch = 0
roll = 0
delta_roll = 0
update = False

def Cube():
    glBegin(GL_QUADS)
    for i, surface in enumerate(surfaces):
        for vertex in surface:
            glColor3fv(colors[i])
            glVertex3fv(vertices[vertex])
    glEnd()


    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

MAC_ADDRESS = None
ADDRESS_TYPE = pygatt.BLEAddressType.random

def twos_comp(val, n_bytes):
    val = int(val, 16)
    b = val.to_bytes(n_bytes, byteorder="big")                                                          
    return int.from_bytes(b, byteorder="big", signed=True)

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def force_handler(handle, value):
    print(f"Force -> 1: ({twos_comp(''.join(reversed(''.join(reversed(value.hex()[0:2])) + ''.join(reversed(value.hex()[2:4])))), 2)}), 2: ({twos_comp(''.join(reversed(''.join(reversed(value.hex()[4:6])) + ''.join(reversed(value.hex()[6:8])))), 2)})")


def accel_handler(handle, value):
    global pitch
    global roll
    global delta_pitch
    global delta_roll
    global update

    x = twos_comp(value.hex()[0:4], 2)  /  MPU_ACCEL_READINGSCALE_2G
    y = twos_comp(value.hex()[4:8], 2)  /  MPU_ACCEL_READINGSCALE_2G
    z = twos_comp(value.hex()[8:12], 2) /  MPU_ACCEL_READINGSCALE_2G
    
    sign_z = lambda z: 1 if z >= 0 else -1

    try:
        prev_pitch = pitch
        prev_roll = roll
        
        calc_y = (-1) * x
        calc_x = sign_z(z)*sqrt(0.01*y*y + z*z)
        pitch = degrees(atan2(calc_y , calc_x ))
        delta_pitch = pitch - prev_pitch
        calc_y = y
        calc_x = sqrt(z*z + x*x)
        roll = degrees(atan2(calc_y , calc_x ))
        delta_roll = roll - prev_roll
    
        print(f"Roll: {pitch}º --- Pitch: {roll}º\n")

        update = True
    
    except ZeroDivisionError:
        return

    print(f"ACELERÒMETRO -> X: ({x}), Y: ({y}), Z: ({z})\n")

    if z >= 0.5:
        print("Position -> Supine")

    elif z <= -0.5:
        print("Position -> Prone")

    else:
        if x >= 0.5:
            print("Position -> RLR")

        elif x <= -0.5:
            print("Position -> LLR")

def gyro_handler(handle, value):
    print(f"GIROSCÓPIO -> X: ({twos_comp(value.hex()[0:4], 2) / MPU_GYRO_READINGSCALE_250DEG}), Y: ({twos_comp(value.hex()[4:8], 2) / MPU_GYRO_READINGSCALE_250DEG}), Z: ({twos_comp(value.hex()[8:12], 2) / MPU_GYRO_READINGSCALE_250DEG})")

async def discover():
    scanner = BleakScanner()
    await scanner.start()
    await asyncio.sleep(2.0)
    await scanner.stop()
    devices = await scanner.get_discovered_devices()

    global MAC_ADDRESS

    for d in devices:
        if d.name == "BRUXISMO":
            MAC_ADDRESS = d.address
            print(f"        {bcolors.BOLD} DEVICE FOUND {bcolors.END} -> {bcolors.OKGREEN}{bcolors.BOLD}{MAC_ADDRESS}{bcolors.END}\n")


def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(discover())
    
    global MAC_ADDRESS
    global delta_pitch
    global delta_roll
    global update

    if MAC_ADDRESS != None:
        adapter = pygatt.GATTToolBackend()
        try:
            adapter.start()
            
            device = adapter.connect(MAC_ADDRESS, address_type = ADDRESS_TYPE)

            for uuid in device.discover_characteristics().keys():
                if uuid in BRUXISM_UUIDS:
                    print(f"{bcolors.HEADER} CHARACTERISTIC {bcolors.END}\n")
                    print(f"    {bcolors.BOLD}{bcolors.OKGREEN} UUID {bcolors.END} -> {bcolors.BOLD}{bcolors.OKBLUE}{uuid}{bcolors.END}\n")
                    print(f"    {bcolors.BOLD}{bcolors.OKGREEN} VALUE {bcolors.END} -> {bcolors.BOLD}{bcolors.OKBLUE}{binascii.hexlify(device.char_read(uuid))}{bcolors.END}\n")
        
            
            pygame.init()
            display = (800, 600)
            pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

            gluPerspective(90, (display[0]/display[1]), 0.1, 50.0)

            glTranslatef(0.0, 0.0 , -10.0)

            glRotatef(0, 0.0, 0.0, 0.0)
            
            device.subscribe(ACCEL_UUID, callback=accel_handler)
            device.subscribe(FORCE_UUID, callback=force_handler)
            #device.subscribe(GYRO_UUID, callback=gyro_handler)
            
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        device.unsubscribe(ACCEL_UUID)
                        device.unsubscribe(FORCE_UUID)
                        adapter.stop()
                        quit()
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 4:
                            glTranslatef(0,0,1.0)
                        if event.button == 5:
                            glTranslatef(0,0,-1.0)
                
                if update:
                    glTranslatef(0.0, 0.0 , 0.0)
                    glRotatef(delta_roll, 1.0, 0.0, 0.0)
                    glRotatef(delta_pitch, 0.0, 0.0, -1.0)
                    update = False
                
                glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
                Cube()
                pygame.display.flip()
                pygame.time.wait(10)
            

        except Exception as e:
            print(e)
        finally:
            adapter.stop()

if __name__ == "__main__":
    main()