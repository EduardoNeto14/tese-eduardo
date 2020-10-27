import pygatt
from bleak import BleakScanner
import asyncio
import binascii
import ctypes as ct
from math import sqrt, atan, degrees, atan2

from constants import *
from orientation import *

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

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

def force_handler(handle, value):
    print(f"Force -> 1: ({ComplementaryFilter.twos_comp(''.join(reversed(''.join(reversed(value.hex()[0:2])) + ''.join(reversed(value.hex()[2:4])))), 2)}), 2: ({ComplementaryFilter.twos_comp(''.join(reversed(''.join(reversed(value.hex()[4:6])) + ''.join(reversed(value.hex()[6:8])))), 2)})")

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
    filter = ComplementaryFilter(False)

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

            device.subscribe(FORCE_UUID, callback=force_handler)
            device.subscribe(ACCEL_UUID, callback=filter.accel_handler)
            #device.subscribe(GYRO_UUID, callback=filter.gyro_handler)
            
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        device.unsubscribe(ACCEL_UUID)
                        device.unsubscribe(FORCE_UUID)
                        #device.unsubscribe(GYRO_UUID)
                        adapter.stop()
                        quit()
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 4:
                            glTranslatef(0,0,1.0)
                        if event.button == 5:
                            glTranslatef(0,0,-1.0)
                
                if filter.update:
                    glTranslatef(0.0, 0.0 , 0.0)
                    glRotatef(filter.delta_roll, 1.0, 0.0, 0.0)
                    glRotatef(filter.delta_pitch, 0.0, 0.0, -1.0)
                    
                    if filter.gyro_enabled:
                        glRotatef(filter.delta_yaw, 0.0, 1.0, 0.0)
                    
                    filter.update = False
                
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