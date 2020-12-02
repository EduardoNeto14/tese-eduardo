import pygatt
from bleak import BleakScanner
import asyncio
import binascii
import ctypes as ct
from math import sqrt, atan, degrees, atan2
import signal
import sys
import time
import csv
import keyboard
from threading import Thread

from constants import *
from orientation import *

MAC_ADDRESS = None
ADDRESS_TYPE = pygatt.BLEAddressType.random

#device = None
adapter = None

filter = ComplementaryFilter()

force = False

i = 0

def force_handler(handle, value):
    global i
    i+=1

    f1_val = ComplementaryFilter.twos_comp(''.join(reversed(''.join(reversed(value.hex()[0:2])) + ''.join(reversed(value.hex()[2:4])))), 2)
    f2_val = ComplementaryFilter.twos_comp(''.join(reversed(''.join(reversed(value.hex()[4:6])) + ''.join(reversed(value.hex()[6:8])))), 2)
    print(f"Force {i}-> 1: ({f1_val}), 2: ({f2_val})")
    
    with open("data/force_another.csv", "a") as f2:
        f2 = csv.writer(f2, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
        f2.writerow([f'{time.time()}', f'{f1_val}', f'{f2_val}'])

async def discover():
    scanner = BleakScanner()
    await scanner.start()
    await asyncio.sleep(2.0)
    await scanner.stop()
    devices = await scanner.get_discovered_devices()

    global MAC_ADDRESS

    for d in devices:
        if d.name == "BRUXISM":
            MAC_ADDRESS = d.address
            print(f"        {bcolors.BOLD} DEVICE FOUND {bcolors.END} -> {bcolors.OKGREEN}{bcolors.BOLD}{MAC_ADDRESS}{bcolors.END}\n")

def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(discover())
    
    global MAC_ADDRESS
    global adapter

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
        
            
            #device.subscribe(ACCEL_UUID, callback=filter.accel_handler)
            #device.subscribe(GYRO_UUID, callback=filter.gyro_handler)
            #device.subscribe(FORCE_UUID, callback=force_handler)
            asyncio.run(detect_key_press(device))
            #x = asyncio.new_event_loop()
            #asyncio.set_event_loop(x)
            #asyncio.ensure_future(detect_key_press(x))
            #x.run_forever()

        except Exception as e:
            print(e)

async def detect_key_press(device):
    global force
    global filter
    print(device)

    while True:
        if keyboard.is_pressed("f"):
            if not force:
                print("Ativar Força")
                force = True
                
                #asyncio.create_task(force_enabled(device))
                device.subscribe(FORCE_UUID, callback=force_handler)
        
            else:
                print("Desativar Força")
                force = False
                #asyncio.create_task(force_disabled(device))
                device.unsubscribe(FORCE_UUID)
        
        if keyboard.is_pressed("a"):
            if not filter.accel_enabled:
                print("Ativar Acelerómetro")
                filter.accel_enabled = True

                device.subscribe(ACCEL_UUID, callback=filter.accel_handler)
                
            else:
                print("Desativar Acelerómetro")
                filter.accel_enabled = False
                
                device.unsubscribe(ACCEL_UUID)

        if keyboard.is_pressed("g"):
            if not filter.gyro_enabled:
                print("Ativar Giroscópio")
                filter.gyro_enabled = True
                device.subscribe(GYRO_UUID, callback=filter.gyro_handler)

            else:
                print("Desativar Giroscópio")
                filter.gyro_enabled = False
                device.unsubscribe(GYRO_UUID)

def force_enabled(device):
    device.subscribe(FORCE_UUID, callback=force_handler)

def force_disabled(device):
    device.unsubscribe(FORCE_UUID)

if __name__ == "__main__":
    main()