import pygatt
from bleak import BleakScanner
import asyncio
import binascii
import ctypes as ct
from math import sqrt, atan, degrees, atan2
import signal
import sys

from constants import *
from orientation import *

MAC_ADDRESS = None
ADDRESS_TYPE = pygatt.BLEAddressType.random

device = None
adapter = None

def force_handler(handle, value):
    print(f"Force -> 1: ({ComplementaryFilter.twos_comp(''.join(reversed(''.join(reversed(value.hex()[0:2])) + ''.join(reversed(value.hex()[2:4])))), 2)}), 2: ({ComplementaryFilter.twos_comp(''.join(reversed(''.join(reversed(value.hex()[4:6])) + ''.join(reversed(value.hex()[6:8])))), 2)})")

async def signal_handler(sig, frame):
    global adapter
    global device

    print("\nCatched CTRL-C\n")
    #await device.unsubscribe(ACCEL_UUID)
    await device.unsubscribe(FORCE_UUID)
    #device.unsubscribe(GYRO_UUID)
    await adapter.stop()
    sys.exit(0)

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
    global adapter
    global device

    filter = ComplementaryFilter(False)
    signal.signal(signal.SIGINT, signal_handler)
    
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
        
            device.subscribe(FORCE_UUID, callback=force_handler)
            #device.subscribe(ACCEL_UUID, callback=filter.accel_handler)
            #device.subscribe(GYRO_UUID, callback=filter.gyro_handler)
            
            loop = asyncio.get_event_loop()
            loop.run_until_complete(signal.pause())

        except Exception as e:
            print(e)

if __name__ == "__main__":
    main()