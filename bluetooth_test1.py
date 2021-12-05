#!/usr/bin/python3
import os,sys
import time
import asyncio
from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData
from bleak import BleakScanner
from bleak import BleakClient

class bluetooth_test1():


    def __init__(self):
        self.mouse1_addr    = "60:CF:2D:45:3B:1B"
        self.mouse2_addr    = "AA:AA:AA:AA:AA:AA"
        self.mouse3_addr    = "BB:BB:BB:BB:BB:BB"
        self.mouse1_avail   = 0
        self.mouse2_avail   = 0
        self.mouse3_avail   = 0
        self.got_directions = 0
        self.on             = 0


    def simple_callback(self, device: BLEDevice, 
                        advertisement_data: AdvertisementData):

        print(device.address)
        self.on = 1
        if device.address == self.mouse1_addr:
            if self.mouse1_avail == 0:
                print("found Ed's Mouse\n")
                self.mouse1_avail = 1
        elif device.address == self.mouse2_addr:
            if self.mouse2_avail == 0:
                print("found Zach's Mouse :)")
                self.mouse2_avail = 1
        elif device.address == self.mouse3_addr:
            if self.mouse3_avail == 0:
                print("found Mochi's Mouse")
                self.mouse3_avail = 1
        else:
            self.on = 0
        

    async def run_ble_client1(self, address: str, queue: asyncio.Queue):
        async def callback_handler(sender, data):
            await queue.put(time.time(),data)


    def start(self):
        self.queue = asyncio.Queue()
        loop = asyncio.get_event_loop()
        self.future = asyncio.ensure_future(self.run())
        loop.run_until_complete(self.future) 

    async def run(self):
        scanner = BleakScanner()
        scanner.register_detection_callback(self.simple_callback)
        
        try:
            while True:
                await scanner.start()
                if self.on:
                    print("if self.on")
                    if self.mouse1_avail == 1:
                        await self.run_ble_client1(self.mouse1_addr,self.queue)
                        self.mouse1_avail = 2

                    try:
                        print("before await\n")
                        epoch,data = await asyncio.wait_for(self.queue.get(), timeout=1.0)
                        print("after await \n")
                        if data != None:
                            print(data)
                            print("\nData should have been printed\n")
                        else:
                            asyncio.sleep(1.0)
                    except Exception as err:
                        print(err)

                asyncio.sleep(1.0)
                await scanner.stop()
        except KeyboardInterrupt:
            print("EXIT!!")

                


if __name__== '__main__':
    test1_proc = bluetooth_test1()
    test1_proc.start()

