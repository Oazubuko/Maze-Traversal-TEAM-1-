#!/usr/bin/python3
import os,sys
import time
import asyncio
from bleak import BleakScanner
from bleak import BleakClient
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData
from bleak import BleakScanner
from bleak import BleakClient

class bluetooth_test1():


    def __init__(self):
        self.mouse1_addr    = "60:CF:2D:45:3B:1B"
        self.dirs_CHAR_UUID = "0fe79935-cd39-480a-8a44-06b70f36f24c"

        self.mouse2_addr    = "5B:FD:4B:44:C7:3E"
        self.mouse3_addr    = "B0:83:0B:D4:2B:70"
        self.mouse1_avail   = 0
        self.mouse2_avail   = 0
        self.mouse3_avail   = 0
        self.got_directions = 0
        self.on             = 0


    def simple_callback(self, device: BLEDevice, 
                        advertisement_data: AdvertisementData):

        self.on = 1
        if device.address == self.mouse1_addr:
            if self.mouse1_avail == 0:
                print("found Ed's Mouse \n")
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
                    if self.mouse1_avail == 1:
                        try:
                            async with BleakClient(self.mouse1_addr) as client:
                                value = await client.read_gatt_char\
                                     (self.dirs_CHAR_UUID)
                                directions = str(value)
                                shortened =self.shortenDirections(directions)

                                print("directions %s \n" %directions)
                                print("shortened %s \n" %shortened)
                                b=bytearray()
                                b.extend(map(ord,shortened))
                                print(b)
                                await client.write_gatt_char\
                                    (self.dirs_CHAR_UUID,'The End',True)
                                self.mouse1_avail = 2
                        except:
                                print("NOT THIS TIME")
                                self.mouse1_avail = 0
                                self.on  = 0
                            
                asyncio.sleep(1.0)
                await scanner.stop()
        except KeyboardInterrupt:
            print("EXIT!!")

    def shortenDirections(self,dirStr):
        lendirStr  = len(dirStr)
        match      = 1
        if lendirStr < 3: 
            match = True
        while match == True:
            match = False
            EOS   = False
            indx  = 1
            while match == False and EOS == False:
                substr         = dirStr[indx-1:indx+2]
                match,replace  = self.check4match(substr)
                if match == True:
                    dirStr = self.rewrite(dirStr,indx,replace)
                    indx = 1
                else:
                    indx = indx + 1
                    if indx+2 > lendirStr:
                     EOS = 1
            return dirStr


    def check4match(self,substr):
        match   = False
        replace = ""
        if substr == "LBL":
            replace = "S"
        elif substr == "LBS":
            replace = "R"
        elif substr == "RBL":
            replace = "B"
        elif substr == "SBS":
            replace = "B"
        elif substr == "SBL":
            replace = "R"
        elif substr == "LBR":
            replace = "B"

        if replace != "":
            match = True

        return [match,replace]
                

    def rewrite(self,dirStr, indx, replace):
        if indx == 1:
            tmpStr = replace
        else:
            tmpStr = dirStr[0:indx-1]+replace

        lendirStr = len(dirStr)
    
        if indx+2 < lendirStr:
            tmpStr = tmpStr + dirStr[indx+2:lendirStr]
        return tmpStr


if __name__== '__main__':
    test1_proc = bluetooth_test1()
    test1_proc.start()

