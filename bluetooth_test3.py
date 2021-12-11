#!/usr/bin/python3 -Wignore
import os,sys
import time
import asyncio
from bleak import BleakClient
from typing import Callable, Any

class connection:
    
    client: BleakClient = None

    def __init__(self, loop: asyncio.AbstractEventLoop, \
            read_characteristic: str, dir_characteristic: str ):
        self.loop = loop
        self.read_characteristic = read_characteristic
        self.dir_characteristic  = dir_characteristic
        self.connected = False

    def on_disconnect(self, client: BleakClient):
        self.connected = False
        print("diconnected \n")

    async def cleanup(self):
        if self.client:
            await self.client.disconnect()

    async def manager(self):
        print("Starting connection manager \n")
        while True:
            if self.client:
                await self.connect()
            else:
                await self.select_device()
                await asyncio.sleep(15.0,loop=loop)

    async def connect(self):
        if self.connected:
            return
        print("trying to connect \n")
        try:
            await self.client.connect()
            self.connected = await self.client.is_connected()
            if self.connected:
                print("Connected to Ed's mouse \n")
                self.client.set_disconnected_callback(self.on_disconnect)
                await self.client.start_notify(self.read_characteristic, \
                        self.notification_handler)
                value = await self.client.read_gatt_char(self.read_characteristic)
                print(value)
                while True:
                    if not self.connected:
                        break
                    await asyncio.sleep(3.0, loop=loop)
            else:
                print("failed to connect \n")
        except Exception as e:
            print("Leaving \n")

    async def select_device(self):
        mouse1_addr    = "60:CF:2D:45:3B:1B"
        self.connected_device = mouse1_addr
        self.client = BleakClient(mouse1_addr, loop=self.loop)

    async def notification_handler(self, sender: str, data: Any):
        #value = await self.client.read_gatt_char(self.read_characteristic)
        print(data)
        if data == b'\x01':
            value = await self.client.read_gatt_char(self.dir_characteristic)
            directions = str(value)
            shortened = self.shortenDirections(directions)
            print(shortened)
            b = bytearray()
            b.extend(map(ord,shortened))
            #await self.client.write_gatt_char(self.read_characteristic,value,True)


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
                     EOS = True
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



async def main():
    while True:
        await asyncio.sleep(5)



if __name__ == "__main__":

    #create event loop
    loop = asyncio.get_event_loop()
    flag_characteristic = "0fe79935-cd39-480a-8a44-06b70f36f24a"
    dir_characteristic = "0fe79935-cd39-480a-8a44-06b70f36f24c"

    connection = connection(loop, flag_characteristic, dir_characteristic)

    try:
        asyncio.ensure_future(connection.manager())
        asyncio.ensure_future(main())
        loop.run_forever()
    except KeyboardInterrupt:
        print("User stopped program")

    finally:
        print("disconnecting...\n")
        loop.run_until_complete(connection.cleanup())


