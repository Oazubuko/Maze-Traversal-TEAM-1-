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

    async def manager(self,selector: int):
        print("Starting connection manager \n")
        while True:
            if self.client:
                await self.connect()
            else:
                await self.select_device(selector)
                await asyncio.sleep(15.0,loop=loop)

    async def connect(self):
        if self.connected:
            return
        print("trying to connect \n")
        try:
            await self.client.connect()
            self.connected = await self.client.is_connected()
            if self.connected:
                print(self.connected_device)
                if self.connected_device == "60:CF:2D:45:3B:1B": 
                    print("Connected to Ed's Mouse \n")
                elif self.connected_device == "5B:FD:4B:44:C7:3E": 
                    print("Connected to Zach's Mouse \n")
                elif self.connected_device == "B0:83:0B:D4:2B:70":
                    print("Connected to Machi's Mouse \n")
                else:
                    print("Connected to Invaild Device \n")
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

    async def select_device(self,selector: int):
        mouse_addr_Ed    = "60:CF:2D:45:3B:1B"
        mouse_addr_Zach    = "5B:FD:4B:44:C7:3E"
        mouse_addr_Machi    = "B0:83:0B:D4:2B:70"
        if selector == 1: 
        	self.connected_device = mouse_addr_Ed
        	self.client = BleakClient(mouse_addr_Ed, loop=self.loop)
        elif selector == 2: 
        	self.connected_device = mouse_addr_Zach
        	self.client = BleakClient(mouse_addr_Zach, loop=self.loop)
        elif selector == 3: 
        	self.connected_device = mouse_addr_Mochi
        	self.client = BleakClient(mouse_addr_Machi, loop=self.loop)
        else: 
                print("Invalid Device Selected \n")

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
    Ed_flag_characteristic = "0fe79935-cd39-480a-8a44-06b70f36f24a"
    Ed_dir_characteristic = "0fe79935-cd39-480a-8a44-06b70f36f24c"
    Zach_flag_characteristic = "1fe79935-cd39-480a-8a44-06b70f36f24a"
    Zach_dir_characteristic = "1fe79935-cd39-480a-8a44-06b70f36f24c"
    Machi_flag_characteristic = "2fe79935-cd39-480a-8a44-06b70f36f24a"
    Machi_dir_characteristic = "2fe79935-cd39-480a-8a44-06b70f36f24c"

    connection_Ed = connection(loop, Ed_flag_characteristic, Ed_dir_characteristic)
    connection_Zach = connection(loop, Zach_flag_characteristic, Zach_dir_characteristic)
    #connection_Machi = connection(loop, Machi_flag_characteristic, Machi_dir_characteristic)

    try:
        asyncio.ensure_future(connection_Ed.manager(1))
        asyncio.ensure_future(connection_Zach.manager(2))
        #asyncio.ensure_future(connection_Machi.manager(3))
        asyncio.ensure_future(main())
        loop.run_forever()
    except KeyboardInterrupt:
        print("User stopped program")

    finally:
        print("disconnecting...\n")
        loop.run_until_complete(connection.cleanup())


