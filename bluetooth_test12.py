#!/usr/bin/python3 -Wignore
import os,sys
import time
import asyncio
import signal
import socket
from bleak import BleakClient
from typing import Callable, Any

global theTransport
global Connected

class EchoClientProtocol(asyncio.Protocol):

    def __init__(self, message, on_con_lost,Theconnection,loop):
        self.message = message
        self.on_con_lost = on_con_lost
        self.connection = Theconnection

    def connection_made(self, transport):
        self.transport = transport

    def data_received(self,data):
        message = data.decode()
        print("Client Message %s " %message)
        ipos = message.find("SDIR:")
        if ipos >= 0:
            shortenDir = message[ipos+5:]
            if shortenDir != "NONE":
                print("from client: %s" %shortenDir)
                task = asyncio.ensure_future(\
                self.connection.helpSendMessage(shortenDir))
        

    def connection_lost(self,exc):
        print('The server closed the connection')
        self.on_con_lost.set_result(True)

class connection:
    
    client: BleakClient = None

    def __init__(self, loop: asyncio.AbstractEventLoop, \
            read_characteristic: str, dir_characteristic: str ):
        self.loop = loop
        self.read_characteristic = read_characteristic
        self.dir_characteristic  = dir_characteristic
        self.connected = False
        self.mouse_addr_Ed    = "60:CF:2D:45:3B:1B"
        self.mouse_addr_Zach  = "35:BA:0C:91:CE:CC"
        self.mouse_addr_Machi = "86:8E:B7:B2:CB:B2"
        self.mouse_addr_Loaner = "2C:72:52:BE:CC:0D"
        global Connected
        Connected = False

    def on_disconnect(self, client: BleakClient):
        self.connected = False
        print("diconnected \n")

    async def cleanup(self):
        if self.client:
            await self.client.disconnect()

    async def manager(self,selector: int):
        print("Starting connection manager %d \n" %selector)
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
                global Connected
                Connected = True
                if self.connected_device == "60:CF:2D:45:3B:1B": 
                    print("Connected to Ed's Mouse \n")
                elif self.connected_device == "35:BA:0C:91:CE:CC": 
                    print("Connected to Zach's Mouse \n")
                elif self.connected_device == "B0:83:0B:D4:2B:70":
                    print("Connected to Machi's Mouse \n")
                elif self.connected_device == "86:8E:B7:B2:CB:B2":
                    print("Connected to Loaner Mouse \n")
                else:
                    print("Connected to Invaild Device \n")
                self.client.set_disconnected_callback(self.on_disconnect)
                await self.client.start_notify(self.read_characteristic, \
                        self.notification_handler)
                value = await self.client.read_gatt_char(self.read_characteristic)
                while True:
                    if not self.connected:
                        break
                    await asyncio.sleep(3.0, loop=loop)
            else:
                print("failed to connect \n")
                await asyncio.sleep(3.0)
        except Exception as e:
            await asyncio.sleep(3.0)

    async def select_device(self,selector: int):
        if selector == 1: 
        	self.connected_device = self.mouse_addr_Ed
        	self.client = BleakClient(self.mouse_addr_Ed, loop=self.loop)
        elif selector == 2: 
        	self.connected_device = self.mouse_addr_Zach
        	self.client = BleakClient(self.mouse_addr_Zach, loop=self.loop)
        elif selector == 3: 
        	self.connected_device = self.mouse_addr_Machi
        	self.client = BleakClient(self.mouse_addr_Machi, loop=self.loop)
        elif selector == 4: 
        	self.connected_device = self.mouse_addr_Loaner
        	self.client = BleakClient(self.mouse_addr_Loaner, loop=self.loop)
        else: 
                print("Invalid Device Selected \n")

    async def notification_handler(self, sender: str, data: Any):
        print ("IN NOTIFIFCATION: the Flag with data = ")
        print(data)
        if data == b'\x01':
            value = await self.client.read_gatt_char(self.dir_characteristic)
            directions = value.decode()
            print("orignal direction string %s " %directions)
            shortened = self.shortenDirections(directions)
            self.shortenedDir = shortened
            b = bytearray()
            b=b'\x03'
            await self.client.write_gatt_char(self.read_characteristic,b)
            global theTransport
            message = "SDIR:"+shortened
            print("Sending %s " %message)
            theTransport.write(message.encode())

        elif data == b'\x02':
             message = "send SDIR"
             theTransport.write(message.encode())

    async def helpSendMessage(self,shortenDir):
        b = bytearray()
        b.extend(map(ord,shortenDir))
        await  self.client.write_gatt_char(self.dir_characteristic,b)
        b = bytearray()
        b=b'\x04'
        await self.client.write_gatt_char(self.read_characteristic,b)


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
        print("the shortebed %s " %dirStr)
        return dirStr

    def check4match(self,substr):
        match   = False
        replace = ""
        if substr == "RBR":
            replace = "S"
        elif substr == "RBS":
            replace = "L"
        elif substr == "LBR":
            replace = "B"
        elif substr == "SBS":
            replace = "B"
        elif substr == "SBR":
            replace = "L"
        elif substr == "RBL":
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



async def main( loop,Theconnection):
    global theTransport

    on_con_lost = loop.create_future()
    message = b'SDIR:RL'

    theTransport, protocol = await loop.create_connection( \
                          lambda: EchoClientProtocol(message, \
                          on_con_lost,Theconnection,loop), \
                          '127.0.0.1',15136)

    try:
        await on_con_lost
    finally:
            theTransport.close()

    print("in main before true")
    while True:
        await asyncio.sleep(5)


def signalExitHandler(signum, frame):
    print('Signal Handler called with signal',signum)
    raise KeyboardInterrupt


if __name__ == "__main__":

    mouse    = sys.argv[1]

    signal.signal(signal.SIGTERM,signalExitHandler)

    mypid = os.getpid()


    if mouse == "Ed":
        affinity = {1}
        os.sched_setaffinity(0,affinity)
    elif mouse == "Zach":
        affinity = {2}
        os.sched_setaffinity(0,affinity)
    elif mouse == "Machi":
        affinity = {3}
        os.sched_setaffinity(0,affinity)
    elif mouse == "Loaner":
        affinity = {3}
        os.sched_setaffinity(0,affinity)

    affinity = os.sched_getaffinity(mypid)
    print(affinity)

    #create event loop
    loop = asyncio.get_event_loop()
    Ed_flag_characteristic = "0fe79935-cd39-480a-8a44-06b70f36f24a"
    Ed_dir_characteristic = "0fe79935-cd39-480a-8a44-06b70f36f24c"
    Zach_flag_characteristic = "1fe79935-cd39-480a-8a44-06b70f36f24a"
    Zach_dir_characteristic = "1fe79935-cd39-480a-8a44-06b70f36f24c"
    Machi_flag_characteristic = "2fe79935-cd39-480a-8a44-06b70f36f24a"
    Machi_dir_characteristic = "2fe79935-cd39-480a-8a44-06b70f36f24c"
    Loaner_flag_characteristic = "3fe79935-cd39-480a-8a44-06b70f36f24a"
    Loaner_dir_characteristic = "3fe79935-cd39-480a-8a44-06b70f36f24c"


    try:
        if mouse == "Ed":
            connection_Ed = connection(loop, Ed_flag_characteristic, \
                    Ed_dir_characteristic)
            asyncio.ensure_future(connection_Ed.manager(1))
            asyncio.ensure_future(main(loop,connection_Ed))
        elif mouse == "Zach":
            connection_Zach = connection(loop, Zach_flag_characteristic, \
                    Zach_dir_characteristic)
            asyncio.ensure_future(connection_Zach.manager(2))
            asyncio.ensure_future(main(loop,connection_Zach))
        elif mouse == "Machi":
            connection_Machi = connection(loop, Machi_flag_characteristic, \
                    Machi_dir_characteristic)
            asyncio.ensure_future(connection_Machi.manager(3))
            asyncio.ensure_future(main(loop,connection_Machi))
        elif mouse == "Loaner":
            connection_Loaner = connection(loop, Loaner_flag_characteristic, \
                    Loaner_dir_characteristic)
            asyncio.ensure_future(connection_Loaner.manager(4))
            asyncio.ensure_future(main(loop,connection_Loaner))

        try:
            loop.run_forever()
        except:
            print("shutdown confirmed!!")
    except KeyboardInterrupt:
        print("User stopped program")

    finally:
        global Connected
        if mouse == "Ed":
            if Connected:
                print("disconnecting...\n")
                loop.run_until_complete(connection_Ed.cleanup())
        elif mouse == "Zach":
            if Connected:
                print("disconnecting...\n")
                loop.run_until_complete(connection_Zach.cleanup())
        elif mouse == "Machi":
            if Connected:
                print("disconnecting...\n")
                loop.run_until_complete(connection_Machi.cleanup())
        elif mouse == "Loaner":
            if Connected:
                print("disconnecting...\n")
                loop.run_until_complete(connection_Loaner.cleanup())

