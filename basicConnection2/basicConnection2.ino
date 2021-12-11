#include <ArduinoBLE.h>

BLEService mazeService("0fe79935-cd39-480a-8a44-06b70f36f248");

// create switch characteristic and allow remote device to read and write
BLEUnsignedIntCharacteristic flagCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24a", BLERead | BLEWrite | BLENotify);
BLEStringCharacteristic directionsCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24c", BLERead | BLEWrite,100);
String directions="LBLLLBSBLLBSLL";

void bluetooth_init();

void setup() 
{
  Serial.begin(115200);
  bluetooth_init();
  update_directions(directions);
 }

void loop() 
{
  
  //polls and handles any events on the queue
  BLE.poll();
  
  //add delay to prevent continuous polling
  delay(50);
}

void blePeripheralConnectHandler(BLEDevice central) 
{
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) 
{
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void mazeCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) 
{
  directions = directionsCharacteristic.value();  
  flagCharacteristic.setValue(2);
  Serial.print("directionsCharacteristicWritten: ");
  Serial.print(directions);
}

void update_directions(String directions)
{
  flagCharacteristic.setValue(1);
  directionsCharacteristic.writeValue(directions);
}

void bluetooth_init()
{
 if (!BLE.begin()) 
 {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Set the connection interval to be as fast as possible (about 40 Hz)
  BLE.setConnectionInterval(0x0006, 0x0006);

  BLE.setLocalName("Ed's Mouse :)");
  BLE.setAdvertisedService(mazeService);
  mazeService.addCharacteristic(flagCharacteristic);
  mazeService.addCharacteristic(directionsCharacteristic);
  BLE.addService(mazeService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  directionsCharacteristic.setEventHandler(BLEWritten, mazeCharacteristicWritten);
  flagCharacteristic.setValue(-1);
  String directions="LBLLLBSBLLBSLL";
  directionsCharacteristic.writeValue(directions);
  BLE.advertise();
  Serial.println("Waiting for connection");
}
