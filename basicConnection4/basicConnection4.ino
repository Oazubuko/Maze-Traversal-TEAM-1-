#include <ArduinoBLE.h>
BLEService mazeService("0fe79935-cd39-480a-8a44-06b70f36f248");

// create switch characteristic and allow remote device to read and write
BLEUnsignedCharCharacteristic flagCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24a", BLERead | BLEWrite | BLENotify);
BLEStringCharacteristic directionsCharacteristic("0fe79935-cd39-480a-8a44-06b70f36f24c", BLERead | BLEWrite,100);
void update_directions(String directions);
String directions="LBLLLBSBLLBSLL";
int milliseconds;

  
void bluetooth_init();

void setup() 
{
  Serial.begin(115200);
  bluetooth_init();
  //update_directions(directions);
  milliseconds = millis();
  flagCharacteristic.setValue(-1);
 }

void loop() 
{
  int millis2;

  millis2 = millis();
  //currently updates once every 1000ms=1s
  //30-60 seconds may be better
  if((millis2 - milliseconds) > 60000)
  {
    update_directions(directions);
    milliseconds = millis2;
  }

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

void flagCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) 
{
  unsigned char flag = flagCharacteristic.value();
  Serial.print("flagCharacteristicWritten ");
  Serial.println(flag);
  if(flag == 3)
  {
      flagCharacteristic.setValue(2);
  }
  else if(flag == 4)
  {
   directions = directionsCharacteristic.value();    
  Serial.print("directionsCharacteristicWritten: ");
  Serial.println(directions);
  }
}

void update_directions(String directions)
{
  directionsCharacteristic.writeValue(directions);
  flagCharacteristic.setValue(1);
}

void bluetooth_init()
{
 if (!BLE.begin()) 
 {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Set the connection interval to be as fast as possible (about 40 Hz)
  BLE.setConnectionInterval(0x0006, 0x0050);

  BLE.setLocalName("Ed's Mouse :)");
  BLE.setAdvertisedService(mazeService);
  mazeService.addCharacteristic(flagCharacteristic);
  mazeService.addCharacteristic(directionsCharacteristic);
  BLE.addService(mazeService);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  flagCharacteristic.setEventHandler(BLEWritten, flagCharacteristicWritten);
  flagCharacteristic.setValue(-1);
  String directions="LBLLLBSBLLBSLL";
  directionsCharacteristic.writeValue(directions);
  BLE.advertise();
  Serial.println("Waiting for connection");
}
