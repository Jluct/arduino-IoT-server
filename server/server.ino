/*
  Web Server

  created 18 Dec 2009
  by David A. Mellis
  modified 9 Apr 2012
  by Tom Igoe
  modified 02 Sept 2015
  by Arturo Guadalupi

  modified 30 Aug 2017
  by Listopadov Sergei

*/

#include <SPI.h>
#include <Ethernet.h>
#include <OneWire.h>

/////SENSOR FUNCTIONS///////

//int getTemperature(int pin) {
//  return 27;
//  OneWire  ds(pin);
//
//  byte i;
//  byte present = 0;
//  byte type_s;
//  byte data[12];
//  byte addr[8];
//  float celsius;
//
//  if ( !ds.search(addr)) {
//    ds.reset_search();
//    delay(250);
//    return;
//  }
//
//  if (OneWire::crc8(addr, 7) != addr[7]) {
//    Serial.println("CRC is not valid!");
//    return;
//  }
//  //  Serial.println();
//
//  // the first ROM byte indicates which chip
//  switch (addr[0]) {
//    case 0x10:
//      //      Serial.println("  Chip = DS18S20");  // or old DS1820
//      type_s = 1;
//      break;
//    case 0x28:
//      //      Serial.println("  Chip = DS18B20");
//      type_s = 0;
//      break;
//    case 0x22:
//      //      Serial.println("  Chip = DS1822");
//      type_s = 0;
//      break;
//    default:
//      //      Serial.println("Device is not a DS18x20 family device.");
//      return;
//  }
//
//  ds.reset();
//  ds.select(addr);
//  ds.write(0x44);        // start conversion, with parasite power on at the end
//
//  delay(1000);     // maybe 750ms is enough, maybe not
//  // we might do a ds.depower() here, but the reset will take care of it.
//
//  present = ds.reset();
//  ds.select(addr);
//  ds.write(0xBE);         // Read Scratchpad
//
//  //  Serial.print("  Data = ");
//  //  Serial.print(present, HEX);
//  //  Serial.print(" ");
//  for ( i = 0; i < 9; i++) {           // we need 9 bytes
//    data[i] = ds.read();
//    //    Serial.print(data[i], HEX);
//    //    Serial.print(" ");
//  }
//  //  Serial.print(" CRC=");
//  //  Serial.print(OneWire::crc8(data, 8), HEX);
//  //  Serial.println();
//
//  // Convert the data to actual temperature
//  // because the result is a 16 bit signed integer, it should
//  // be stored to an "int16_t" type, which is always 16 bits
//  // even when compiled on a 32 bit processor.
//  int16_t raw = (data[1] << 8) | data[0];
//  if (type_s) {
//    raw = raw << 3; // 9 bit resolution default
//    if (data[7] == 0x10) {
//      // "count remain" gives full 12 bit resolution
//      raw = (raw & 0xFFF0) + 12 - data[6];
//    }
//  } else {
//    byte cfg = (data[4] & 0x60);
//    // at lower res, the low bits are undefined, so let's zero them
//    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
//    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
//    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
//    //// default is 12 bit resolution, 750 ms conversion time
//  }
//  celsius = (float)raw / 16.0;
//  //  Serial.print("  Temperature = ");
//  //  Serial.println(celsius);
//  //  Serial.print(" Celsius, ");
//  //  Serial.print(fahrenheit);
//  //  Serial.println(" Fahrenheit");
//  //  Serial.println(celsius);
//  return celsius;
//
//}

int analogSensorToProcent(int pin) {
  pinMode(pin, INPUT);
  //  return analogRead(pin);
  return map(analogRead(pin), 0, 1024, 0, 100);
}

//////***************///////

int sensorsPin[] = {A0, A1, 10};

int (*funcArray[])(int pin) = {&analogSensorToProcent, &analogSensorToProcent, &digitalRead};


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(192, 168, 1, 2);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only

  }

  // Serial.println(generateSensorsToJSON()); //from debug

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}


void loop() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/json");
          client.println("Access-Control-Allow-Origin: *");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          //          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();

          client.print(generateSensorsToJSON());

          //          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

String generateSensorsToJSON() {
  ///////////
  char stringBuffer[6] = "";
  String out = String();
  out.concat("{");
  for (int i = 0; i < (sizeof(sensorsPin) / sizeof(int)); i++) {
    out.concat("\"");
    out.concat(itoa(sensorsPin[i], stringBuffer, 10));
    out.concat("\"");
    out.concat(":");
    out.concat(itoa(((*funcArray[i])(sensorsPin[i])), stringBuffer, 10));
    if ((sizeof(sensorsPin) / sizeof(int)) - 1 != i)
      out.concat(",");
  }

  out.concat("}");
  Serial.println(out);
  return out;
}









