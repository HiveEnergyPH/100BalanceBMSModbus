#include <Arduino.h>
#include <HardwareSerial.h>

// Define the pins for the RS485 module
#define DE_RE_PIN 2 // Control pin for DE/RE

// Create a HardwareSerial object
HardwareSerial rs485Serial(1); // Use Serial1 (TX: GPIO 17, RX: GPIO 16 by default)

// Global variables for Modbus communication
const uint8_t SLAVE_ADDRESS_SEND = 0x81; // Slave address for sending commands
const uint8_t SLAVE_ADDRESS_READ = 0x51; // Slave address for reading responses
const uint8_t READ_COMMAND = 0x03; // Read Holding Registers command

// Array to store cell voltages
float cellVoltages[16]; // Array to hold voltages for 16 cells


uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos]; // XOR byte into least sig. byte of crc
    for (uint8_t i = 8; i != 0; i--) { // Loop over each bit
      if ((crc & 0x0001) != 0) { // If the LSB is set
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else { // Else LSB is not set
        crc >>= 1; // Just shift right
      }
    }
  }
  return crc;
}

void sendCommand(uint8_t *command, size_t length) {
  // Set the DE/RE pin to HIGH to enable transmitting mode
  digitalWrite(DE_RE_PIN, HIGH);
  
  // Send the command
  for (size_t i = 0; i < length; i++) {
    rs485Serial.write(command[i]);
  }
  
  // Wait for the command to be sent
  delay(10);
  
  // Set the DE/RE pin to LOW to enable receiving mode
  digitalWrite(DE_RE_PIN, LOW);
}

float readRegister(uint8_t slaveAddress, uint8_t command, uint16_t registerAddress, uint16_t numberOfRegisters) {
  // Prepare the request frame
  uint8_t requestFrame[8];
  requestFrame[0] = slaveAddress;
  requestFrame[1] = command;
  requestFrame[2] = (registerAddress >> 8) & 0xFF; // MSB of register address
  requestFrame[3] = registerAddress & 0xFF;        // LSB of register address
  requestFrame[4] = (numberOfRegisters >> 8) & 0xFF; // MSB of number of registers
  requestFrame[5] = numberOfRegisters & 0xFF;        // LSB of number of registers
  
  // Calculate CRC
  uint16_t crc = calculateCRC(requestFrame, 6);
  requestFrame[6] = crc & 0xFF;        // CRC-L
  requestFrame[7] = (crc >> 8) & 0xFF; // CRC-H
  
  // Send the command
  sendCommand(requestFrame, sizeof(requestFrame));
  
  // Wait for a response
  delay(1000); // Adjust delay as needed
  
  // Read the response from the sensor
  if (rs485Serial.available()) {
    uint8_t response[256]; // Buffer for response
    size_t index = 0;
    
    // Read the response until no more data is available
    while (rs485Serial.available() && index < sizeof(response)) {
      response[index++] = rs485Serial.read();
    }
    
    // Process the response
    if (index > 0 && response[0] == SLAVE_ADDRESS_READ) { // Check for the new slave address
      uint16_t value = (response[3] << 8) | response[4]; // Combine MSB and LSB
      return static_cast<float>(value); // Return the value as a float
    } else {
      Serial.println("Invalid response or address mismatch.");
      return -1.0; // Return an error value
    }
  } else {
    Serial.println("No response received.");
    return -1.0; // Return an error value
  }
}

void readSingleCellVoltages() 
{
  // Read single cell voltages from address 0x00 to 0x0F
  for (uint8_t i = 0; i < 16; i++) 
  {
    uint16_t registerAddress = 0x00 + i; // Register address for each cell voltage
    cellVoltages[i] = readRegister(SLAVE_ADDRESS_SEND, READ_COMMAND, registerAddress, 1); // Read each cell voltage
  }
  
  // Print the cell voltages
  Serial.println("Cell Voltages:");
  for (uint8_t i = 0; i < 16; i++) 
  {
    Serial.print("Cell ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(cellVoltages[i]);
    Serial.println(" V");
  }
}



void setup() {
  // Start the serial communication with the computer
  Serial.begin(9600);
  
  // Start the RS485 serial communication
  rs485Serial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for Serial1
  
  // Set the DE/RE pin as output
  pinMode(DE_RE_PIN, OUTPUT);
  
  // Set the DE/RE pin to LOW to enable receiving mode
  digitalWrite(DE_RE_PIN, LOW);
  
  Serial.println("RS485 Sensor Communication Example");
}

void loop() {
  // Example of reading the SOC from register address 0x3A
  float soc = readRegister(SLAVE_ADDRESS_SEND, READ_COMMAND, 0x3A, 1); // Slave address, command, register address, number of registers
  Serial.print("State of Charge (SOC): ");
  Serial.print(soc);
  Serial.println("%");

  readSingleCellVoltages();

  // Wait before sending the next command
  delay(5000); // Adjust delay as needed
}