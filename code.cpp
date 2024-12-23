#include <OneWire.h>
#include <DallasTemperature.h>
#include "Wire.h"  // Allows communication over i2c devices
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD address to 0x3F for a 16 chars and 2 line display

RF24 radio(7, 8);  // CE, CSN Arduino pins

#define ONE_WIRE_BUS 2 // Data wire is conected to the Arduino digital pin 2 for the heater temperature sensor

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices

DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor

// Variables for the heater temperature sensor
float heaterTemp = 0;
float pumpTemp = 0;

// Variables for the pressure sensor
const int pressureInput = A0; // Select the analog input pin for the pressure transducer
const int pressureZero = 97.000; // Analog reading of pressure transducer at 0psi
const int pressureMax = 921.600; //Analog reading of pressure transducer at 100psi
const int pressuretransducermaxPSI = 100.000; // Psi value of transducer being used

float pressureValue = 0; // Variable to store the value coming from the pressure transducer
float pressureValueBar = 0; // Variable to store the converted pressure in bar

bool pressureHigh = LOW; // Variable to store excess pressure in the pump output
bool pressureLow = LOW; // Variable to store insuficient pressure in the pump output

// Variables for the thermistor
int thermistorPin = A1; // Pin where the voltage divider connects
int sensorValue = 0; // Variable to store the sensor value
float voltage = 0.0; // Variable to store the calculated voltage
float resistance = 0.0; // Variable to store the thermistor resistance
float temperatureC = 0.0; // Variable to store the temperature in Celsius

const float R25 = 50000.0; // Resistance of the thermistor at 25°C (50kΩ)
const float B = 3950.0; // B-constant (3950K)
const float R_fixed = 50000.0; // The fixed resistor value (50kΩ)

// Variables for the flow meter
int flowPin = 3; //This is the input pin on the Arduino
double flowRate; //This is the value we intend to calculate.
int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.
unsigned long startTime = 0;
int lastState = LOW; // Variable to hold the previous state of the pin
bool flowRateHigh = LOW; // High flow rate variable
bool flowRateLow = LOW; // Low flow rate variable

// Variables for the relays
int sewerRelay = 4; // Relay 1 pin
int pumpRelay = 5; // Relay 2 pin
bool pumpStatus = LOW; // Working status of the water pump
bool sewerStatus = LOW; // Working status of the sewer valve

// Variables for the radio
const byte address[6] = "00001"; //Set the radio address
int radioTest = 100; //Variable with arbitrary value to test radio
int receivedValue = -128; //For incoming radio message
int waterLevel = -128; // Variable for the level of water in the expansion tank
bool waterLevelLow = LOW; // Low water level variable
bool waterLevelHigh = LOW; // High water level variable
bool waterLevelInapplicable = LOW; // Out of applicable range water level variable

// Trigger constants
const int distFromSensorToBottom = 60; // Distance from the sensor to the bottom of the expansion tank
const int PumpOnTriggTemp = 45; // Temperature to activate the pump
const int pumpOffTriggTemp = 35; // Temperature to deactivate the pump
const int sewerOnTriggTemp = 80; // Temperature to activate the sewer valve
const int sewerOffTriggTemp = 70; // Temperature to deactivate the sewer valve
const int HighTriggPressure = 2; // High pressure trigger value
const int LowPressureTriggPumpOn = 1.5; // Low pressure value trigger when the pump is active
const int LowPressureTriggPumpOff = 1; // Low pressure value trigger when the pump is not active
const int HighTriggFlowRate = 5; // High flow rate trigger value
const int LowTriggFlowRate = 2; // High flow rate trigger value

// Warning variables
String warnings[8]; // An array to store multiple strings
int warningPosition = 0;
int loopCount = 0;


void setup(){
  // Start serial communication for debugging purposes
  Serial.begin(9600);
  // Start up the library
  sensors.begin();
  //Sets the pin as an input
  pinMode(flowPin, INPUT);

  //Sets the pins as outputs
  pinMode(sewerRelay, OUTPUT);
  pinMode(pumpRelay, OUTPUT);

  radio.begin();                      //start radio
  radio.openReadingPipe(0, address);  //open defined address
  radio.setPALevel(RF24_PA_MIN);      //set minimum radio power level
  radio.setDataRate(RF24_250KBPS);    //set data transfer rate
  radio.setChannel(76);               //Channel 0 corresponds to 2.400 GHz. Channel 76 corresponds to 2.476 GHz. Channel 125 corresponds to 2.525 GHz.
  radio.startListening();             //active radio listening

  lcd.init();
  lcd.clear();
  lcd.backlight();  // Make sure backlight is on

  startupSequence();
}

void loop(){
  //Get sensors data
  readHeaterTemp();
  readPressure();
  readPumpTemp();
  readFlow();
  radioListen(); //Get data from second Arduino. This case, get water level in expansion tank
  readPumpRelay();
  readSewerRelay();

  Serial.println();

  processSensorData(); // Control function to activate or deactivate modules
  sensorsLcdPrint(); // Prints the sensors values to the LCD screen
  warningsLcdPrint(); // Prints warnings to the LCD screen

  delay(2000);
}

void readHeaterTemp(){
  /*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
  Based on the Dallas Temperature Library example
  *********/

  //https://randomnerdtutorials.com/guide-for-ds18b20-temperature-sensor-with-arduino/

  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  sensors.requestTemperatures();

  heaterTemp = sensors.getTempCByIndex(0);

  Serial.print("Heater Celsius temperature: ");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.println(heaterTemp, 2);
  //Serial.print(" - Fahrenheit temperature: ");
  //Serial.println(sensors.getTempFByIndex(0));
}

void readPressure(){
  /*
  Sensor de pressão

  Saída: 0.5V ~ 4.5V tensão linear saída. 0 psi = 0,5 V, 50 psi = 2,5 V, 100 psi = 4,5 V.

  This example demonstrates how to take a standard 3-wire pressure transducer
  and read the analog signal, then convert the signal to a readable output and
  display it onto an LCD screen.

  https://www.youtube.com/watch?v=UrqPxwsPWGk

  Contact Tyler at tylerovens@me.com if you have any questions
 */

  pressureValue = analogRead(pressureInput);                                                                   //reads value from input pin and assigns to variable
  pressureValue = ((pressureValue - pressureZero) * pressuretransducermaxPSI) / (pressureMax - pressureZero);  //conversion equation to convert analog reading to psi

  pressureValueBar = pressureValue * 0.068;

  Serial.print("Pump output pressure in bar: ");  //prints label to serial
  Serial.println(pressureValueBar, 3);            //prints value from previous line to serial
  /*lcd.setCursor(0,0); //sets cursor to column 0, row 0
  lcd.print("Pressure:"); //prints label
  lcd.print(pressureValue, 1); //prints pressure value to lcd screen, 1 digit on float
  lcd.print("psi"); //prints label after value
  lcd.print("   "); //to clear the display after large values or negatives
  */
}

void readPumpTemp(){
  /* Thermistor YF-B7

  TEMPERATURA VS TABELA DE RESISTÊNCIA
  Resistência 50k Ohms em 25deg. C
  B Valor 3950K a 25/50 graus. C
  Temperatura. Escala:-40-+ 150 ℃

  Voltage divider circuit.
  https://www.circuitbasics.com/wp-content/uploads/2015/12/Arduino-Thermistor-Basic-Set-Up.png
  */

  sensorValue = analogRead(thermistorPin);  // Read the voltage value
  voltage = sensorValue * (5.0 / 1023.0);   // Convert to actual voltage (0-5V)

  // Calculate the thermistor resistance
  resistance = R_fixed * (5.0 / voltage - 1);

  // Calculate the temperature in Kelvin using the B-parameter equation
  float temperatureK = 1.0 / (1.0 / 298.15 + (log(resistance / R25)) / B);  // In Kelvin

  // Convert temperature from Kelvin to Celsius
  temperatureC = temperatureK - 273.15;

  pumpTemp = temperatureC;

  // Output the results to the Serial Monitor
  Serial.print("Pump Celsius Temperature: ");
  Serial.println(pumpTemp, 1);  // Print temperature in Celsius
}

void readFlow(){
  /* Water flow meter YF-B7

  https://bc-robotics.com/tutorials/using-a-flow-sensor-with-arduino/

  YF-B7 data Specs:
  (1) caudal: 1 ~ 25 L/min
  (2) pulso do fluxo: F = 11 * Q ± 5% , Q = L/min
  (3) Tensão De Trabalho: DC3V ~ 24V
  (4) trabalho atual: 15mA
  (5) Sofrendo a pressão: 1.5Mpa máximo
  (6) temperatura de trabalho: -40 °C ~ + 80 °C
  (7)Min Isolamento Resistência: 100M Ohm
  (8) ciclo do dever do pulso da saída: 50% +/-- 10%
  (9) saída máxima Tensão do pulso: >DC4.7V (tensão de entrada DC5V)
  (10) Tensão mínima do pulso da saída:<DC0.5V (tensão entrada DC5V)
  (11) Sensor: sensor do efeito do salão, saída digital. e sensor da temperatura
  (12) Conexão do fio:
  Terminal 1 (vermelho) : Vdd "+"
  Terminal 2 (Preto) GND " - "
  Terminal 3 (amarelo) Signal: saída
  Sensor do Temp do Terminal 4
  Temp do terminal 5. Sensor
  (13) Conector: Habitação XH2.5-3P

  If 𝑄 = 1 L/min Q=1L/min, then 𝐹 = 11 ± 5 % = 10.45 Hz to 11.55 Hz F=11±5%=10.45 Hzto11.55 Hz.
  */

  startTime = millis();  // Record the current time
  count = 0;             // Reset the counter so we start counting from 0 again

  while (millis() - startTime < 1000) {
    // For each iteration, check if the pin is HIGH
    for (int i = 0; i < 1; i++) {               //Rising edge counting on the pin limited to one time
      int currentState = digitalRead(flowPin);  // Read the current state of the pin

      if (lastState == LOW && currentState == HIGH) { // Detect rising edge: pin goes from LOW to HIGH
        count++;  // Increment the count on a rising edge
      }
      lastState = currentState;  // Update lastState for the next iteration
    }
  }
  //Start the math
  flowRate = (count * 1.515);  //Take counted pulses in the last second and multiply by 1.515mL
  flowRate = flowRate * 60;    //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate / 1000;  //Convert mL to Liters, giving you Liters / Minute

  Serial.print("Pump output flow rate in L/min: ");
  Serial.println(flowRate);  //Print the variable flowRate to Serial
}

void radioListen(){
  while (millis() - startTime < 6000) {
    if (radio.available()) {
      radio.read(&receivedValue, sizeof(receivedValue));  // Read the message
      waterLevel = receivedValue;                         // The received data is the tank water level
    }
    break;  // Exit the loop once message is received
  }
  Serial.print("Water level in expansion tank: ");
  Serial.println(waterLevel);
}

void readPumpRelay(){
  pumpStatus = digitalRead(pumpRelay);
  Serial.print("Pump status: ");
  Serial.println(pumpStatus);
}

void readSewerRelay(){
  sewerStatus = digitalRead(sewerRelay);
  Serial.print("Sewer status: ");
  Serial.println(sewerStatus);
}

void startupSequence(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUECIMENTO CENTRAL");
  delay(2000);
  lcd.clear();
  lcd.print("TESTE SENSORES:");

  Serial.println("SENSORS TEST:");

  //test heater temperature sensor range
  readHeaterTemp();
  if (heaterTemp > 0 && heaterTemp < 110) {  //test between 0 and 110 ºC
    lcd.setCursor(0, 2);                     //Set cursor to lcd line 2
    lcd.print("T1:OK");
    Serial.println("T1:OK");
  } else {
    lcd.setCursor(0, 2);
    lcd.print("T1:NOK");
    Serial.println("T1:NOK");
  }
  delay(1500);  //display the message for 1 second
  lcd.setCursor(0, 2);
  lcd.print("                    ");  //clear specific lcd line with blanks

  //test pressure sensor range
  readPressure();
  if (pressureValueBar > 0 && heaterTemp < 4) {  //test between 0 and 6 bar
    lcd.setCursor(0, 2);
    lcd.print("P1:OK");
    Serial.println("P1:OK");
  } else {
    lcd.setCursor(0, 2);
    lcd.print("P1:NOK");
    Serial.println("P1:NOK");
  }
  delay(1500);
  lcd.setCursor(0, 2);
  lcd.print("                    ");

  //test pump thermistor temperature range
  readPumpTemp();
  if (pumpTemp > 0 && pumpTemp < 110) {  //test between 0 and 110 ºC
    lcd.setCursor(0, 2);
    lcd.print("T2:OK");
    Serial.println("T2:OK");
  } else {
    lcd.setCursor(0, 2);
    lcd.print("T2:NOK");
    Serial.println("T2:NOK");
  }
  delay(1500);
  lcd.setCursor(0, 2);
  lcd.print("                    ");  //clear specific lcd line with blanks

  //test water flow meter range
  readFlow();
  if (flowRate > 0 && flowRate < 20) {  //test between 0 and 20 L/min
    lcd.setCursor(0, 2);
    lcd.print("Fluxo:OK");
    Serial.println("Flow:OK");
  } else {
    lcd.setCursor(0, 2);
    lcd.print("Fluxo:NOK");
    Serial.println("Flow:NOK");
  }
  delay(1500);
  lcd.setCursor(0, 2);
  lcd.print("                    ");  //clear specific lcd line with blanks

  //test radio receive
  startTime = millis();  // Record the current time
  receivedValue = -128;
  radioTest = 100;  //Arbitrary value

  while (millis() - startTime < 6000) {
    if (radio.available()) {
      radio.read(&receivedValue, sizeof(receivedValue));  // Read the message
      if (receivedValue == -128) {                        //In the case the data received is the same that the receivedValue had before, it will increment 1 on the radioTest variable. Serves to exclude a particular scenario.
        radioTest++;                                      //Add 1
      }
      break;  // Exit the loop once message is received
    }
  }
  if (receivedValue == -128 && radioTest == 100) {  // Check if the receivedValue and radioTest did not change
    lcd.setCursor(0, 2);
    lcd.print("Radio:NOK");
    Serial.println("Radio:NOK");
  } else {
    lcd.setCursor(0, 2);
    lcd.print("Radio:OK");
    Serial.println("Radio:OK");
  }
  delay(1500);
  lcd.setCursor(0, 2);
  lcd.print("                    ");  //clear specific lcd line with blanks

  //test ultrasonic sensor
  startTime = millis();  // Record the current time
  receivedValue = -128;

  while (millis() - startTime < 6000) {
    if (radio.available()) {
      radio.read(&receivedValue, sizeof(receivedValue));  // Read the value
      waterLevel = receivedValue;                         // The received data is the tank water level
    }
    break;  // Exit the loop once message is received
  }
  if (waterLevel >= 0 && waterLevel < distFromSensorToBottom) {  // The distance between the sensor and the bottom of the tank is 60cm.
    lcd.setCursor(0, 2);
    lcd.print("Nivel:OK");
    Serial.print("Water level in expansion tank: ");
    Serial.println(waterLevel);
    Serial.println("Level:OK");

  } else {
    lcd.setCursor(0, 2);
    lcd.print("Nivel:NOK");
    Serial.print("Water level in expansion tank: ");
    Serial.println(waterLevel);
    Serial.println("Level:NOK");
  }
  delay(1500);
  lcd.setCursor(0, 2);
  lcd.print("                    ");  //clear specific lcd line with blanks

  //Display water pump status
  readPumpRelay();
  lcd.setCursor(0, 2);
  lcd.print("BOMBA:");
  lcd.setCursor(7, 2);

  if (pumpStatus == LOW) {
    lcd.print("OFF");
  } else {
    lcd.print("ON");
  }
  delay(1500);
  lcd.setCursor(0, 2);
  lcd.print("                    ");  //clear specific lcd line with blanks

  //Display sewer valve status
  readSewerRelay();
  lcd.setCursor(0, 2);
  lcd.print("ESGOTO:");
  lcd.setCursor(8, 2);

  if (sewerStatus == LOW) {
    lcd.print("OFF");
  } else {
    lcd.print("ON");
  }
  delay(1500);
  lcd.setCursor(0, 2);
  lcd.print("                    ");  //clear specific lcd line with blanks


  Serial.println("...END TEST...");
  Serial.println();
}

void processSensorData(){  // This function reads sensor values and controls the status of components
  // Check the heater temperature and turn the water pump ON or OFF
  if (pumpStatus == LOW && heaterTemp > PumpOnTriggTemp) { // Check if the temperature in the heater is higher than the set value
    digitalWrite(pumpRelay, HIGH); // Turn on the water pump if condition is met
    Serial.print("Pump turned ON from heater temperature sensor");
    Serial.println();
  }
  if (pumpStatus == HIGH && heaterTemp < pumpOffTriggTemp && pumpTemp < pumpOffTriggTemp) { // Check if the pump is ON and the temperature is below the set value
    digitalWrite(pumpRelay, LOW); // Turn OFF pump if condition is met
    Serial.print("Pump turned OFF");
    Serial.println();
  }

  // Backup temperature check. Check the pump temperature and turn the water pump ON only
  if (pumpStatus == LOW && pumpTemp > PumpOnTriggTemp) { // Check if the temperature in the pump is higher than the set value
    digitalWrite(pumpRelay, HIGH); // Turn on the water pump if condition is met
    Serial.print("Pump turned ON from backup pump temperature sensor");
    Serial.println();
  }

  // Check the heater temperature and turn the sewer valve ON or OFF
  if (sewerStatus == LOW && heaterTemp > sewerOnTriggTemp) { // Check if the temperature in the heater is higher than the set value
    digitalWrite(sewerRelay, HIGH); // Open sewer valve if condition is met
    Serial.print("Sewer valve OPEN from heater temperature sensor");
    Serial.println();
  }
  if (sewerStatus == HIGH && heaterTemp < sewerOffTriggTemp && pumpTemp < sewerOffTriggTemp) { // Check if the sewer valve is ON and the temperature is below the set value
    digitalWrite(sewerRelay, LOW); // Turn OFF sewer valve if condition is met
    Serial.print("Sewer valve CLOSED");
    Serial.println();
  }

  // Backup temperature check. Check the pump temperature and turn the sewer valve ON only
  if (sewerStatus == LOW && pumpTemp > (sewerOnTriggTemp - 20)) {  // Check if the temperature in the pump is higher than the set value. Offset of 20ºC for this temperature sensor
    digitalWrite(sewerRelay, HIGH); // Turn on the water pump if condition is met
    Serial.print("Sewer valve OPEN from backup pump temperature sensor");
    Serial.println();
  }

  // Check the pump output pressure
  if (pressureHigh == LOW && pressureValueBar > HighTriggPressure) { // Check if the pressure value is above the set value
    pressureHigh = HIGH; // Set the excess pressure variable HIGH
    Serial.print("High pressure");
    Serial.println();
  }
  if (pressureLow == LOW && pressureValueBar < LowPressureTriggPumpOn && pumpStatus == HIGH) { // Check if the pressure value is below the set value when the pump is running
    pressureLow = HIGH; // Set the insuficient pressure variable HIGH
    Serial.print("Low pressure with pump ON");
    Serial.println();
  }
  if (pressureLow == LOW && pressureValueBar < LowPressureTriggPumpOff && pumpStatus == LOW) { // Check if the pressure value is below the set value when the pump is off
    pressureLow = HIGH; // Set the insuficient pressure variable HIGH
    Serial.print("Low pressure with pump OFF");
    Serial.println();
  }

  if (pressureHigh == HIGH && pressureValueBar <= HighTriggPressure) { // Check if the pressure value is below the set value
    pressureHigh = LOW; // Set the excess pressure variable LOW
    Serial.print("Pressure OK. Previously high");
    Serial.println();
  }
  if (pressureLow == HIGH && pressureValueBar >= LowPressureTriggPumpOn && pumpStatus == HIGH) { // Check if the pressure value is above the set value when the pump is running
    pressureLow = LOW; // Set the insuficient pressure variable LOW
    Serial.print("Pressure OK. Previously low with the pump on");
    Serial.println();
  }
  if (pressureLow == HIGH && pressureValueBar >= LowPressureTriggPumpOff && pumpStatus == LOW) { // Check if the pressure value is above the set value when the pump is off
    pressureLow = LOW; // Set the insuficient pressure variable LOW
    Serial.print("Pressure OK. Previously low with the pump off");
    Serial.println();
  }

  // Check the water flow at the pump output
  if (flowRateHigh == LOW && pumpStatus == HIGH && flowRate > HighTriggFlowRate) { // Check if the flow rate is above the set value
    flowRateHigh = HIGH; // Set the excess flow rate variable HIGH
    Serial.print("High flow rate");
    Serial.println();
  }
  if (flowRateLow == LOW && pumpStatus == HIGH && flowRate < LowTriggFlowRate) { // Check if the flow rate is below the set value
    flowRateLow = HIGH; // Set the insuficient flow rate variable HIGH
    Serial.print("Low flow rate");
    Serial.println();
  }

  if (flowRateHigh == HIGH && pumpStatus == HIGH && flowRate <= HighTriggFlowRate) { // Check if the flow rate is below the set value
    flowRateHigh = LOW; // Set the excess flow rate variable LOW
    Serial.print("Flow rate OK. Previously high");
    Serial.println();
  }
  if (flowRateLow == HIGH && pumpStatus == HIGH && flowRate >= LowTriggFlowRate) { // Check if the flow rate is above the set value
    flowRateLow = LOW; // Set the insuficient flow rate variable LOW
    Serial.print("Low flow rate. Previously low");
    Serial.println();
  }

  // Check water level
  if (waterLevelLow == LOW && waterLevel >= 0 && waterLevel < 10) { // Check if the water level is insuficient in the tank
    waterLevelLow = HIGH; // Set the low water level variable HIGH
    Serial.print("Low water level in the expansion tank");
    Serial.println();
  }
  if (waterLevelHigh == LOW && waterLevel > 30 && waterLevel <= 60) { // Check if the water level is excessive in the tank
    waterLevelHigh = HIGH; // Set the excessive water level variable HIGH
    Serial.print("High water level in the expansion tank");
    Serial.println();
  }
  if (waterLevelInapplicable == LOW && waterLevel < 0 || waterLevel > 60) { // Check if the water level is out of applicable range
    waterLevelInapplicable = HIGH; // Set the out of range variable HIGH
    Serial.print("Water level in the expansion tank out of range");
    Serial.println();
  }

  if (waterLevelLow == HIGH || waterLevelHigh == HIGH || waterLevelInapplicable == HIGH && waterLevel >= 10 && waterLevel <= 30) { // Check if the water level is in the correct range
    waterLevelLow = LOW; // Set low water level to LOW
    waterLevelHigh = LOW; // Set high water level to LOW
    waterLevelInapplicable = LOW; // Set out of range variable to LOW
    Serial.print("Water level in the expansion tank OK");
    Serial.println();
  }
  Serial.println();
}

void sensorsLcdPrint(){
  // Write values on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T1:");  // Heater water temperature
  lcd.setCursor(3, 0);
  lcd.print(heaterTemp);

  lcd.setCursor(9, 0);
  lcd.print("T2:");  // Pump water temperature
  lcd.setCursor(12, 0);
  lcd.print(pumpTemp);

  lcd.setCursor(0, 1);
  lcd.print("bar:");  // Heater pressure
  lcd.setCursor(4, 1);
  lcd.print(pressureValueBar);

  lcd.setCursor(9, 1);
  lcd.print("L/m:");  // Flow rate at pump output
  lcd.setCursor(13, 1);
  lcd.print(flowRate);

  lcd.setCursor(0, 2);
  lcd.print("N:");  // Water level at the tank
  lcd.setCursor(2, 2);
  lcd.print(waterLevel);
  lcd.setCursor(4, 2);
  lcd.print("cm");

  lcd.setCursor(9, 2);
  lcd.print("B:");  // Pump status
  lcd.setCursor(11, 2);
  if (pumpStatus == LOW) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }

  lcd.setCursor(15, 2);
  lcd.print("E:");  // Water level at the tank
  lcd.setCursor(17, 2);
  if (pumpStatus == LOW) {
    lcd.print("ON");
  } else {
    lcd.print("OFF");
  }
}

void warningsLcdPrint(){
  // Check the sewer status
  if(sewerStatus == HIGH){
    warnings[0] = "ESGOTO ABERTO";
  }
  else{
    warnings[0] = "";
  }

  // Check the pressure status
  if(pressureHigh == HIGH){
    warnings[1] = "PRESSAO ALTA";
  }
  else{
    warnings[1] = "";
  }

  if(pressureLow == HIGH){
    warnings[2] = "PRESSAO BAIXA";
  }
  else{
    warnings[2] = "";
  }

  // Check the flow rate
  if(flowRateHigh == HIGH){
    warnings[3] = "FLUXO AGUA ALTO";
  }
  else{
    warnings[3] = "";
  }

  if(flowRateLow == HIGH){
    warnings[4] = "FLUXO AGUA BAIXO";
  }
  else{
    warnings[4] = "";
  }

  // Check the water level
    if(waterLevelLow == HIGH){
    warnings[5] = "NIVEL AGUA BAIXO";
  }
  else{
    warnings[5] = "";
  }

  if(waterLevelHigh == HIGH){
    warnings[6] = "NIVEL AGUA ALTO";
  }
  else{
    warnings[6] = "";
  }

  if(waterLevelInapplicable == HIGH){
    warnings[7] = "NIVEL AGUA. ERRO";
  }
  else{
    warnings[7] = "";
  }

  //LCD message print loop
  for(warningPosition; warningPosition <= 7; warningPosition++){ // For loop cycling through the set amount of array positions
    if(warnings[warningPosition] != ""){ // Check if the array is not empty on the set position
      lcd.setCursor(0, 3);
      lcd.print("                    ");  // Clear specific lcd line with blanks
      lcd.print(warnings[warningPosition]); // Print the set position of the array
      break;
    }
    if(warningPosition == 7){ // Reset warningPosition to 0 if it reaches 7
      warningPosition = 0;
    }
    if(loopCount == 7){ // Reset loopCount to 0 and exit the loop after going through the full array
      loopCount = 0; // Resets to zero and
      break; // Breaks the loop
    }
    loopCount++;
  }

  if(warningPosition == 7){ // If we're at the last position, reset to 0, otherwise increment by 1
    warningPosition = 0; // It resets to zero
  }
  else{ 
    warningPosition++; // Increment warningPosition if it's not the last position
  }
}
