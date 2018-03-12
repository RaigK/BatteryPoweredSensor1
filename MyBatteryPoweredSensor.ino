/**
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 *
 * This is an example that demonstrates how to report the battery level for a sensor
 * Instructions for measuring battery capacity on A0 are available here:
 * http://www.mysensors.org/build/battery
 *
 */

#define MY_NODE_ID 101 
//#define MY_DEBUG // Enable debug prints to serial monitor
//#define MY_DEBUG_PRINT

#define MyRSSI_Values

// Uncomment the line below, to transmit battery voltage as a normal sensor value
// ID of the sensor child
#define CHILD_ID_RSSI           (1)
//#define CHILD_ID_TX_LEVEL       (2)
//#define CHILD_ID_TX_PERCENT     (3)
//#define CHILD_ID_TX_RSSI        (4)
//#define CHILD_ID_RX_RSSI        (5)
//#define CHILD_ID_TX_SNR         (6)
//#define CHILD_ID_RX_SNR         (7)

//#define BATT_SENSOR    (8)
#define TEMP_SENSOR    (10)

// Enable and select radio type attached
//#define MY_DEBUG_VERBOSE_RFM69
//#define MY_DEBUG_VERBOSE_RFM69_REGISTERS
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_RFM69_NEW_DRIVER   // ATC on RFM69 works only with the new driver (not compatible with old=default driver)
#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW
#define MY_SENSOR_NETWORK
#include <MyConfig.h>

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected ->PD3
#define MAX_ATTACHED_DS18B20 16
#define TEMPERATURE_PRECISION 12 // bit resolution
#define BATT_OUTPUT_PIN A1

#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <MySensors.h>
#include <avr/pgmspace.h>

int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors = 0;
bool metric = true;
DeviceAddress tempDeviceAddress;
int resolution = 12;
int sensorValue = 0;


unsigned long SLEEP_TIME = 10 * 1000;  // sleep time between reads (seconds * 1000 milliseconds)
int oldBatteryPcnt = 0;
int loopcounter = 10;

// Initialize general messages
//MyMessage msgBattVoltage(BATT_SENSOR, V_VAR1);
MyMessage msgTemp(TEMP_SENSOR, V_TEMP);
#ifdef MyRSSI_Values
MyMessage msgRSSI_BAT(CHILD_ID_RSSI, V_VAR1);
MyMessage msgRSSI_TX(CHILD_ID_RSSI, V_VAR2);
MyMessage msgRSSI_RX(CHILD_ID_RSSI, V_VAR3);
MyMessage msgRSSI_TX_LEVEL(CHILD_ID_RSSI, V_VAR4);
MyMessage msgRSSI_TX_PERC(CHILD_ID_RSSI, V_VAR5);
#endif


void before()
{
	// Startup up the OneWire library
	sensors.setResolution(12);
	sensors.begin();
}


void setup()
{
	digitalWrite(BATT_OUTPUT_PIN, 1);
	analogReference(INTERNAL);
	sensors.setWaitForConversion(false);
	// second adc read to get a correct value
	sensorValue = analogRead(BATTERY_SENSE_PIN);

#ifdef MY_DEBUG_PRINT
	Serial.print("ADC=");
	Serial.println(sensorValue);
#endif

}

void presentation()
{

	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("MyBattery Meter", "1.2");
	wait(1000);
	// Fetch the number of attached temperature sensors  
	numSensors = sensors.getDeviceCount();
	int i;
	sensorValue = analogRead(BATTERY_SENSE_PIN);

	// Present all sensors to controller
	for (i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
		present(TEMP_SENSOR + i, S_TEMP, ("DEG C"), true);
		// Search the wire for address
		if (sensors.getAddress(tempDeviceAddress, i))
		{
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: ");
			printAddress(tempDeviceAddress);
			Serial.println();

			Serial.print("Setting resolution to ");
			Serial.println(TEMPERATURE_PRECISION, DEC);

			// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
			sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

			Serial.print("Resolution actually set to: ");
			Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
			Serial.println();
		}
		else {
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");
		}
		wait(1000);
	}
#ifdef MyRSSI_Values
	//	Register all sensors to gw (they will be created as child devices)
	present(CHILD_ID_RSSI, S_CUSTOM, "Batt Volt, TX UPLINK QUALITY RSSI");
#endif
}

void loop()
{
	float temperature;
	//switch the battery divder on this takes about 3µA
	digitalWrite(BATT_OUTPUT_PIN, 1);

	loopcounter++;
#ifdef MY_DEBUG_PRINT
	Serial.print("Loop= ");
	Serial.println(loopcounter);
#endif

	// Fetch temperatures from Dallas sensors							
	sensors.requestTemperatures();

	// query conversion time and sleep until conversion completed
	int16_t conversionTime = 700;//sensors.millisToWaitForConversion(sensors.getResolution());

	// sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
	sleep(conversionTime);

	// Read temperatures and send them to controller 
	for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
		// Fetch and round temperature to one decimal
		temperature = getControllerConfig().isMetric ? sensors.getTempCByIndex(i) : sensors.getTempFByIndex(i);

		if (temperature < (lastTemperature[i] - 0.1) || temperature >(lastTemperature[i] + 0.1)) {
			// Send in the new temperature
			send(msgTemp.setSensor(TEMP_SENSOR + i).set(temperature, 2));
			//loopcounter = 0;
			}
		// Save new temperatures for next compare
		lastTemperature[i] = temperature;

#ifdef MY_DEBUG_PRINT
		Serial.print(i);
		Serial.print(". Temperature=");
		Serial.println(temperature, 3);
#endif
		}
		if (loopcounter >= 1) {
			loopcounter = 0;
			for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {
				send(msgTemp.setSensor(TEMP_SENSOR + i).set(temperature, 2));
			}

		// get the battery Voltage
		sensorValue = analogRead(BATTERY_SENSE_PIN);
		// 1M, 470K divider across battery and using internal ADC ref of 1.1V
		// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
		// ((1.11e6+470e3)/470e3)*1.1 = Vmax = 3.7 Volts
		// 3.7/1023 = Volts per bit = 0,003613281

#ifdef MY_DEBUG_PRINT
		Serial.print("ADC=");
		Serial.println(sensorValue);
#endif

		int batteryPcnt = sensorValue / 10;

		float batteryV = sensorValue * 0.003613281;

#ifdef MY_DEBUG_PRINT
		Serial.print("Battery Voltage: ");
		Serial.print(batteryV);
		Serial.println(" V");

		Serial.print("Battery percent: ");
		Serial.print(batteryPcnt);
		Serial.println(" %");
#endif

		//send(msgBattVoltage.set(batteryV, 2));
		sendBatteryLevel(batteryPcnt);
#ifdef MyRSSI_Values
		// retrieve RSSI / SNR reports from incoming ACK
#ifdef MY_DEBUG_PRINT
		Serial.print("Batt V : ");
		Serial.println(batteryV);
		Serial.print("TX RSSI dBm : ");
		Serial.println(transportGetSignalReport(SR_TX_RSSI));
		Serial.print("RX RSSI dBm : ");
		Serial.println(transportGetSignalReport(SR_RX_RSSI));
		Serial.print("TX Power Level : ");
		Serial.println(transportGetSignalReport(SR_TX_POWER_LEVEL));
		Serial.print("TX Power Percent : ");
		Serial.println(transportGetSignalReport(SR_TX_POWER_PERCENT));
#endif

		send(msgRSSI_BAT.set(batteryV, 2));
		send(msgRSSI_TX.set(transportGetSignalReport(SR_TX_RSSI)));
		send(msgRSSI_RX.set(transportGetSignalReport(SR_RX_RSSI)));
		send(msgRSSI_TX_LEVEL.set(transportGetSignalReport(SR_TX_POWER_LEVEL)));
		send(msgRSSI_TX_PERC.set(transportGetSignalReport(SR_TX_POWER_PERCENT)));

#endif
		}



	
	//switch the battery divder off this saves about 3µA
	digitalWrite(BATT_OUTPUT_PIN, 0);

	sleep(SLEEP_TIME);



}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (deviceAddress[i] < 16) Serial.print("0");
		Serial.print(deviceAddress[i], HEX);
	}
}

