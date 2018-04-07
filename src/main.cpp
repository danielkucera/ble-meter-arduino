/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
//#include "nrf_gpio.h"
//#include "ArduinoLowPower.h"

// for nrf51_dk
#define P0_21 4 // = P0_21
#define P0_22 5 // = P0_22
#define P0_23 6 // = P0_23

// for waveshare_ble400
#define P0_21 21 // = P0_21
#define P0_22 22 // = P0_22
#define P0_23 23 // = P0_23

#define LED_PIN P0_21
#define SENSOR_PIN P0_22


BLEPeripheral blePeripheral = BLEPeripheral();
long i = 0;
char name[10];


void configure_ram_retention(void)
{
    // Configure nRF51 RAM retention parameters. Set for System On 16kB RAM retention
    NRF_POWER->RAMON = POWER_RAMON_ONRAM0_RAM0On   << POWER_RAMON_ONRAM0_Pos
                     | POWER_RAMON_ONRAM1_RAM1On   << POWER_RAMON_ONRAM1_Pos
                     | POWER_RAMON_OFFRAM0_RAM0Off << POWER_RAMON_OFFRAM0_Pos
                     | POWER_RAMON_OFFRAM1_RAM1Off << POWER_RAMON_OFFRAM1_Pos;
		NRF_POWER->RAMONB = POWER_RAMONB_ONRAM2_RAM2Off   << POWER_RAMONB_ONRAM2_Pos
                      | POWER_RAMONB_ONRAM3_RAM3Off   << POWER_RAMONB_ONRAM3_Pos
	                    | POWER_RAMONB_OFFRAM2_RAM2Off  << POWER_RAMONB_OFFRAM2_Pos
	                    | POWER_RAMONB_OFFRAM3_RAM3Off  << POWER_RAMONB_OFFRAM3_Pos;

}

void blink() {
  //state = !state;
}

void deepSleep() {
	//Enter in systemOff mode only when no EasyDMA transfer is active
	//this is achieved by disabling all peripheral that use it
	/*
	NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled;								//disable UART
	NRF_SAADC ->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);	//disable ADC
	NRF_PWM0  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);		//disable all pwm instance
	NRF_PWM1  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
	NRF_PWM2  ->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
	NRF_TWIM1 ->ENABLE = (TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos);	//disable TWI Master
	NRF_TWIS1 ->ENABLE = (TWIS_ENABLE_ENABLE_Disabled << TWIS_ENABLE_ENABLE_Pos);	//disable TWI Slave
*/
  //configure_ram_retention();
	//Enter in System OFF mode

  //enable wakeup by gpio;
  //nrf_gpio_cfg_sense_input(g_APinDescription[SENSOR_PIN].ulPin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW)
  //nrf_gpio_cfg_sense_input();
  attachInterrupt(SENSOR_PIN, blink, RISING);

	sd_power_system_off();

	/*Only for debugging purpose, will not be reached without connected debugger*/
  //  while(1);
}

int32_t temperature_data_get(void)
{
    int32_t temp;
    uint32_t err_code;

    err_code = sd_temp_get(&temp);

    return temp/4;
}

void setup() {

	blePeripheral.setLocalName("GasMeter");

  pinMode(LED_PIN, OUTPUT);

  pinMode(SENSOR_PIN, INPUT_PULLUP);

	// Set the GPIOTE PORT event as interrupt source, and enable interrupts for GPIOTE
	//NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	//NVIC_EnableIRQ(GPIOTE_IRQn);

}

int led_status = 0;

void loop() {
/*
  while (digitalRead(SENSOR_PIN)){
    delay(1);
  }
*/
  i++;
	char data[30];
	unsigned char len = sprintf(data, "{ data: %ld, uptime: %ld }", i, temperature_data_get());
	unsigned char *udata = (unsigned char*)(&data);
	//const unsigned char sudata[len];
	// = &udata;
  led_status = !led_status;

	blePeripheral.end();
  blePeripheral.setManufacturerData(udata, len);
  blePeripheral.begin();

  digitalWrite(LED_PIN, led_status);

  delay(1000);
  //deepSleep();
  //deepSleep();
}
