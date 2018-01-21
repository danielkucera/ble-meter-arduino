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


void setup() {

	configure_ram_retention();

	blePeripheral.setLocalName("GasMeter");

	// Set the GPIOTE PORT event as interrupt source, and enable interrupts for GPIOTE
	//NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
	//NVIC_EnableIRQ(GPIOTE_IRQn);

}

void loop() {
  i++;
	char data[30];
	unsigned char len = sprintf(data, "{ data: %ld }", i);
	unsigned char *udata = (unsigned char*)(&data);
	//const unsigned char sudata[len];
	// = &udata;

	blePeripheral.end();
  blePeripheral.setManufacturerData(udata, len);
  blePeripheral.begin();

  delay(1000);
	__WFE();
	__SEV();
	__WFE();

}
