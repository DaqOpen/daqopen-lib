/*
  Program Name: adc_dma_usb
  Description: continous sample 6 channel in differential mode and stream via usb to host pc

  Author: Michael Oberhofer
  Created on: 2017-05-01
  Last Updated: 2023-11-14

  Hardware: Arduino Due (with SAM3X)

  Libraries: None

  License: MIT

  Notes: Correctly set the USB interface branding for detection of host driver

  Connections:
  - USB (native otg port) to host pc (or arduino)

  Pinout:
  - uses SAM3X AD0 to AD7 (Board Pin A7 to A0) and AD10 to AD13 (Board Pin A8-A11)
  - Differential Mode, AD0 Result is AD0-AD1

  Resources:
  - http://forum.arduino.cc/index.php?topic=137635.msg1136315#msg1136315
  - http://www.robgray.com/temp/Due-pinout.pdf

  Version: 0.91
  Github: https://github.com/DaqOpen/daqopen-lib/firmware/due-daq
  
  */

#undef HID_ENABLED

#define buffer_size 6 * 2048
#define start_marker_value 0xFFFF

#define RX_LED 72
#define TX_LED 73

uint8_t protocol_version = 0x01;
String serial_input_buffer;
uint16_t adc_buffers[2][buffer_size];   
uint16_t simulation_buffer[buffer_size];
uint16_t start_marker[1];
const adc_channel_num_t adc_channels[] = {ADC_CHANNEL_0, ADC_CHANNEL_2, ADC_CHANNEL_4, ADC_CHANNEL_6, ADC_CHANNEL_10, ADC_CHANNEL_12};
volatile uint32_t packet_count = 0, last_packet_count = 0;
bool send_data = false;

void ADC_Handler(){
  int interrupt_flags = ADC->ADC_ISR;
  if (interrupt_flags & (1 << 27)){
    ADC->ADC_RNPR = (uint32_t)adc_buffers[packet_count % 2];
    ADC->ADC_RNCR = buffer_size;
    packet_count++;
  } 
}

void configureADC(){
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  NVIC_EnableIRQ(ADC_IRQn);
  adc_disable_all_channel(ADC);
  
  ADC->ADC_MR |= 0x80;  // Free running
  bitSet(ADC->ADC_MR, 10);  // Set ADC Clock to ~48 kHz per Channel
  ADC->ADC_CHER = 0x1455;  // Select Channels CH0, CH2, CH4, CH6, CH10, CH12
  ADC->ADC_COR = 0x14550000;  // Set Channels CH0, CH2, CH4, CH6, CH10, CH12 to differential
  ADC->ADC_CGR = 0x00;  // Set GAIN to 1 (DIFF=1)
  adc_configure_sequence(ADC, adc_channels, 6);
  ADC->ADC_IDR = ~(1 << 27);
  ADC->ADC_IER = 1 << 27;
}

void configureDMA(){
  ADC->ADC_RPR = (uint32_t)adc_buffers[0];  // DMA buffer
  ADC->ADC_RCR = buffer_size;
  ADC->ADC_RNPR = (uint32_t)adc_buffers[1];  // Next DMA buffer
  ADC->ADC_RNCR = buffer_size;
  ADC->ADC_PTCR = 1;
  ADC->ADC_CR = 2;
}

void sendADCData(){
  SerialUSB.write((uint8_t *)start_marker, sizeof(start_marker));
  SerialUSB.write((uint8_t *)&packet_count, sizeof(packet_count));

  #ifdef SIMULATION
  SerialUSB.write((uint8_t *)simulation_buffer, sizeof(simulation_buffer));
  #endif

  #ifndef SIMULATION
  SerialUSB.write((uint8_t *)adc_buffers[(packet_count - 1) % 2], 2 * buffer_size);
  #endif
}

void setup(){
  pinMode(RX_LED, OUTPUT);
  digitalWrite(RX_LED, 1);
  pinMode(TX_LED, OUTPUT);
  digitalWrite(TX_LED, 1);
  
  SerialUSB.begin(0);
  while(!SerialUSB);
  start_marker[0] = start_marker_value;
  
  configureADC();
  configureDMA();

  #ifdef SIMULATION
  // Initialize simulation buffer here
  #endif
}

void loop(){
  if (SerialUSB.available() > 0) {
    digitalWrite(RX_LED, 0);
    // Read the incoming bytes:
    serial_input_buffer = SerialUSB.readStringUntil('\n');
    if (serial_input_buffer == "START") {
      send_data = true;
      packet_count = 0;
    }
    if (serial_input_buffer == "STOP") {
      send_data = false;
      SerialUSB.flush();
    }
    if (serial_input_buffer == "RESET") {
      send_data = false;
      SerialUSB.flush();
      SerialUSB.end();
      rstc_start_software_reset(RSTC);
    }
  }

  while(last_packet_count == packet_count);  // Wait for the next ADC DMA packet

  if (send_data) {
    sendADCData();
    digitalWrite(TX_LED, packet_count % 2);
  }

  last_packet_count = packet_count;
  digitalWrite(RX_LED, 1);
}
