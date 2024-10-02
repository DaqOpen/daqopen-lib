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

#define buffer_size 6 * 2048  // Define the size of the ADC buffers
#define start_marker_value 0xFFFF  // Start marker value to indicate data transmission start

#define RX_LED 72  // Define the pin for RX LED
#define TX_LED 73  // Define the pin for TX LED

uint8_t protocol_version = 0x01;  // Protocol version
String serial_input_buffer;  // Buffer to hold incoming serial data
uint16_t adc_buffers[2][buffer_size];  // Double buffer for ADC data storage
uint16_t simulation_buffer[buffer_size];  // Buffer for simulation data (not implemented yet)
uint16_t start_marker[1];  // Marker indicating the start of ADC data transmission
const adc_channel_num_t adc_channels[] = {ADC_CHANNEL_0, ADC_CHANNEL_2, ADC_CHANNEL_4, ADC_CHANNEL_6, ADC_CHANNEL_10, ADC_CHANNEL_12};  // Array holding the ADC channels
volatile uint32_t packet_count = 0, last_packet_count = 0;  // Counters to track packets of ADC data
bool send_data = false;  // Flag to control whether data should be sent
bool is_differential = false;  // Variable for the ADC mode (Differential/Single-Ended)
bool offset_enabled = false;  // Flag to enable or disable offset mode
uint8_t gain_value = 0x00;  // Default gain (1x)

/**
 * Interrupt handler for the ADC.
 * Called when the ADC finishes a data conversion.
 */
void ADC_Handler(){
  int interrupt_flags = ADC->ADC_ISR;
  // Check if the RX buffer has finished (DMA transfer complete interrupt)
  if (interrupt_flags & (1 << 27)){
    // Update the DMA next pointer and start the next buffer
    ADC->ADC_RNPR = (uint32_t)adc_buffers[packet_count % 2];
    ADC->ADC_RNCR = buffer_size;
    packet_count++;
  } 
}

/**
 * Configure the ADC hardware and start continuous conversions.
 */
void configureADC(){
  pmc_enable_periph_clk(ID_ADC);  // Enable ADC peripheral clock
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);  // Initialize the ADC
  NVIC_EnableIRQ(ADC_IRQn);  // Enable ADC interrupts
  adc_disable_all_channel(ADC);  // Disable all ADC channels initially
  
  ADC->ADC_MR |= 0x80;  // Enable free-running mode for continuous ADC conversions
  bitSet(ADC->ADC_MR, 10);  // Set ADC clock to approximately 48 kHz per channel
  ADC->ADC_CHER = 0x1455;  // Enable specific ADC channels (CH0, CH2, CH4, CH6, CH10, CH12)
  
  configureADCMode();  // Configure the ADC mode (Differential/Single-Ended)
  configureADCGain();  // Set the gain for the ADC channels
  
  adc_configure_sequence(ADC, adc_channels, 6);  // Configure ADC conversion sequence
  ADC->ADC_IDR = ~(1 << 27);  // Disable all ADC interrupts except the RX buffer complete interrupt
  ADC->ADC_IER = 1 << 27;  // Enable the RX buffer complete interrupt
}

/**
 * Configure ADC to either Differential or Single-Ended mode.
 * This function also takes into account if offset mode is enabled.
 */
void configureADCMode() {
  if (is_differential && !offset_enabled) {
    // Set channels to differential mode without offset correction
    ADC->ADC_COR = 0x14550000;
  } else if (is_differential && offset_enabled) {
    // Set channels to differential mode with offset correction
    ADC->ADC_COR = 0x14551455;
  } else if (!is_differential && offset_enabled) {
    // Set channels to single-ended mode with offset correction
    ADC->ADC_COR = 0x00001455;
  } else {
    // Set channels to single-ended mode without offset correction
    ADC->ADC_COR = 0x00000000;
  }
}

/**
 * Configure the gain for all ADC channels.
 * Gain values range from 0.5x to 4x depending on the gain_value.
 */
void configureADCGain() {
  // Set the gain value for each enabled ADC channel
  ADC->ADC_CGR = (gain_value << 0) |   // Gain for CH0
                 (gain_value << 2) |   // Gain for CH2
                 (gain_value << 4) |   // Gain for CH4
                 (gain_value << 6) |   // Gain for CH6
                 (gain_value << 10) |  // Gain for CH10
                 (gain_value << 12);   // Gain for CH12
}

/**
 * Configure DMA for ADC data transfers.
 * Two buffers are set up to continuously receive ADC data.
 */
void configureDMA(){
  ADC->ADC_RPR = (uint32_t)adc_buffers[0];  // Set the current DMA buffer
  ADC->ADC_RCR = buffer_size;  // Set the size of the current buffer
  ADC->ADC_RNPR = (uint32_t)adc_buffers[1];  // Set the next DMA buffer
  ADC->ADC_RNCR = buffer_size;  // Set the size of the next buffer
  ADC->ADC_PTCR = 1;  // Enable the DMA receiver
  ADC->ADC_CR = 2;  // Start ADC conversions
}

/**
 * Restart the ADC and DMA.
 * This is useful when changing ADC settings.
 */
void restartADC() {
  // Stop ADC and DMA
  ADC->ADC_PTCR = 1 << 1;  // Disable the DMA receiver
  ADC->ADC_CR = 1 << 1;    // Stop ADC conversions
  
  // Reconfigure the ADC and DMA
  configureADC();
  configureDMA();
}

/**
 * Send ADC data over SerialUSB.
 * This function sends a start marker, packet count, and the ADC data.
 */
void sendADCData(){
  SerialUSB.write((uint8_t *)start_marker, sizeof(start_marker));  // Send start marker
  SerialUSB.write((uint8_t *)&packet_count, sizeof(packet_count));  // Send packet count

  #ifdef SIMULATION
  SerialUSB.write((uint8_t *)simulation_buffer, sizeof(simulation_buffer));  // Send simulation data if in simulation mode
  #endif

  #ifndef SIMULATION
  SerialUSB.write((uint8_t *)adc_buffers[(packet_count - 1) % 2], 2 * buffer_size);  // Send real ADC data from the previous buffer
  #endif
}

/**
 * Main setup function.
 * This is run once at the beginning to initialize the system.
 */
void setup(){
  pinMode(RX_LED, OUTPUT);  // Set RX LED pin as output
  digitalWrite(RX_LED, 1);  // Turn off RX LED
  pinMode(TX_LED, OUTPUT);  // Set TX LED pin as output
  digitalWrite(TX_LED, 1);  // Turn off TX LED
  
  SerialUSB.begin(0);  // Begin SerialUSB communication
  while(!SerialUSB);  // Wait for SerialUSB to be ready
  start_marker[0] = start_marker_value;  // Set start marker
  
  configureADC();  // Configure ADC
  configureDMA();  // Configure DMA

  #ifdef SIMULATION
  // Initialize simulation buffer here (not implemented yet)
  #endif
}

/**
 * Main loop function.
 * Continuously checks for incoming serial commands and processes them.
 */
void loop(){
  if (SerialUSB.available() > 0) {
    digitalWrite(RX_LED, 0);  // Turn on RX LED when data is received
    // Read the incoming bytes into the buffer:
    serial_input_buffer = SerialUSB.readStringUntil('\n');
    
    if (serial_input_buffer == "START") {
      send_data = true;
      packet_count = 0;  // Reset packet count when starting data transmission
    }
    else if (serial_input_buffer == "STOP") {
      send_data = false;
      SerialUSB.flush();  // Stop sending data
    }
    else if (serial_input_buffer == "RESET") {
      send_data = false;
      SerialUSB.flush();
      SerialUSB.end();
      rstc_start_software_reset(RSTC);  // Reset the microcontroller
    }
    else if (serial_input_buffer.startsWith("SETMODE")) {
      // Set ADC mode (0 = Single-Ended, 1 = Differential)
      int mode = serial_input_buffer.substring(8).toInt();  // Get the mode value after "SETMODE"
      if (mode == 0) {
        is_differential = false;
      } else if (mode == 1) {
        is_differential = true;
      }
      restartADC();  // Restart the ADC to apply mode changes
    }
    else if (serial_input_buffer.startsWith("SETGAIN")) {
      // Set ADC gain (0 to 3 based on requested gain)
      int gain = serial_input_buffer.substring(8).toInt();  // Get the gain value after "SETGAIN"
      if (gain >= 0 && gain <= 3) {
        gain_value = gain;
      }
      restartADC();  // Restart the ADC to apply gain changes
    }
    else if (serial_input_buffer.startsWith("SETOFFSET")) {
      // Enable or disable offset mode
      int offset = serial_input_buffer.substring(10).toInt();  // Get the offset value after "SETOFFSET"
      if (offset == 0) {
        offset_enabled = false;
      } else if (offset == 1 ) {
        offset_enabled = true;
      }
      restartADC();  // Restart the ADC to apply offset changes
    }
  }

  // Wait for the next ADC DMA packet
  while(last_packet_count == packet_count);  

  if (send_data) {
    sendADCData();  // Send the ADC data when send_data flag is true
    digitalWrite(TX_LED, packet_count % 2);  // Toggle TX LED to indicate transmission
  }

  last_packet_count = packet_count;  // Update packet count tracker
  digitalWrite(RX_LED, 1);  // Turn off RX LED
}



