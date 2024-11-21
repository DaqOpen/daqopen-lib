/*
  Program Name: adc_dma_usb
  Description: continous sample 6 channel in differential mode and stream via usb to host pc

  Author: Michael Oberhofer
  Created on: 2017-05-01
  Last Updated: 2024-11-16

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

  Version: 0.98
  Github: https://github.com/DaqOpen/daqopen-lib/firmware/due-daq
  
  */
#undef HID_ENABLED

#define MAX_BUFFER_SIZE 20000  // Define the maximum size of the ADC buffers
#define START_MARKER_VALUE 0xFFFF  // Start marker value to indicate data transmission start
#define MAJOR_VERSION 0
#define MINOR_VERSION 98

#define RX_LED 72  // Define the pin for RX LED
#define TX_LED 73  // Define the pin for TX LED

#include <DueFlashStorage.h>
DueFlashStorage dueFlashStorage;

String serial_input_buffer;  // Buffer to hold incoming serial data
uint16_t adc_buffers[2][MAX_BUFFER_SIZE];  // Double buffer for ADC data storage
uint16_t start_marker[1];  // Marker indicating the start of ADC data transmission
volatile uint32_t packet_count = 0, last_packet_count = 0;  // Counters to track packets of ADC data
bool send_data = false;  // Flag to control whether data should be sent
bool is_differential = false;  // Variable for the ADC mode (Differential/Single-Ended)
bool offset_enabled = false;  // Flag to enable or disable offset mode
uint8_t gain_value = 0x00;  // Default gain (1x)
uint16_t buffer_size = 20000; // Used size of the buffer
uint16_t adc_prescal = 1; 
uint16_t adc_cher = 0x0040; // ADC_CHER_CH6; Enable Channel 6 = A1
const adc_channel_num_t adc_channels[] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_5, ADC_CHANNEL_4, ADC_CHANNEL_3, ADC_CHANNEL_2, ADC_CHANNEL_1, ADC_CHANNEL_0, ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13};

void configurePWM() {
  // PWM Set-up on pin: PB14
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                     // Enable PWM peripheral
  REG_PIOB_ABSR |= PIO_ABSR_P14;                        // Set PWM pin peripheral type A or B, in this case B for PB14
  REG_PIOB_PDR |= PIO_PDR_P14;                          // Set PWM pin to an output (disable PIO control on PB14)
  REG_PIOC_ABSR |= PIO_ABSR_P7;                        // Set PWM pin peripheral type A or B, in this case B for PB14
  REG_PIOC_PDR |= PIO_PDR_P7;                          // Set PWM pin to an output (disable PIO control on PB14)
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);      // Set the PWM clock rate to 84MHz (84MHz/1) 
  REG_PWM_CMR2 = PWM_CMR_CPRE_CLKA;                     // Enable single slope PWM and set the clock source as CLKA for Channel 2 (PB14 is PWM Channel 2)
  REG_PWM_CPRD2 = 8400;                                 // Set the PWM frequency 84MHz/10kHz = 8400 
  REG_PWM_CDTY2 = 4200;                                 // Set the PWM duty cycle 50% (8400/2=4200)
  REG_PWM_ENA = PWM_ENA_CHID2;                          // Enable the PWM channel 2                          // Enable the PWM channel     
}


/**
 * Set the serial number of the device
 */
void setSerial(uint16_t serial) {
  dueFlashStorage.write(0, (byte *) &serial, sizeof(serial));
}

/**
 * Read the serial number of the device
 */
uint16_t getSerial() {
  byte* b = dueFlashStorage.readAddress(0);
  uint16_t serial;
  memcpy(&serial, b, sizeof(serial));
  return(serial);
}

/**
 * Read the device id
 */
String getDeviceId() {
  uint32_t serial = getSerial();
  // "DueDaq,Protocol_Version,Serial_Number"
  String device_id_str = "DueDaq," + String(MAJOR_VERSION) + "." + String(MINOR_VERSION) + "," + String(serial) + "\n";
  return(device_id_str);
}

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
    PMC->PMC_PCER1 |= PMC_PCER1_PID37;                    // ADC power ON

  ADC->ADC_CR = ADC_CR_SWRST;                           // Reset ADC
  ADC->ADC_MR |=  ADC_MR_TRGEN_DIS                      // Free Running Mode selected
                  | ADC_MR_FREERUN
                  | ADC_MR_PRESCAL(adc_prescal);                  // Or PRESCAL (1) to reduce ADC frequency to 21 MHz

  ADC->ADC_ACR = ADC_ACR_IBCTL(0b01);                   // For frequencies > 500 KHz

  //adc_configure_sequence(ADC, adc_channels, 12);
  ADC->ADC_IER = ADC_IER_ENDRX;                         // End Of Conversion interrupt enable for channel 7
  NVIC_EnableIRQ(ADC_IRQn);                             // Enable ADC interrupt
  ADC->ADC_CHER = adc_cher;
}

/**
 * Configure ADC to either Differential or Single-Ended mode.
 * This function also takes into account if offset mode is enabled.
 */
void configureADCMode() {
  if (is_differential && !offset_enabled) {
    // Set channels to differential mode without offset correction
    ADC->ADC_COR = 0xFFFF0000;
  } else if (is_differential && offset_enabled) {
    // Set channels to differential mode with offset correction
    ADC->ADC_COR = 0xFFFFFFFF;
  } else if (!is_differential && offset_enabled) {
    // Set channels to single-ended mode with offset correction
    ADC->ADC_COR = 0x0000FFFF;
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
  uint32_t gain_mask = 0x55555555;
  ADC->ADC_CGR = gain_mask * gain_value;
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
  configureADCGain();
  configureADCMode();
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

  #ifdef EXTERNAL_SUPPLY
  PIOB->PIO_PER |= 1<<10;      // Never do this once Serial.begin() is called,
  PIOB->PIO_OER |= 1<<10;      // it will disconnect the programming USB port from the PC.
  PIOB->PIO_SODR |= 0<<10;     // UOTGVBOF HIGH  
  #endif

  SerialUSB.begin(0);  // Begin SerialUSB communication
  while(!SerialUSB);  // Wait for SerialUSB to be ready
  start_marker[0] = START_MARKER_VALUE;  // Set start marker
  
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
    else if (serial_input_buffer.startsWith("SETPRESCAL")) {
      // Enable or disable offset mode
      int prescal = serial_input_buffer.substring(11).toInt();  // Get the prescaler value after "SETPRESCAL"
      if (prescal >= 1 && prescal <= 255) {
        adc_prescal = prescal;
        restartADC();  // Restart the ADC to apply offset changes
      }
    }
    else if (serial_input_buffer.startsWith("SETCHANNEL")) {
      // Enable or disable offset mode
      adc_cher = serial_input_buffer.substring(11).toInt();  // Get the channel enable value after "SETCHANNEL"
      restartADC();  // Restart the ADC to apply offset changes
    }
    else if (serial_input_buffer.startsWith("SETDMABUFFERSIZE")) {
      // Enable or disable offset mode
      uint16_t dma_buffer_size = serial_input_buffer.substring(17).toInt();  // Get the prescaler value after "SETDMABUFFERSIZE"
      if (dma_buffer_size >= 1000 && dma_buffer_size <= MAX_BUFFER_SIZE) {
        buffer_size = dma_buffer_size;
        restartADC();  // Restart the ADC to apply offset changes
      }
    }
    else if (serial_input_buffer.startsWith("ENABLEPWM")) {
      // Enable experimental 10 kHz output for external charge pump on D53
      configurePWM();      
    }
    else if (serial_input_buffer.startsWith("SETSERIAL")) {
      uint16_t serial_number = serial_input_buffer.substring(10).toInt();
      setSerial(serial_number);
    }
    else if (serial_input_buffer.startsWith("GETSERIAL")) {
      uint16_t serial_number = getSerial();
      String serial_number_str = String(serial_number) + "\n";
      SerialUSB.print(serial_number_str);
    }
    else if (serial_input_buffer.startsWith("*IDN?")) {
      String device_id_str = getDeviceId();
      SerialUSB.print(device_id_str);
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



