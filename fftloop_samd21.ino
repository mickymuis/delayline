/*
 * A simple DDS + additive synthesis + FFT on microphone test for the SAMD21
 * An amplifier should be connected to DAC0
 * A microphone (amplifier) should be connected to A4
 */

// Include the CMSIS library for hardware DSP
#define ARM_MATH_CM0
#include <arm_math.h>

#define FFT_SIZE 32

// table of 256 sine values / one sine period / stored in flash memory
const int8_t sine256[]  = {
   0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 59, 62, 65, 67, 70, 73, 75, 78, 80, 82, 85, 87, 89, 91, 94, 96, 98, 100, 102, 103, 105, 107, 108, 110, 112, 113, 114, 116, 117, 118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126, 126, 126, 126, 126, 127, 126, 126, 126, 126, 126, 125, 125, 124, 123, 123, 122, 121, 120, 119, 118, 117, 116, 114, 113, 112, 110, 108, 107, 105, 103, 102, 100, 98, 96, 94, 91, 89, 87, 85, 82, 80, 78, 75, 73, 70, 67, 65, 62, 59, 57, 54, 51, 48, 45, 42, 39, 36, 33, 30, 27, 24, 21, 18, 15, 12, 9, 6, 3, 0, -3, -6, -9, -12, -15, -18, -21, -24, -27, -30, -33, -36, -39, -42, -45, -48, -51, -54, -57, -59, -62, -65, -67, -70, -73, -75, -78, -80, -82, -85, -87, -89, -91, -94, -96, -98, -100, -102, -103, -105, -107, -108, -110, -112, -113, -114, -116, -117, -118, -119, -120, -121, -122, -123, -123, -124, -125, -125, -126, -126, -126, -126, -126, -127, -126, -126, -126, -126, -126, -125, -125, -124, -123, -123, -122, -121, -120, -119, -118, -117, -116, -114, -113, -112, -110, -108, -107, -105, -103, -102, -100, -98, -96, -94, -91, -89, -87, -85, -82, -80, -78, -75, -73, -70, -67, -65, -62, -59, -57, -54, -51, -48, -45, -42, -39, -36, -33, -30, -27, -24, -21, -18, -15, -12, -9, -6, -3
};

q15_t adc_samples[2][FFT_SIZE*2] = {
{ 0, 1607, 3211, 4807, 6392, 7961, 9511, 11038, 12539, 14009, 15446, 16845, 18204, 19519, 20787, 22004, 23169, 24278, 25329, 26318, 27244, 28105, 28897, 29621, 30272, 30851, 31356, 31785, 32137, 32412, 32609, 32727, 32767, 32727, 32609, 32412, 32137, 31785, 31356, 30851, 30272, 29621, 28897, 28105, 27244, 26318, 25329, 24278, 23169, 22004, 20787, 19519, 18204, 16845, 15446, 14009, 12539, 11038, 9511, 7961, 6392, 4807, 3211, 1607,  },
{ 0, 1607, 3211, 4807, 6392, 7961, 9511, 11038, 12539, 14009, 15446, 16845, 18204, 19519, 20787, 22004, 23169, 24278, 25329, 26318, 27244, 28105, 28897, 29621, 30272, 30851, 31356, 31785, 32137, 32412, 32609, 32727, 32767, 32727, 32609, 32412, 32137, 31785, 31356, 30851, 30272, 29621, 28897, 28105, 27244, 26318, 25329, 24278, 23169, 22004, 20787, 19519, 18204, 16845, 15446, 14009, 12539, 11038, 9511, 7961, 6392, 4807, 3211, 1607,  }
};

const q15_t hamming32[32] = { 5, 6, 8, 11, 14, 19, 24, 30, 36, 42, 47, 52, 56, 60, 62, 63, 63, 62, 60, 56, 52, 47, 42, 36, 30, 24, 19, 14, 11, 8, 6, 5 };
const q15_t hamming64[64] = { 5, 5, 6, 7, 8, 9, 10, 12, 14, 16, 19, 21, 24, 27, 29, 32, 35, 38, 41, 44, 46, 49, 51, 54, 56, 58, 59, 61, 62, 63, 63, 63, 63, 63, 63, 62, 61, 59, 58, 56, 54, 51, 49, 46, 44, 41, 38, 35, 32, 29, 27, 24, 21, 19, 16, 14, 12, 10, 9, 8, 7, 6, 5, 5 };
const q15_t hamming128[128] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

arm_rfft_instance_q15 rfft_instance;

const uint8_t WIDTH = 5 ;                // The number of pulses
volatile uint32_t phase_accu[WIDTH];    // 32 bits phase accumulator for every waveform
volatile uint32_t tword[WIDTH];         // 32 bits tuning word for every waveform
volatile uint8_t amp_offset =0; // amplitude offset in array
volatile int16_t amplitude;
volatile bool update_dac = false;
const double freq[WIDTH] = { 1700, 2400, 3200, 4000, 5000 }; // Freq per waveform
const int treshold[WIDTH] = { 800, 450, 350, 300, 250 };
const int center_bin[WIDTH] = { 11, 15, 20, 25, 30 };
int pulse_vector_in[WIDTH];
int pulse_vector_out[WIDTH] = { 0, 0, 0, 0 , 0 };


const double CLOCKSPEED = 48e6; // 48 Mhz
const double CLOCKREF = CLOCKSPEED / (8 * 256); // reference clock freq

volatile uint16_t adc_result;

q15_t fft_out[FFT_SIZE*2] = {0};
volatile int adc_index =0;
volatile int adc_buffer =0;
volatile bool adc_window_ready =false;

volatile unsigned int irq_ovf_count = 0;

TcCount16* tcTimer( int t ) {
  switch( t ) {
      case 3:
        return (TcCount16*)TC3;
      case 4:
        return (TcCount16*)TC4;
      case 5:
        return (TcCount16*)TC5;
      default:
        break;
        
  }
  return (TcCount16*)NULL;
}

void setupTC8bit( int tc, unsigned int prescaler)
{
  TcCount16* TC = tcTimer( tc );                  // get timer struct
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;              // Disable TC
  while (TC->STATUS.bit.SYNCBUSY == 1); 
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT8;          // Set Timer counter Mode to 8 bits
  while (TC->STATUS.bit.SYNCBUSY == 1);
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ;         // Set TC as normal Normal Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); 
  TC->CTRLA.reg |= prescaler;       // Set prescaler to given argument
  while (TC->STATUS.bit.SYNCBUSY == 1);
  // Interrupts
  TC->INTENSET.reg = 0;                           // disable all interrupts
  TC->INTENSET.bit.OVF = 1;                       // enable overfollow interrup
  // Enable InterruptVector
  if( TC == (TcCount16*)TC3 )
    NVIC_EnableIRQ(TC3_IRQn);
  else
    NVIC_EnableIRQ(TC5_IRQn);
  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);           // wait for sync
}

void setupADC(void)
{
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;             
     
  REG_ADC_CTRLA &= ~(ADC_CTRLA_ENABLE);           //disable ADC module
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
  
  //software reset ADC module
  REG_ADC_CTRLA = ADC_CTRLA_SWRST;
  while ((REG_ADC_STATUS & ADC_STATUS_SYNCBUSY) || (REG_ADC_CTRLA & ADC_CTRLA_SWRST));
  
  //all interrupts disable
  REG_ADC_INTENCLR=ADC_INTENCLR_RESRDY|ADC_INTENCLR_OVERRUN|ADC_INTENCLR_WINMON|ADC_INTENCLR_SYNCRDY;
  
  //ADC clock sampling=48MHz/256, 16bit averaging, free-running mode
  REG_ADC_CTRLB=(ADC_CTRLB_PRESCALER_DIV256)|(ADC_CTRLB_RESSEL_10BIT)|(ADC_CTRLB_FREERUN);
  REG_ADC_AVGCTRL=ADC_AVGCTRL_ADJRES(1) | ADC_AVGCTRL_SAMPLENUM(1); 
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
          
  //ADC input setting , AIN[5]
  PORT->Group[0].PINCFG[5].reg =    PORT_PINCFG_PMUXEN;//PA5, Arduino A4
  PORT->Group[0].PMUX[3].reg &= ~(PORT_PMUX_PMUXE_Msk);
  PORT->Group[0].PMUX[3].reg |= PORT_PMUX_PMUXE_B;//ADC AIN[5]
  
  REG_ADC_INPUTCTRL=(ADC_INPUTCTRL_MUXPOS_PIN5)|(ADC_INPUTCTRL_MUXNEG_IOGND);
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
  
  //REG_ADC_REFCTRL= 0x3;//ADC_REFCTRL_REFSEL_INTVCC1; //V_REF setting, 1V internal
  REG_ADC_REFCTRL= 0x3;                         //V_REF setting, use external AREF
  REG_ADC_INTFLAG=ADC_INTFLAG_RESRDY;           //interrupt flag clearing
  
  //interrupt enable    
  NVIC_EnableIRQ(ADC_IRQn);
  REG_ADC_INTENSET=ADC_INTENSET_RESRDY;  
  
  REG_ADC_CTRLA |= (ADC_CTRLA_ENABLE);          //ADC module enable
  while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);
                    
}

void setup() {
  // Setup the clocks
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_DAC ) );
  while( GCLK->STATUS.bit.SYNCBUSY );
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TCC2_TC3 ) );
  while( GCLK->STATUS.bit.SYNCBUSY );
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_TC4_TC5 ) );
  while( GCLK->STATUS.bit.SYNCBUSY );
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_ADC ) );
  while( GCLK->STATUS.bit.SYNCBUSY ); 
  
  // Initialize all different waveforms
  for( uint8_t i =0; i < WIDTH; i++ ) {
    // Calculate tuning word
    tword[i] = pow( 2, 32 ) * freq[i] / CLOCKREF;
    phase_accu[i] =0;
  }

  // Arduino SerialUSB native usb port
  SerialUSB.begin( 20000000 );

  // Enable the debug led
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite( 7, HIGH );

  // Enable the DAC
  DAC->CTRLB.bit.EOEN = 1;
  DAC->CTRLB.bit.LEFTADJ =0;
  DAC->CTRLA.bit.ENABLE =1;
  while (DAC->STATUS.reg & DAC_STATUS_SYNCBUSY);

  // Initialize the CMSIS-DSP rFFT 
  arm_rfft_init_q15( &rfft_instance, FFT_SIZE, 0, 1 );
  arm_fill_q15( 0, adc_samples[0], FFT_SIZE*2 );
  arm_fill_q15( 0, adc_samples[1], FFT_SIZE*2 );

  // Setup two timers, one for DDS and one for control
  setupTC8bit( 3, TC_CTRLA_PRESCALER_DIV8 );
  //setupTC8bit( 5, TC_CTRLA_PRESCALER_DIV64 );
  // Setup the ADC
  setupADC();

  // Start ADC
  REG_ADC_SWTRIG = ADC_SWTRIG_START;

}

uint16_t sdiv =0;

void loop() {
  while(!adc_window_ready);
  adc_window_ready =false;
  //NVIC_DisableIRQ(ADC_IRQn);

  arm_rfft_q15( &rfft_instance, adc_samples[!adc_buffer], fft_out );
  arm_abs_q15( fft_out, fft_out, FFT_SIZE );
  //arm_sub_q15( fft_out, noisefloor32, fft_out, FFT_SIZE );

  for( int i =0; i < WIDTH; i++ ) {
    int accumulator = fft_out[center_bin[i]-1] + fft_out[center_bin[i]] + fft_out[center_bin[i]+1];
    if( accumulator >= treshold[i] )
      pulse_vector_in[i] =1;
    else
      pulse_vector_in[i] =0;
  }

  if( ++sdiv % 512 ) {
    /*for( int i =0; i < FFT_SIZE; i++ ) {
      SerialUSB.print( fft_out[i] );
      SerialUSB.print( ", " );
    }
    SerialUSB.println( "Done" );*/

    for( int i =0; i < WIDTH; i++ ) {
      SerialUSB.print( pulse_vector_in[i] );
    }
    SerialUSB.println( "" );

    if( SerialUSB.available() ) {
      char c =SerialUSB.read();

      int index = (c - '0');
      if( index >= 0 && index < WIDTH )
        pulse_vector_out[index] = !pulse_vector_out[index];
    }
  }
  
}

void dac_write( uint16_t x ) {
 // wait till it's ready
  while (DAC->STATUS.reg & DAC_STATUS_SYNCBUSY);

  if( x > 0x3ff ) x = 0x3ff;
  // and write the data  
  DAC->DATA.reg = x;
}

void dds_step() {
  amplitude =0;
  
  for( uint8_t i =0; i < WIDTH; i++ ) {
    phase_accu[i] += tword[i];
    int8_t a = sine256[ (phase_accu[i] >> 24) ];
    //a -= 127;
    if( pulse_vector_out[i] )
      amplitude += a;
  }

  amplitude /= 2;

  //amplitude = amplitude * 4;
  dac_write( amplitude + 511 );
}

void TC3_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  dds_step();
  TC->INTFLAG.bit.OVF = 1;    // writing a one clears the ovf flag
}

// TC5 is used for testing and is disabled by default...
void TC5_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC5; // get timer struct
  TC->INTFLAG.bit.OVF = 1;    // writing a one clears the ovf flag
  irq_ovf_count++;

  if( irq_ovf_count == 1000 ) {
    for( int i =WIDTH-1; i >= 0; i-- ) {
      if( pulse_vector_out[i] )
        pulse_vector_out[i] = 0;
      else {
        pulse_vector_out[i] = 1;
        break;
      }
    }
    irq_ovf_count =0;
  }

}

void ADC_Handler(void)
{
  adc_result = REG_ADC_RESULT;  

  adc_samples[adc_buffer][adc_index] = ((q15_t)(adc_result) - 511) * hamming32[adc_index];

  if( ++adc_index == FFT_SIZE ) {
    adc_index =0;
    adc_window_ready =true;
    adc_buffer = !adc_buffer;

    //arm_fill_q15( 0, fft_out, FFT_SIZE*2 );
    //arm_rfft_q15( &rfft_instance, sine128, fft_out );
  }
     
  //clearing interrupt flag
  REG_ADC_INTFLAG=ADC_INTFLAG_RESRDY;   
}

void resetStartTC( TcCount16* TC )
{
  TC->CTRLBSET.reg |= TC_CTRLBCLR_CMD_RETRIGGER;   // restart
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void stopTC( TcCount16* TC )
{
  TC->CTRLBSET.reg |= TC_CTRLBSET_CMD_STOP;   // Stop counter
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTC( TcCount16* TC )
{
  TC->CTRLBSET.reg |= TC_CTRLBSET_CMD_RETRIGGER;   //  Start
  while (TC->STATUS.bit.SYNCBUSY == 1);
} 


