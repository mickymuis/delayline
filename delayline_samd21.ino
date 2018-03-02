/*
 * The most basic delay line memory for SAMD21
 * An amplifier should be connected to DAC0
 * A microphone (amplifier) should be connected to A4
 * A 50% duty-cycle squarewave is driven to DAC0 and is used to generate a pulse array.
 * The length of the memory can be adjusted by changing the distance of the speaker.
 */

#define WINDOW_SIZE 64
#define MAX_LENGTH 32
#define LENGTH 6

#define DETECTION_TOTAL_PWR
//#define DETECTION_PEAK_COUNT

// Number of samples to skip at the window boundary
int dc =8;

volatile int square_index =0;
volatile int square_phase =0;
const int square_phase_length = 4;
volatile int square_enable =1;

volatile int adc_result;
uint32_t adc_treshold = 80;
#define ADC_TRESHOLD_STEP 5

volatile int adc_index =0;
volatile int adc_over =0;
volatile int adc_under =0;
volatile uint32_t adc_accu =0;
volatile bool adc_window_ready =false;
volatile bool full_cycle_ready =false;

const int pulse_distance =1;
volatile int cycle_length = LENGTH;
volatile int pulse_count =LENGTH-1;
int command_queue[MAX_LENGTH] = {0};
volatile int print_buffer[MAX_LENGTH] = {0};


volatile unsigned int irq_ovf_count = 0;

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
  
  //ADC clock sampling=48MHz/128, 8bit, free-running mode
  REG_ADC_CTRLB=(ADC_CTRLB_PRESCALER_DIV128)|(ADC_CTRLB_RESSEL_8BIT)|(ADC_CTRLB_FREERUN);
  REG_ADC_AVGCTRL=ADC_AVGCTRL_ADJRES(1) | ADC_AVGCTRL_SAMPLENUM(1); // samples average
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
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCM_ADC ) );
  while( GCLK->STATUS.bit.SYNCBUSY ); 
  
  // Arduino SerialUSB native usb port
  //SerialUSB.begin( 115200 );
  SerialUSB.begin( 20000000 );

  // Enable the debug led
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite( 7, HIGH );

  //analogWriteResolution(10);
  //analogWrite(A0, 0x200);

  // Enable the DAC
  DAC->CTRLB.bit.EOEN = 1;
  DAC->CTRLB.bit.LEFTADJ =0;
  DAC->CTRLA.bit.ENABLE =1;
  while (DAC->STATUS.reg & DAC_STATUS_SYNCBUSY);

  // Setup the ADC
  setupADC();

  // Start ADC
  REG_ADC_SWTRIG = ADC_SWTRIG_START;

}

uint16_t sdiv =0;

void loop() {
  while(!full_cycle_ready);
  full_cycle_ready =false;
  //SerialUSB.println( adc_result );

  for( int i =0; i < cycle_length; i+= pulse_distance) {
    SerialUSB.print( print_buffer[i] );
  }
  SerialUSB.println("");

  //if( ++sdiv % 512 ) {

  if( SerialUSB.available() ) {
    char c =SerialUSB.read();
    if( c == 'a' ) {
      cycle_length++;
    } else if( c == 'z' ) {
      cycle_length--;
    } else if( c == 's' ) {
      adc_treshold += ADC_TRESHOLD_STEP;
    } else if( c == 'x' ) {
      adc_treshold -= ADC_TRESHOLD_STEP;
    } else if( c == 'd' ) {
      dc += square_phase_length;
    } else if( c == 'c' ) {
      dc -= square_phase_length;
    }else {
      int index = (c - '0') * pulse_distance;
      if( index >= 0 && index < cycle_length )
        command_queue[index] =1;
    }
  }
}

void dac_write( uint16_t x ) {
 // wait till it's ready
  while (DAC->STATUS.reg & DAC_STATUS_SYNCBUSY);

  if( x > 0x3FF ) x = 0x3FF;
  // and write the data  
  DAC->DATA.reg = x;
}

void square_step() {
  if( !square_enable ) {
    //dac_write( 0x1ff);
    return; 
  }
  if( ++square_index == square_phase_length ) {
    square_phase = !square_phase;
    //if( ++square_phase == 4 ) square_phase =0;
    square_index =0;
    if( square_phase )
      dac_write( 0x3ff );
    else
      dac_write( 0x1ff );
    /*switch( square_phase ) {
     case 0:
     case 2:
     case 3:
       dac_write( 0x1ff);
       break;
     case 1:
       dac_write( 0x3ff );
       break;
   }*/
  }
  
  
}

void ADC_Handler(void)
{
  int16_t i = REG_ADC_RESULT;
  
  if( adc_index > dc && adc_index < WINDOW_SIZE - dc ) {
    i -= 127;
    //i = (i + (i >> 15)) ^ (i >> 15);
    i = abs(i);
    #ifdef DETECTION_PEAK_COUNT
    if( i > adc_treshold )
      adc_over ++;
    else
      adc_under ++;
    #endif
    #ifdef DETECTION_TOTAL_PWR
    adc_accu += i;
    #endif
  }

  if( ++adc_index == WINDOW_SIZE ) {
      #ifdef DETECTION_PEAK_COUNT
      if( adc_over >= adc_under )
        adc_result =1;
      else
        adc_result =0;
      adc_over = adc_under = adc_index =0;
      #endif
      #ifdef DETECTION_TOTAL_PWR
      if( adc_accu > (adc_treshold * (WINDOW_SIZE-dc*2)) )
        adc_result =1;
      else
        adc_result =0;
      adc_index = adc_accu =0;
      #endif

      if( pulse_count % pulse_distance != 0 )
        square_enable =0;
      else {
        square_enable =command_queue[pulse_count] ? !adc_result : adc_result;
        command_queue[pulse_count] =0;
        print_buffer[pulse_count] = square_enable;

      }

      if( --pulse_count < 0 ) {
        full_cycle_ready =true;
        pulse_count =cycle_length-1;
      }
      
      adc_window_ready =true;
  }

  square_step();

  //clearing interrupt flag
  REG_ADC_INTFLAG=ADC_INTFLAG_RESRDY;   
}

