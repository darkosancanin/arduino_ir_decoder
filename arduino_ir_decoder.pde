#include <WProgram.h>
#include <avr/interrupt.h>

#define INPUT_PIN_NUMBER 11  //the ir receiver pin
#define IR_PIN_IS_ACTIVE !(PINB & (1 << PB3)) // if the ir receiver pin is low/active, does not use arduino library for speed 
#define PULSE_TIME_BLOCKS_IN_MICROSECONDS 50  // microseconds per clock interrupt tick (can range from 0.5 to 128 microseconds minus fudge factor)
#define CLOCK_FUDGE_FACTOR 2      // fudge factor for clock interrupt overhead
#define MAX_TIMER_CLOCK_VALUE 256      // max value for timer clock count (timer 2)
#define PRESCALE 8      // timer clock prescale
#define SYSTEM_CLOCK 16000000  // main arduino clock
#define TIMER_CLOCKS_PER_MICROSECOND (SYSTEM_CLOCK/PRESCALE/1000000)   // timer clocks per microsecond
#define PULSE_DECODING_MARGIN_OF_ERROR 150 // number of microseconds to use as margin of error when decoding
#define INITIAL_TIMER_COUNT_VALUE (MAX_TIMER_CLOCK_VALUE - PULSE_TIME_BLOCKS_IN_MICROSECONDS*TIMER_CLOCKS_PER_MICROSECOND + CLOCK_FUDGE_FACTOR)  // clock timer initial value

#define STATE_IDLE  0
#define STATE_MARK  1
#define STATE_SPACE  2
#define STATE_FINISHED  3 

volatile uint16_t ir_pulses[200]; // stores the pulse lengths, up to 200 pulses can be stored
volatile uint16_t current_ir_pulse_index = 0;
volatile uint16_t number_of_pulses = 0;
volatile uint8_t state;
volatile uint16_t timeblock_counter = 0;

void setup(){
  TCCR2A = 0;  // set the timer register to normal mode i.e. not in compare mode

  //set the timer prescale to 8
  TCCR2B &= ~(1 << CS22);
  TCCR2B |= (1 << CS21);
  TCCR2B &= ~(1 << CS20);

  TIMSK2 |= (1 << TOIE2);  // timer2 overflow interrupt enable

  TCNT2 = INITIAL_TIMER_COUNT_VALUE; // reset timer value

  sei();  // enable interrupts
  
  pinMode(INPUT_PIN_NUMBER, INPUT);  // enable input pin
  
  Serial.begin(9600); // set up the serial port
  
  state = STATE_IDLE; // initialize the state
}

void loop(){
  if(state == STATE_FINISHED){
    cli();
    print_out_pulse_details();
    state = STATE_IDLE;
    sei();
  }
}

ISR(TIMER2_OVF_vect)
{
  TCNT2 = INITIAL_TIMER_COUNT_VALUE; // reset timer value
  
  // do not update anything if the state is finished, as it needs to print out to the serial port first
  if(state != STATE_FINISHED){
    timeblock_counter += 1; // increment the time block counter
    
    if(state == STATE_IDLE){
      if(IR_PIN_IS_ACTIVE){
        state = STATE_MARK;
        timeblock_counter = 1;
        ir_pulses[current_ir_pulse_index] = timeblock_counter;
      } 
    }
    else if(state == STATE_MARK){
      if(IR_PIN_IS_ACTIVE){
        ir_pulses[current_ir_pulse_index] = timeblock_counter; // increment the pulse count
      } 
      else{
        state = STATE_SPACE;
        timeblock_counter = 1;
        current_ir_pulse_index += 1;
        ir_pulses[current_ir_pulse_index] = timeblock_counter;
      }
    }
    else if(state == STATE_SPACE){
      // if we are actively receiving a ir input and we are waiting
      // for the next input for too long (over 30000 us) then the transmission must be complete
      if(timeblock_counter >= 600){
        state = STATE_FINISHED;
        timeblock_counter = 0;
        number_of_pulses = current_ir_pulse_index;
        current_ir_pulse_index = 0;
      } 
      else{
        if(IR_PIN_IS_ACTIVE){
          state = STATE_MARK;
          timeblock_counter = 1;
          current_ir_pulse_index += 1;
          ir_pulses[current_ir_pulse_index] = timeblock_counter;
        } 
        else{
          ir_pulses[current_ir_pulse_index] = timeblock_counter; // increment the pulse count
        }
      }
    }
  }
}

void print_out_pulse_details(){
  Serial.print("\nNumber of pulses = ");
  Serial.print(number_of_pulses, DEC);
  Serial.print("\n[");
  for(int i = 0; i < number_of_pulses; i++){
    Serial.print(ir_pulses[i] * PULSE_TIME_BLOCKS_IN_MICROSECONDS, DEC);
    Serial.print(",");
  }
  Serial.print("]\n");
  
  if(!try_print_nec_details()){
    Serial.print("Unknown format.\n");  
  }
}

// prints out the data if it uses the NEC protocol, returns 1 if it is NEC and 0 if it is not
int try_print_nec_details(){
  // the NEC has 67 pulses, with leader bits of 9000 on then 4500 off
  if(number_of_pulses != 67 || !is_pulse_match(9000, ir_pulses[0]) || !is_pulse_match(4500, ir_pulses[1]) || !is_pulse_match(560, ir_pulses[66]))
    return 0;
    
  uint32_t nec_data = 0;
  // the value is stored in index 3 to 65, with lsb first 
  for(int i = 3; i < 66; i++){
    if(i % 2){ // if its a off pulse, this is what determines if its a 0 bit or a 1 bit
      nec_data = nec_data >> 1;
      if(is_pulse_match(565, ir_pulses[i])){ // check if its a 1 bit
        nec_data = nec_data | 0b10000000000000000000000000000000;
      }
    }
    else{ // if its a on pulse, then validate it as it should always be 560
      if(!is_pulse_match(560, ir_pulses[i]))
        return 0;
    }
  }
  
  // print out the data in binary format
  Serial.print("NEC Data : ");
  for(int i = 0; i < 32; i++){
  if(nec_data & 0b10000000000000000000000000000000){
    Serial.print("1");
  }
  else{
    Serial.print("0");
  }
    nec_data = nec_data << 1;
  }
  Serial.print("\n"); 
  return 1;
}

// returns 1 if the pulse matches, takes into account a margin of error of either longer or shorter than actual length
int is_pulse_match(uint16_t expected_value, uint16_t pulse_time_blocks){
  uint16_t actual_time = pulse_time_blocks *= PULSE_TIME_BLOCKS_IN_MICROSECONDS;
  if((expected_value <= actual_time + PULSE_DECODING_MARGIN_OF_ERROR ) && (expected_value >= actual_time - PULSE_DECODING_MARGIN_OF_ERROR ))
    return 1;
  else{
    return 0;
  }
}



