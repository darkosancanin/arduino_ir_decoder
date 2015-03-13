// ************* Implementation Overview ************* 
// A timer interrupt is set up to trigger every 50 microseconds. On each interrupt a check is done 
// to see if the IR receiver pin is high or low with the pulses stored in the ir_pulses array.
// With a 16Mhz Arduino clock with the prescaler set to 8 that is 2 clock ticks every microsecond. 
//(System Clock Speed / prescaler / 1000000 which is microseconds per second or 16000000/8/1000000)
// The timer used is Timer 2 is a 8bit counter with a max value of 256 [Page 162].
// The counter starts (and reset after each interrupt) to 156. There is 100 ticks until
// the interrupt fires again. (With one microsecond taking 2 clock ticks, it requires 100 clock ticks
// to equal 50 microseconds. So we subtract this 100 from the max value of 256.)

#include <avr/interrupt.h>

#define IR_PIN_IS_ACTIVE !(PINB & (1 << PINB3)) // Check if the pin is high. PINB3 is Pin 11 on the Arduino. [Page 92] 
#define TIMER_COUNT_START_VALUE 156
#define PULSE_DECODING_MARGIN_OF_ERROR 150 // Number of microseconds to use as margin of error when decoding

#define STATE_IDLE  0
#define STATE_MARK  1
#define STATE_SPACE  2
#define STATE_FINISHED  3 

volatile uint16_t ir_pulses[200]; // Stores the pulse lengths, up to 200 pulses can be stored.
volatile uint16_t current_ir_pulse_index = 0;
volatile uint16_t number_of_pulses = 0;
volatile uint8_t state;
volatile uint16_t current_pulse_block_counter = 0; // Stores how many 50 microsecond blocks are in the currently pulse

void setup(){
    TCCR2A = 0;  // Initialize Timer/Counter 2 Control Register A which set the timer register to normal mode and not in compare mode [Page 158]
    TCCR2B |= (1 << CS21); //Set the Timer/Counter2 prescaler to 8 [Page 162]
    TIMSK2 |= (1 << TOIE2);  // Enable the overflow interrupt for Timer/Counter2 [Page 163] 
    TCNT2 = TIMER_COUNT_START_VALUE; // Set the initial Timer/Counter2 value. [Page 162]

    sei();  // Enable interrupts
  
    Serial.begin(9600);
  
    state = STATE_IDLE; // Initialize the global state
}

void loop(){
    if(state == STATE_FINISHED){
        cli();
        print_out_pulse_details();
        state = STATE_IDLE;
        sei();
    }
}

// Timer/Counter 2 overflow interrupt handler
ISR(TIMER2_OVF_vect)
{
    TCNT2 = TIMER_COUNT_START_VALUE; // Reset the initial Timer/Counter2 value.
  
    // Ignore interrupt if the state is finished, as it needs to print out to the serial port first
    if(state != STATE_FINISHED){ return; }
    
    if(state == STATE_IDLE){
        if(IR_PIN_IS_ACTIVE){
            state = STATE_MARK;
            current_pulse_block_counter = 1;
            ir_pulses[current_ir_pulse_index] = current_pulse_block_counter;
        } 
    }
    else if(state == STATE_MARK){
        if(IR_PIN_IS_ACTIVE){
            current_pulse_block_counter += 1; // increment the time block counter
            ir_pulses[current_ir_pulse_index] = current_pulse_block_counter;
        } 
        else{
            state = STATE_SPACE;
            current_pulse_block_counter = 1;
            current_ir_pulse_index += 1;
            ir_pulses[current_ir_pulse_index] = current_pulse_block_counter;
        }
    }
    else if(state == STATE_SPACE){
        // Timeout after 30,000us (600 counts of 50us) as the transmission must be complete
        if(current_pulse_block_counter >= 600){
            state = STATE_FINISHED;
            cli();
            print_out_pulse_details();
            state = STATE_IDLE;
            current_pulse_block_counter = 0;
            number_of_pulses = current_ir_pulse_index;
            current_ir_pulse_index = 0;
            sei();
        } 
        else{
            if(IR_PIN_IS_ACTIVE){
                state = STATE_MARK;
                current_pulse_block_counter = 1;
                current_ir_pulse_index += 1;
                ir_pulses[current_ir_pulse_index] = current_pulse_block_counter;
            } 
            else{
                current_pulse_block_counter += 1; // increment the time block counter
                ir_pulses[current_ir_pulse_index] = current_pulse_block_counter; 
            }
        }
    }
}

void print_out_pulse_details(){
    Serial.print("\nNumber of pulses = ");
    Serial.print(number_of_pulses, DEC);
    Serial.print("\n[");
    for(int i = 0; i < number_of_pulses; i++){
        Serial.print(ir_pulses[i] * 50, DEC);
        Serial.print(",");
    }
    Serial.print("]\n");
  
    if(!try_print_nec_details()){
        Serial.print("Unknown format.\n");  
    }
}

// Prints out the data if it uses the NEC protocol, returns 1 if it is NEC and 0 if it is not
// The NEC protocol that the remote uses expects commands in the following format (time in us): 
// leader: 9000 on, 4500 off
// 0 bit: 560 on, 1690 off
// 1 bit: 560 on, 565 off
// stop: 560 on
int try_print_nec_details(){
    // the NEC has 67 pulses, with leader bits of 9000 on then 4500 off
    if(number_of_pulses != 67 || !is_pulse_match(9000, ir_pulses[0]) || !is_pulse_match(4500, ir_pulses[1]) || !is_pulse_match(560, ir_pulses[66]))
        return 0;
    
    uint32_t nec_data = 0;
    // the value is stored in index 3 to 65, with lsb first 
    for(int i = 3; i < 66; i++){
        if(i % 2){ // check if its a off pulse, this is what determines if its a 0 bit or a 1 bit
            nec_data = nec_data >> 1;
            if(is_pulse_match(565, ir_pulses[i])){ // check if its a 1 bit
                nec_data = nec_data | 0b10000000000000000000000000000000;
            }
        }
        else{ // if its a on pulse, then validate it, as it should always be 560
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
    uint16_t actual_time = pulse_time_blocks *= 50;
    if((expected_value <= actual_time + PULSE_DECODING_MARGIN_OF_ERROR ) && (expected_value >= actual_time - PULSE_DECODING_MARGIN_OF_ERROR ))
        return 1;
    else{
        return 0;
    }
}



