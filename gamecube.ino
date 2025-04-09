/**
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 * Modified by Joef112 & jarutherford (Raphnet Curves)
 * Analog L & R disabled, GCZ=N64 CDown, GC CDown=N64 L.
 */

/**
 * To use, hook up the following to the Arduino Duemilanove:
 * Digital I/O 2: Gamecube controller serial line
 * Digital I/O 8: N64 serial line
 * All appropriate grounding and power lines
 * A 1K resistor to bridge digital I/O 2 and the 3.3V supply
 *
 * The pin-out for the N64 and Gamecube wires can be found here:
 * http://svn.navi.cx/misc/trunk/wasabi/devices/cube64/hardware/cube64-basic.pdf
 * Note: that diagram is not for this project, but for a similar project which
 * uses a PIC microcontroller. However, the diagram does describe the pinouts
 * of the gamecube and N64 wires.
 */

/*
 Copyright (c) 2009 Andrew Brown

 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 */

#include "pins_arduino.h"

// Pin definitions remain the same...
#define GC_PIN 2
#define GC_PIN_DIR DDRD
#define GC_HIGH DDRD &= ~0x04
#define GC_LOW DDRD |= 0x04
#define GC_QUERY (PIND & 0x04)

#define N64_PIN 8
#define N64_HIGH DDRB &= ~0x01
#define N64_LOW DDRB |= 0x01
#define N64_QUERY (PINB & 0x01)

// Define conversion modes
enum ConversionMode {
    CONVERSION_MODE_2v0,
    CONVERSION_MODE_1v5,
    CONVERSION_MODE_EXTENDED
};

// Configuration structure
struct Config {
    ConversionMode conversion_mode;
    bool deadzone_enabled;
} convert;

// Variables for stick mapping
static char gc_x_origin = 0;
static char gc_y_origin = 0;

// 8 bytes of data that we get from the controller
static struct {
    unsigned char data1;
    unsigned char data2;
    unsigned char stick_x;
    unsigned char stick_y;
    unsigned char cstick_x;
    unsigned char cstick_y;
    unsigned char left;
    unsigned char right;
} gc_status;

static char n64_raw_dump[281];
static unsigned char n64_command;
static unsigned char zero_x;
static unsigned char zero_y;
static unsigned char n64_buffer[33];

// Function declarations
static void gc_send(unsigned char *buffer, char length);
static int gc_get();
static void init_gc_controller();
static void print_gc_status();
static void get_n64_command();
void setOriginsFromStatus();

#include "crc_table.h"

static void init_gc_controller()
{
    // Initialize the gamecube controller by sending it a null byte.
    // This is unnecessary for a standard controller, but is required for the
    // Wavebird.
    unsigned char initialize = 0x00;
    gc_send(&initialize, 1);

    // Stupid routine to wait for the gamecube controller to stop
    // sending its response. We don't care what it is, but we
    // can't start asking for status if it's still responding
    int x;
    for (x=0; x<64; x++) {
        // make sure the line is idle for 64 iterations, should
        // be plenty.
        if (!GC_QUERY)
            x = 0;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("Code has started!");
    Serial.flush();
    
    // Status LED
    digitalWrite(13, LOW);
    pinMode(13, OUTPUT);
    
    // Communication pins setup
    digitalWrite(GC_PIN, LOW);  
    pinMode(GC_PIN, INPUT);
    digitalWrite(N64_PIN, LOW);
    pinMode(N64_PIN, INPUT);

    noInterrupts();
    init_gc_controller();

    // Initialize stick mapping configuration
    convert.conversion_mode = CONVERSION_MODE_2v0;
    convert.deadzone_enabled = true;

    // Read initial stick position
    do {
        unsigned char command[] = {0x40, 0x03, 0x00};
        gc_send(command, 3);
        gc_get();
        interrupts();
        zero_x = 0;  // Set to center position
        zero_y = 0;  // Set to center position
        Serial.print("Setting neutral point to: ");
        Serial.print(zero_x, DEC);
        Serial.print(", ");
        Serial.println(zero_y, DEC);
        Serial.flush();
    } while (false);  // Only do once, assuming controller is centered at startup

    setOriginsFromStatus();
}

// Stick mapping functions
void setOriginsFromStatus() {
    gc_x_origin = 0;  // Center position
    gc_y_origin = 0;  // Center position
    
    Serial.print("Origins set to - X: ");
    Serial.print(gc_x_origin, DEC);
    Serial.print(" Y: ");
    Serial.println(gc_y_origin, DEC);
}

int calb(char orig, unsigned char val) {
    short tmp;
    long mult = 26000; // V1.6 default
    char dz = 0;

    if (convert.conversion_mode == CONVERSION_MODE_1v5) {
        mult = 25000;
    }

    if (convert.deadzone_enabled) {
        dz = 12;
        mult = 30000; // V1.6
        if (convert.conversion_mode == CONVERSION_MODE_1v5) {
            mult = 29000;
        }
    }

    tmp = (signed char)(val^0x80) - orig;

    if (dz) {
        if (tmp > 0) {
            if (tmp < dz) {
                tmp = 0;
            } else {
                tmp -= dz;
            }
        }
        else if (tmp < 0) {
            if (tmp > -dz) {
                tmp = 0;
            } else {
                tmp += dz;
            }
        }
    }

    if (convert.conversion_mode != CONVERSION_MODE_EXTENDED) {
        tmp = tmp * mult / 32000L;
    }

    if (tmp <= -127)
        tmp = -127;

    if (tmp > 127)
        tmp = 127;

    return tmp;
}


void gamecubeXYtoN64(unsigned char x, unsigned char y, char *dst_x, char *dst_y) {
    unsigned char abs_y, abs_x;
    long sig_x, sig_y;
    long sx, sy;
    int n64_maxval = 80; // Version 1.6
    long l = 256; // Version 1.6

    if (convert.conversion_mode == CONVERSION_MODE_EXTENDED) {
        sig_x = calb(gc_x_origin, x);
        sig_y = calb(gc_y_origin, y);

        *dst_x = sig_x;
        *dst_y = sig_y;
        return;
    }

    if (convert.conversion_mode == CONVERSION_MODE_1v5) {
        l = 512;
        n64_maxval = 127;
    }

    sig_x = calb(gc_x_origin, x);
    sig_y = calb(gc_y_origin, y);

    abs_y = abs(sig_y);
    abs_x = abs(sig_x);

    sx = sig_x + sig_x * abs_y / l;
    sy = sig_y + sig_y * abs_x / l;

    // Clamp to N64 range
    if (sx <= -n64_maxval) sx = -n64_maxval;
    if (sx > n64_maxval) sx = n64_maxval;
    if (sy <= -n64_maxval) sy = -n64_maxval;
    if (sy > n64_maxval) sy = n64_maxval;

    *dst_x = sx;
    *dst_y = sy;
}

void mapGamecubeToN64() {
    memset(n64_buffer, 0, sizeof(n64_buffer));

    // Button mappings
    n64_buffer[0] |= (gc_status.data1 & 0x01) << 7; // A -> A
    n64_buffer[0] |= (gc_status.data1 & 0x02) << 5; // B -> B
    if (gc_status.data2 & 0x10) {
        n64_buffer[1] |= 0x04; // N64 C Down
    }
    n64_buffer[0] |= (gc_status.data1 & 0x10); // Start -> Start
    n64_buffer[0] |= (gc_status.data2 & 0x01) << 1; // D-pad mapped
    n64_buffer[0] |= (gc_status.data2 & 0x02) >> 1; // D-pad mapped
    n64_buffer[0] |= (gc_status.data2 & 0x04); // D-pad mapped
    n64_buffer[0] |= (gc_status.data2 & 0x08); // D-pad mapped

    n64_buffer[0] |= (gc_status.data2 & 0x40) >> 1; // L -> Z
    n64_buffer[1] |= (gc_status.data2 & 0x20) >> 1; // R -> R

    n64_buffer[1] |= (gc_status.data1 & 0x08) >> 2; // Y -> Cleft
    n64_buffer[1] |= (gc_status.data1 & 0x04) >> 2; // X -> Cright

    // C-stick mapping
    if (gc_status.cstick_x < 0x50) n64_buffer[1] |= 0x02; // C-left
    if (gc_status.cstick_x > 0xB0) n64_buffer[1] |= 0x01; // C-right
    if (gc_status.cstick_y < 0x50) n64_buffer[1] |= 0x20; // N64 L
    if (gc_status.cstick_y > 0xB0) n64_buffer[1] |= 0x08; // C-up

    // Use the conversion functions for stick values
    char n64_x, n64_y;
    gamecubeXYtoN64(gc_status.stick_x, gc_status.stick_y, &n64_x, &n64_y);
    n64_buffer[2] = n64_x;
    n64_buffer[3] = n64_y;
}

/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * hardcoded for Arduino DIO 2 and external pull-up resistor
 */
static void gc_send(unsigned char *buffer, char length)
{
    asm volatile (
            "; Start of gc_send assembly\n"

            // passed in to this block are:
            // the Z register (r31:r30) is the buffer pointer
            // %[length] is the register holding the length of the buffer in bytes

            // Instruction cycles are noted in parentheses
            // branch instructions have two values, one if the branch isn't
            // taken and one if it is

            // r25 will be the current buffer byte loaded from memory
            // r26 will be the bit counter for the current byte. when this
            // reaches 0, we need to decrement the length counter, load
            // the next buffer byte, and loop. (if the length counter becomes
            // 0, that's our exit condition)
            
            "ld r25, Z\n" // load the first byte

            // This label starts the outer loop, which sends a single byte
            ".L%=_byte_loop:\n"
            "ldi r26,lo8(8)\n" // (1)

            // This label starts the inner loop, which sends a single bit
            ".L%=_bit_loop:\n"
            "sbi 0xa,2\n" // (2) pull the line low

            // line needs to stay low for 1µs for a 1 bit, 3µs for a 0 bit
            // this block figures out if the next bit is a 0 or a 1
            // the strategy here is to shift the register left, then test and
            // branch on the carry flag
            "lsl r25\n" // (1) shift left. MSB goes into carry bit of status reg
            "brcc .L%=_zero_bit\n" // (1/2) branch if carry is cleared

            
            // this block is the timing for a 1 bit (1µs low, 3µs high)
            // Stay low for 16 - 2 (above lsl,brcc) - 2 (below cbi) = 12 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\n" // (2)
            "cbi 0xa,2\n" // (2) set the line high again
            // Now stay high for 2µs of the 3µs to sync up with the branch below
            // 2*16 - 2 (for the rjmp) = 30 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "rjmp .L%=_finish_bit\n" // (2)


            // this block is the timing for a 0 bit (3µs low, 1µs high)
            // Need to go high in 3*16 - 3 (above lsl,brcc) - 2 (below cbi) = 43 cycles
            ".L%=_zero_bit:\n"
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\n" // (3)
            "cbi 0xa,2\n" // (2) set the line high again


            // The two branches meet up here.
            // We are now *exactly* 3µs into the sending of a bit, and the line
            // is high again. We have 1µs to do the looping and iteration
            // logic.
            ".L%=_finish_bit:\n"
            "subi r26,1\n" // (1) subtract 1 from our bit counter
            "breq .L%=_load_next_byte\n" // (1/2) branch if we've sent all the bits of this byte

            // At this point, we have more bits to send in this byte, but the
            // line must remain high for another 1µs (minus the above
            // instructions and the jump below and the sbi instruction at the
            // top of the loop)
            // 16 - 2(above) - 2 (rjmp below) - 2 (sbi after jump) = 10
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "rjmp .L%=_bit_loop\n"


            // This block starts 3 cycles into the last 1µs of the line being high
            // We need to decrement the byte counter. If it's 0, that's our exit condition.
            // If not we need to load the next byte and go to the top of the byte loop
            ".L%=_load_next_byte:\n"
            "subi %[length], 1\n" // (1)
            "breq .L%=_loop_exit\n" // (1/2) if the byte counter is 0, exit
            "adiw r30,1\n" // (2) increment byte pointer
            "ld r25, Z\n" // (2) load the next byte
            // delay block:
            // needs to go high after 1µs or 16 cycles
            // 16 - 9 (above) - 2 (the jump itself) - 3 (after jump) = 2
            "nop\nnop\n" // (2)
            "rjmp .L%=_byte_loop\n" // (2)


            // Loop exit
            ".L%=_loop_exit:\n"

            // final task: send the stop bit, which is a 1 (1µs low 3µs high)
            // the line goes low in:
            // 16 - 6 (above since line went high) - 2 (sbi instruction below) = 8 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\n" // (3)
            "sbi 0xa,2\n" // (2) pull the line low
            // stay low for 1µs
            // 16 - 2 (below cbi) = 14
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\n" // (4)
            "cbi 0xa,2\n" // (2) set the line high again

            // just stay high. no need to wait 3µs before returning

            :
            // outputs:
            "+z" (buffer) // (read and write)
            :
            // inputs:
            [length] "r" (length)
            :
            // clobbers:
                "r25", "r26"
            );

}
/**
 * Complete copy and paste of gc_send, but with the N64
 * pin being manipulated instead.
 */
static void n64_send(unsigned char *buffer, char length, bool wide_stop)
{
    asm volatile (";Starting N64 Send Routine");
    // Send these bytes
    char bits;
    
    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop
    
    asm volatile (";Starting outer for loop");
outer_loop:
    {
        asm volatile (";Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            N64_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (*buffer >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile (";Setting line to high");
                N64_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              );

            } else {
                asm volatile (";Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\n");

                asm volatile (";Setting line to high");
                N64_HIGH;

                // wait for 1us
                asm volatile ("; end of conditional branch, need to wait 1us more before next bit");
                
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // branch path

            asm volatile (";finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                // this block is why a for loop was impossible
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                // rotate bits
                asm volatile (";rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile (";continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    N64_LOW;
    // wait 1 us, 16 cycles, then raise the line 
    // take another 3 off for the wide_stop check
    // 16-2-3=11
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\n");
    if (wide_stop) {
        asm volatile (";another 1us for extra wide stop bit\n"
                      "nop\nnop\nnop\nnop\nnop\n"
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\n");
    }

    N64_HIGH;

}

// Read 8 bytes from the gamecube controller
// hardwired to read from Arduino DIO2 with external pullup resistor
static int gc_get()
{
    // listen for the expected 8 bytes of data back from the controller and
    // and pack it into the gc_status struct.
    asm volatile (";Starting to listen");

    // treat the 8 byte struct gc_status as a raw char array.
    unsigned char *bitbin = (unsigned char*) &gc_status;

    unsigned char retval;

    asm volatile (
            "; START OF MANUAL ASSEMBLY BLOCK\n"
            // r25 is our bit counter. We read 64 bits and increment the byte
            // pointer every 8 bits
            "ldi r25,lo8(0)\n"
            // read in the first byte of the gc_status struct
            "ld r23,Z\n"
            // default exit value is 1 (success)
            "ldi %[retval],lo8(1)\n"

            // Top of the main read loop label
            "L%=_read_loop:\n"

            // This first spinloop waits for the line to go low. It loops 64
            // times before it gives up and returns
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_1:\n"
            "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
            "rjmp L%=_2\n" // line is low. jump to below
            // the following happens if the line is still high
            "subi r24,lo8(1)\n"
            "brne L%=_1\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and jump to
            // the end
            "ldi %[retval],lo8(0)\n"
            "rjmp L%=_exit\n"
            "L%=_2:\n"

            // Next block. The line has just gone low. Wait approx 2µs
            // each cycle is 1/16 µs on a 16Mhz processor
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"

            // This block left shifts the current gc_status byte in r23,
            // and adds the current line state as the LSB
            "lsl r23\n" // left shift
            "sbic 0x9,2\n" // read PIND2
            "sbr r23,lo8(1)\n" // set bit 1 in r23 if PIND2 is high
            "st Z,r23\n" // save r23 back to memory. We technically only have
            // to do this every 8 bits but this simplifies the branches below

            // This block increments the bitcount(r25). If bitcount is 64, exit
            // with success. If bitcount is a multiple of 8, then increment Z
            // and load the next byte.
            "subi r25,lo8(-1)\n" // increment bitcount
            "cpi r25,lo8(64)\n" // == 64?
            "breq L%=_exit\n" // jump to exit
            "mov r24,r25\n" // copy bitcounter(r25) to r24 for tmp
            "andi r24,lo8(7)\n" // get lower 3 bits
            "brne L%=_3\n" // branch if not 0 (is not divisble by 8)
            "adiw r30,1\n" // if divisible by 8, increment pointer
            "ld r23,Z\n" // ...and load the new byte into r23
            "L%=_3:\n"

            // This next block waits for the line to go high again. again, it
            // sets a timeout counter of 64 iterations
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_4:\n"
            "sbic 0x9,2\n" // checks PIND2
            "rjmp L%=_read_loop\n" // line is high. ready for next loop
            // the following happens if the line is still low
            "subi r24,lo8(1)\n"
            "brne L%=_4\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and fall through to
            // the end
            "ldi %[retval],lo8(0)\n"


            "L%=_exit:\n"
            ";END OF MANUAL ASSEMBLY BLOCK\n"
            // ----------
            // outputs:
            : [retval] "=r" (retval),
            // About the bitbin pointer: The "z" constraint tells the
            // compiler to put the pointer in the Z register pair (r31:r30)
            // The + tells the compiler that we are both reading and writing
            // this pointer. This is important because otherwise it will
            // allocate the same register for retval (r30).
            "+z" (bitbin)
            // clobbers (registers we use in the assembly for the compiler to
            // avoid):
            :: "r25", "r24", "r23"
            );

    return retval;
}

static void print_gc_status()
{
    Serial.println();
    Serial.print("Start: ");
    Serial.println(gc_status.data1 & 0x10 ? 1:0);

    Serial.print("Y:     ");
    Serial.println(gc_status.data1 & 0x08 ? 1:0);

    Serial.print("X:     ");
    Serial.println(gc_status.data1 & 0x04 ? 1:0);

    Serial.print("B:     ");
    Serial.println(gc_status.data1 & 0x02 ? 1:0);

    Serial.print("A:     ");
    Serial.println(gc_status.data1 & 0x01 ? 1:0);

    Serial.print("L:     ");
    Serial.println(gc_status.data2 & 0x40 ? 1:0);
    Serial.print("R:     ");
    Serial.println(gc_status.data2 & 0x20 ? 1:0);
    Serial.print("Z:     ");
    Serial.println(gc_status.data2 & 0x10 ? 1:0);

    Serial.print("Dup:   ");
    Serial.println(gc_status.data2 & 0x08 ? 1:0);
    Serial.print("Ddown: ");
    Serial.println(gc_status.data2 & 0x04 ? 1:0);
    Serial.print("Dright:");
    Serial.println(gc_status.data2 & 0x02 ? 1:0);
    Serial.print("Dleft: ");
    Serial.println(gc_status.data2 & 0x01 ? 1:0);

    Serial.print("Stick X:");
    Serial.println(gc_status.stick_x, DEC);
    Serial.print("Stick Y:");
    Serial.println(gc_status.stick_y, DEC);

    Serial.print("cStick X:");
    Serial.println(gc_status.cstick_x, DEC);
    Serial.print("cStick Y:");
    Serial.println(gc_status.cstick_y, DEC);

    Serial.print("L:     ");
    Serial.println(gc_status.left, DEC);
    Serial.print("R:     ");
    Serial.println(gc_status.right, DEC);
    Serial.flush();
}

static bool rumble = false;
void loop()
{
    int status;
    unsigned char data, addr;

    if (gc_status.stick_x == zero_x && gc_status.stick_y == zero_y) {
    Serial.print("Raw stick: (");
    Serial.print(gc_status.stick_x, DEC);
    Serial.print(",");
    Serial.print(gc_status.stick_y, DEC);
    Serial.print(") After calb: (");
    
    // Calculate intermediate values
    char x_tmp = calb(gc_x_origin, gc_status.stick_x);
    char y_tmp = calb(gc_y_origin, gc_status.stick_y);
    Serial.print(x_tmp, DEC);
    Serial.print(",");
    Serial.print(y_tmp, DEC);
    Serial.print(") Final N64: (");
    
    // Get final mapped values
    char n64_x, n64_y;
    gamecubeXYtoN64(gc_status.stick_x, gc_status.stick_y, &n64_x, &n64_y);
    Serial.print(n64_x, DEC);
    Serial.print(",");
    Serial.print(n64_y, DEC);
    Serial.println(")");
}


    // clear out incomming raw data buffer
    // this should be unnecessary
    //memset(gc_raw_dump, 0, sizeof(gc_raw_dump));
    //memset(n64_raw_dump, 0, sizeof(n64_raw_dump));

    // Command to send to the gamecube
    // The last bit is rumble, flip it to rumble
    unsigned char command[] = {0x40, 0x03, 0x00};
    if (rumble) {
        command[2] = 0x01;
    }

    // turn on the led, so we can visually see things are happening
    digitalWrite(13, LOW);
    // don't want interrupts getting in the way
    noInterrupts();
    // send those 3 bytes
    gc_send(command, 3);
    // read in data and dump it to gc_raw_dump
    status = gc_get();
    // end of time sensitive code
    interrupts();
    digitalWrite(13, HIGH);

    if (status == 0) {
        // problem with getting the gamecube controller status. Maybe it's unplugged?
        // set a neutral N64 string
        Serial.print(millis(), DEC);
        Serial.println(" | GC controller read error. Trying to re-initialize");
        Serial.flush();
        memset(n64_buffer, 0, sizeof(n64_buffer));
        memset(&gc_status, 0, sizeof(gc_status));
        gc_status.stick_x = zero_x;
        gc_status.stick_y = zero_y;
        // this may not work if the controller isn't plugged in, but if it
        // fails we'll try again next loop
        noInterrupts();
        init_gc_controller();
        interrupts();
    } else {
        // translate the data to the n64 byte string
        mapGamecubeToN64();
    }

    // Wait for incomming 64 command
    // this will block until the N64 sends us a command
    noInterrupts();
    get_n64_command();

    // 0x00 is identify command
    // 0x01 is status
    // 0x02 is read
    // 0x03 is write
    switch (n64_command)
    {
        case 0x00:
        case 0xFF:
            // identify
            // mutilate the n64_buffer array with our status
            // we return 0x050001 to indicate we have a rumble pack
            // or 0x050002 to indicate the expansion slot is empty
            //
            // 0xFF I've seen sent from Mario 64 and Shadows of the Empire.
            // I don't know why it's different, but the controllers seem to
            // send a set of status bytes afterwards the same as 0x00, and
            // it won't work without it.
            n64_buffer[0] = 0x05;
            n64_buffer[1] = 0x00;
            n64_buffer[2] = 0x02;

            n64_send(n64_buffer, 3, 0);

            //Serial.println("It was 0x00: an identify command");
            break;
        case 0x01:
            // blast out the pre-assembled array in n64_buffer
            n64_send(n64_buffer, 4, 0);

            //Serial.println("It was 0x01: the query command");
            break;
        case 0x02:
            // A read. If the address is 0x8000, return 32 bytes of 0x80 bytes,
            // and a CRC byte.  this tells the system our attached controller
            // pack is a rumble pack

            // Assume it's a read for 0x8000, which is the only thing it should
            // be requesting anyways
            memset(n64_buffer, 0x80, 32);
            n64_buffer[32] = 0xB8; // CRC

            n64_send(n64_buffer, 33, 1);

            //Serial.println("It was 0x02: the read command");
            break;
        case 0x03:
            // A write. we at least need to respond with a single CRC byte.  If
            // the write was to address 0xC000 and the data was 0x01, turn on
            // rumble! All other write addresses are ignored. (but we still
            // need to return a CRC)

            // decode the first data byte (fourth overall byte), bits indexed
            // at 24 through 31
            data = 0;
            data |= (n64_raw_dump[16] != 0) << 7;
            data |= (n64_raw_dump[17] != 0) << 6;
            data |= (n64_raw_dump[18] != 0) << 5;
            data |= (n64_raw_dump[19] != 0) << 4;
            data |= (n64_raw_dump[20] != 0) << 3;
            data |= (n64_raw_dump[21] != 0) << 2;
            data |= (n64_raw_dump[22] != 0) << 1;
            data |= (n64_raw_dump[23] != 0);

            // get crc byte, invert it, as per the protocol for
            // having a memory card attached
            n64_buffer[0] = crc_repeating_table[data] ^ 0xFF;

            // send it
            n64_send(n64_buffer, 1, 1);

            // end of time critical code
            // was the address the rumble latch at 0xC000?
            // decode the first half of the address, bits
            // 8 through 15
            addr = 0;
            addr |= (n64_raw_dump[0] != 0) << 7;
            addr |= (n64_raw_dump[1] != 0) << 6;
            addr |= (n64_raw_dump[2] != 0) << 5;
            addr |= (n64_raw_dump[3] != 0) << 4;
            addr |= (n64_raw_dump[4] != 0) << 3;
            addr |= (n64_raw_dump[5] != 0) << 2;
            addr |= (n64_raw_dump[6] != 0) << 1;
            addr |= (n64_raw_dump[7] != 0);

            if (addr == 0xC0) {
                rumble = (data != 0);
            }

            //Serial.println("It was 0x03: the write command");
            //Serial.print("Addr was 0x");
            //Serial.print(addr, HEX);
            //Serial.print(" and data was 0x");
            //Serial.println(data, HEX);
            break;

        default:
            Serial.print(millis(), DEC);
            Serial.println(" | Unknown command received!!");
            break;

    }

    interrupts();

    // DEBUG: print it
    //print_gc_status();
    /*
    Serial.print(millis(), DEC);
    Serial.print(" | GC stick: ");
    Serial.print(gc_status.stick_x, DEC);
    Serial.print(",");
    Serial.print(gc_status.stick_y, DEC);
    Serial.print("  To N64: ");
    Serial.print(-zero_x + gc_status.stick_x, DEC);
    Serial.print(",");
    Serial.println(-zero_y + gc_status.stick_y, DEC);
    Serial.flush();
    */
  
}

/**
  * Waits for an incomming signal on the N64 pin and reads the command,
  * and if necessary, any trailing bytes.
  * 0x00 is an identify request
  * 0x01 is a status request
  * 0x02 is a controller pack read
  * 0x03 is a controller pack write
  *
  * for 0x02 and 0x03, additional data is passed in after the command byte,
  * which is also read by this function.
  *
  * All data is raw dumped to the n64_raw_dump array, 1 bit per byte, except
  * for the command byte, which is placed all packed into n64_command
  */
static void get_n64_command()
{
    int bitcount;
    char *bitbin = n64_raw_dump;
    int idle_wait;

    n64_command = 0;

    bitcount = 8;

    // wait to make sure the line is idle before
    // we begin listening
    for (idle_wait=32; idle_wait>0; --idle_wait) {
        if (!N64_QUERY) {
            idle_wait = 32;
        }
    }

read_loop:
        // wait for the line to go low
        while (N64_QUERY){}

        // wait approx 2us and poll the line
        asm volatile (
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                );
        if (N64_QUERY)
            n64_command |= 0x01;

        --bitcount;
        if (bitcount == 0)
            goto read_more;

        n64_command <<= 1;

        // wait for line to go high again
        // I don't want this to execute if the loop is exiting, so
        // I couldn't use a traditional for-loop
        while (!N64_QUERY) {}
        goto read_loop;

read_more:
        switch (n64_command)
        {
            case (0x03):
                // write command
                // we expect a 2 byte address and 32 bytes of data
                bitcount = 272 + 1; // 34 bytes * 8 bits per byte
                //Serial.println("command is 0x03, write");
                break;
            case (0x02):
                // read command 0x02
                // we expect a 2 byte address
                bitcount = 16 + 1;
                //Serial.println("command is 0x02, read");
                break;
            case (0x00):
            case (0x01):
            default:
                // get the last (stop) bit
                bitcount = 1;
                break;
        }

        // make sure the line is high. Hopefully we didn't already
        // miss the high-to-low transition
        while (!N64_QUERY) {}
read_loop2:
        // wait for the line to go low
        while (N64_QUERY){}

        // wait approx 2us and poll the line
        asm volatile (
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                      "nop\nnop\nnop\nnop\nnop\n"  
                );
        *bitbin = N64_QUERY;
        ++bitbin;
        --bitcount;
        if (bitcount == 0)
            return;

        // wait for line to go high again
        while (!N64_QUERY) {}
        goto read_loop2;
}
