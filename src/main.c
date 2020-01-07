#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "symbols.h"
#include "mpp1.h"
#include "driverlib/timer.h"
#include "inc/hw_timer.h"

//#############################################################################
// Defines
//#############################################################################

#define OUTPUT_L            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
#define OUTPUT_M            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
#define INPUT_P             GPIO_PIN_0 | GPIO_PIN_1
#define DISPLAY_WR          GPIO_PIN_1      // write state
#define DISPLAY_CS          GPIO_PIN_3      // chip select
#define DISPLAY_RS          GPIO_PIN_2      // mode select (command/data)
#define DISPLAY_RST         GPIO_PIN_4
#define SELECT_AND_WRITE    (DISPLAY_CS | DISPLAY_WR)
#define BACKGROUND_COLOR    0x000000
#define FRAME_COLOR         0xFFFFFF
#define OFFSET              5
#define CLOCK_FREQUENCY     25000000

#define FAULT_SYSTICK       15
#define INT_GPIOP0_TM4C123  123
#define INT_TIMER1A_TM4C123 37          // 16/32-Bit Timer 1A

#define SPEED_FACTOR        ((double) CLOCK_FREQUENCY * (3.6) - 2500000.0)

//#############################################################################
// GLOBAL / Typedefs
//#############################################################################

static volatile int distance_meter = 0 + OFFSET;
static volatile int measure_call_cnt_velo = 0;
static volatile double velocity = 0;
static volatile double old_velocity = 0;
static volatile struct frame_t meter_frame;
static volatile struct frame_t whole_frame;
static volatile struct frame_t direction_frame;
static volatile struct frame_t meter_frame;

static volatile uint32_t h_km;
static volatile uint32_t ten_km;
static volatile uint32_t one_km;
static volatile uint32_t h_m;
static volatile uint32_t ten_m;
static volatile int co_mass;
static volatile int co_mass_old;

static volatile uint32_t h_km_old = 0;
static volatile uint32_t ten_km_old = 0;
static volatile uint32_t one_km_old = 0;
static volatile uint32_t h_m_old = 0;
static volatile uint32_t ten_m_old = 0;

static volatile uint32_t avr_velocity;
static volatile uint8_t check = 0;
static volatile uint8_t lock = 0;

static volatile uint32_t old_time_since_last_call = 0;

struct frame_t {
    unsigned int start_x;
    unsigned int end_x;
    unsigned int start_y;
    unsigned int end_y;
    unsigned int bg_color;
};

enum dir {
    FORWARD = 0, BACKWARD = 1
};

static volatile bool direction = FORWARD;

//#############################################################################
// Helper functions
//#############################################################################

/**
 * helper class, returns full initilized struct
 * @param start_x
 * @param end_x
 * @param start_y
 * @param end_y
 * @return struct frame_t
 */
static struct frame_t get_frame(unsigned int start_x, unsigned int end_x,
                                unsigned int start_y, unsigned int end_y) {

    struct frame_t frame;
    frame.start_x = start_x;
    frame.end_x = end_x;
    frame.start_y = start_y;
    frame.end_y = end_y;
    return frame;
}

void toggle_gpio() {
    static bool tmp;

    if (tmp) {
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0xFF);
    } else {
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0x00);
    }
    tmp = !tmp;
}

void wait(void) {

    // TODO: refactor implementation
    volatile int tmp;
    for (tmp = 0; tmp < 10000; tmp++) {
    };
}

struct frame_t calculate_pointer(int velocity) {
    double radius = 197.0;
    double rad = 3.14 * (velocity / 400.0);
    double gk = sin(rad) * radius;
    double ak = abs(cos(rad) * radius);

    if ((int) velocity < (int) 200) {
        return get_frame(240, 240 - ak, 271, 271 - gk);
    } else {
        return get_frame(240, 240 + ak, 271, 271 - gk);
    }
}

//#############################################################################
// Display utilities
//#############################################################################

/**
 * writes command to display
 * @param command as hex value
 */
void write_command(unsigned char command) {

    //Ausgang von gesamt Port L wird auf 0x1F gesetzt,
    GPIOPinWrite(GPIO_PORTL_BASE, OUTPUT_L, 0x1F);

    // Cs = 0  --> Chip select signal
    // DISPLAY_RS = 0  --> Command mode
    // DISPLAY_WR = 0  --> write enable
    GPIOPinWrite(GPIO_PORTL_BASE, (SELECT_AND_WRITE | DISPLAY_RS), 0x00);

    //Port M Ausgabe von Command (var)
    GPIOPinWrite(GPIO_PORTM_BASE, OUTPUT_M, command);

    // DISPLAY_WR = 1  --> write disable
    // DISPLAY_CS = 1  --> no Chip select signal
    GPIOPinWrite(GPIO_PORTL_BASE, SELECT_AND_WRITE, 0xFF); // 0xFF represents logical 1 on all pins

}

/**
 * write data to display
 * @param data as hex value
 */
void write_data(unsigned char data) {
//    lock_semaphore(&semaphore);
    //Ausgang von gesamt Port L wird auf 0x1F gesetzt,
    GPIOPinWrite(GPIO_PORTL_BASE, OUTPUT_L, 0x1F);

    // Cs = 0  --> Chip select signal
    // DISPLAY_RS = 1  --> Command mode
    // DISPLAY_WR = 0  --> write enable
    GPIOPinWrite(GPIO_PORTL_BASE, SELECT_AND_WRITE, 0x00);
    GPIOPinWrite(GPIO_PORTL_BASE, DISPLAY_RS, 0xFF);

    //Port M Ausgabe von data (var)
    GPIOPinWrite(GPIO_PORTM_BASE, OUTPUT_M, data);

    // DISPLAY_WR = 1  --> write disable
    // DISPLAY_CS = 1  --> no Chip select signal
    GPIOPinWrite(GPIO_PORTL_BASE, SELECT_AND_WRITE, 0xFF);
//    unlock_semaphore(&semaphore);
}

/**
 * Initialise
 */
void initialise_ssd1963(void) {
    GPIOPinWrite(GPIO_PORTL_BASE, DISPLAY_RST, 0x00);
    wait();
    write_command(0x01);
    wait();
    write_command(0xE2);
    write_data(0x1D);
    write_data(0x02);
    write_data(0x04);
    write_command(0xE0);
    write_data(0x01);
    wait();
    write_command(0xE0);
    write_data(0x03);
    wait();
    write_command(0x01);
    wait();
    write_command(0xE6);
    write_data(0x01);
    write_data(0x70);
    write_data(0xA3);

    write_command(0xB0);
    write_data(0x20);
    write_data(0x00);
    write_data(0x01);
    write_data(0xDF);
    write_data(0x01);
    write_data(0x0F);
    write_data(0x00);

    write_command(0xB4);
    write_data(0x02);
    write_data(0x13);
    write_data(0x00);
    write_data(0x2B);
    write_data(0x0A);
    write_data(0x00);
    write_data(0x08);
    write_data(0x00);

    write_command(0xB6);
    write_data(0x01);
    write_data(0x20);
    write_data(0x00);
    write_data(0x0C);
    write_data(0x0A);
    write_data(0x00);
    write_data(0x04);

    write_command(0x36);
    write_data(0x03);

    write_command(0xF0);
    write_data(0x00);

    write_command(0x29);
}

/**
 * clears display within the given frame
 * @param frame
 */
void clear_display(struct frame_t frame) {

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y),
                   BACKGROUND_COLOR);

}

//#############################################################################
// Drawing functions
//#############################################################################

/**
 * set window to draw in
 * need to be set before
 * drawing something
 * @param frame size of window
 */
void window_set(struct frame_t frame) {

    write_command(0x2A);
    write_data((frame.start_x >> 8));
    write_data(frame.start_x);
    write_data((frame.end_x >> 8));
    write_data(frame.end_x);

    write_command(0x2B);
    write_data((frame.start_y >> 8));
    write_data(frame.start_y);
    write_data((frame.end_y >> 8));
    write_data(frame.end_y);
}

/**
 * draws a single pixel
 * @param curr_x x coordinate
 * @param curr_y y coordinate
 * @param color of the pixel
 */
void draw_pixel(unsigned int curr_x, unsigned int curr_y, unsigned int color) {

    struct frame_t frame;
    frame.end_y = curr_y;
    frame.start_y = curr_y;
    frame.start_x = curr_x;
    frame.end_x = curr_x;

    window_set(frame);
    write_command(0x2C);
    // draw RGB
    write_data((color&0xff0000)>>16);//rot
    write_data((color&0x00ff00)>>8); //gr�n
    write_data((color&0x0000ff)); //blau
}

/**
 * refresh line
 * @param frame
 * @param old_frame
 */
void refresh_line(struct frame_t frame, struct frame_t old_frame) {
    frame.bg_color = FRAME_COLOR;
    old_frame.bg_color = BACKGROUND_COLOR;

    draw_line(old_frame);
    draw_line(frame);

}

/**
 * draws line
 * @param frame
 */
void draw_line(struct frame_t frame) {
    int x0 = frame.start_x;
    int x1 = frame.end_x;
    int y0 = frame.start_y;
    int y1 = frame.end_y;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */

    while (1) {
        draw_pixel(x0, y0, frame.bg_color);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2 * err;
        if (e2 > dy) {
            err += dy;
            x0 += sx;
        } /* e_xy+e_x > 0 */
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        } /* e_xy+e_y < 0 */
    }
}

/**
 * draws a rectangle
 * @param delta_x
 * @param delta_y
 * @param color
 */
void draw_rectangle(unsigned int delta_x, unsigned int delta_y,
                    unsigned int color) {

    write_command(0x2C);
    unsigned int y = 0;
    unsigned int x = 0;
    for (y = 0; y <= delta_y; y++) {
        for (x = 0; x <= delta_x; x++) {
            write_data(color);
            write_data(color);
            write_data(color);
        }
    }
}

/**
 * write array to display
 * @param symbol_arr
 * @param x_start
 * @param y_start
 */
void write_array(char symbol_arr[30][30], int x_start, int y_start, int color) {
    int x;
    int y;

    for (y = 0; y <= 30; ++y) {
        for (x = 0; x <= 30; ++x) {
            if (symbol_arr[y][x] == 1) {
                draw_pixel(x_start + x, y_start + y, color);
            }
        }
    }
}

void write_array_three_signs(char symbol_arr[15][32], int x_start, int y_start, int color) {
    int x;
    int y;

    for (y = 0; y <= 15; ++y) {
        for (x = 0; x <= 32; ++x) {
            if (symbol_arr[y][x] == 1) {
                draw_pixel(x_start + x, y_start + y, color);
            }
        }
    }
}
/**
 *
 * @param h_scale factor
 * @param v_scale factor
 * @param symbol_arr array to scale
 * @param x_start pixel x direction
 * @param y_start pixel y direction
 */
void write_scaled_arr(int h_scale, int v_scale, char symbol_arr[5][5],
                      int x_start, int y_start) {
    int x;
    int y;
    int xx;
    int yy;

    for (y = 0; y <= 4; ++y) {
        for (x = 0; x <= 4; ++x) {
            if (symbol_arr[y][x] == 1) {
                for (xx = 0; xx < h_scale; ++xx) {
                    for (yy = 0; yy < v_scale; ++yy) {
                        draw_pixel(x_start + x * h_scale + xx,
                                   y_start + y * v_scale + yy, FRAME_COLOR);
                    }
                }
            }
        }
    }

}

/**
 * writes static frame
 */
void write_frame(void) {

    struct frame_t frame;
    frame.start_x = 0;
    frame.end_x = 479;
    frame.start_y = 48;
    frame.end_y = 50;

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y),
                   FRAME_COLOR);

    frame.start_x = 415;
    frame.end_x = 417;
    frame.start_y = 0;
    frame.end_y = 50;

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y),
                   FRAME_COLOR);

    frame.start_x = 260;
    frame.end_x = 262;
    frame.start_y = 0;
    frame.end_y = 50;

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y),
                   FRAME_COLOR);

    frame.start_x = 40;
    frame.end_x = 440;
    frame.start_y = 269;
    frame.end_y = 272;

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y),
                   FRAME_COLOR);
}

/**
 * Bresenham's cricle draw algorithm
 * @param x0 circle center
 * @param y0 circle center
 * @param radius
 */
void draw_circle(int x0, int y0, int radius) {

    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    draw_pixel(x0, y0 + radius, FRAME_COLOR);
    draw_pixel(x0, y0 - radius, FRAME_COLOR);
    draw_pixel(x0 + radius, y0, FRAME_COLOR);
    draw_pixel(x0 - radius, y0, FRAME_COLOR);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x + 1;

        draw_pixel(x0 - x, y0 + y, FRAME_COLOR);
        draw_pixel(x0 - x, y0 + y, FRAME_COLOR);
        draw_pixel(x0 + x, y0 - y, FRAME_COLOR);
        draw_pixel(x0 - x, y0 - y, FRAME_COLOR);
        draw_pixel(x0 + y, y0 + x, FRAME_COLOR);
        draw_pixel(x0 - y, y0 + x, FRAME_COLOR);
        draw_pixel(x0 + y, y0 - x, FRAME_COLOR);
        draw_pixel(x0 - y, y0 - x, FRAME_COLOR);
    }
}

//#############################################################################
// Interrupt handler
//#############################################################################
/**
 * signal handler
 * task: measuring
 */
void s1_event_handler(void) {

    check = 1;
    distance_meter++;

    // Read TimerValue
    TimerDisable(TIMER0_BASE, TIMER_A);
    volatile uint32_t time_since_last_call = HWREG(TIMER0_BASE + TIMER_O_TAV);
    HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;
    TimerEnable(TIMER0_BASE, TIMER_A);
    if (GPIOPinRead(GPIO_PORTP_BASE, GPIO_INT_PIN_1) == 2) {
        direction = FORWARD;
    } else {
        direction = BACKWARD;
    }
    if (time_since_last_call > (old_time_since_last_call* 0.9) && time_since_last_call < (old_time_since_last_call* 1.1))
        velocity = ((SPEED_FACTOR) / time_since_last_call);

    old_time_since_last_call = time_since_last_call;
    lock = 1;
    GPIOIntClear(GPIO_PORTP_BASE, GPIO_INT_PIN_0);
}
/**
 * calculate measurements
 */
void systick_handler(void) {

    // kilometer
    h_km = ((distance_meter / 100000) % 10);
    ten_km = ((distance_meter - h_km * 100000) / 10000) % 10;
    one_km = ((distance_meter - ten_km * 10000) / 1000) % 10;
    h_m = ((distance_meter - one_km * 1000) / 100) % 10;
    ten_m = ((distance_meter - h_m * 100) / 10) % 10;

    if (distance_meter > 10000) {
        co_mass = (distance_meter / 1000) * 0.15;
    }

}

/**
 * watchdog, catches velocity at zero
 * task: bring needle down to zero
 */
void timer1_watchdog_handler(void) {

    check = 0;
    if (lock == 1) {
        uint32_t velo = old_velocity;
        for (velo; velo > 2; velo--) {
            refresh_line(calculate_pointer(velo), calculate_pointer(velo + 1));
        }
        old_velocity = 0;
    }
    lock = 0;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/**
 * responsible for output
 */
void timer1_draw_handler(void) {

    IntMasterDisable();
    static volatile bool curdirection;
    uint32_t distance_meter_old;

    // mileage output
    if ((distance_meter / 10) % 10 != (distance_meter_old / 10) % 10) {
        // TODO: CO Output
        //clear_display(meter_frame);

        write_array(&numbers_symbols[h_km_old], 94, 10, BACKGROUND_COLOR);
        write_array(&numbers_symbols[ten_km_old], 124, 10, BACKGROUND_COLOR);
        write_array(&numbers_symbols[one_km_old], 154, 10, BACKGROUND_COLOR);

        write_array(&numbers_symbols[h_m_old], 190, 10, BACKGROUND_COLOR);
        write_array(&numbers_symbols[ten_m_old], 220, 10, BACKGROUND_COLOR);
        write_array(&numbers_symbols[co_mass_old], 325, 10, BACKGROUND_COLOR);

        write_array(&numbers_symbols[h_km], 94, 10, FRAME_COLOR);
        write_array(&numbers_symbols[ten_km], 124, 10, FRAME_COLOR);
        write_array(&numbers_symbols[one_km], 154, 10, FRAME_COLOR);

        write_array(&numbers_symbols[h_m], 190, 10, FRAME_COLOR);
        write_array(&numbers_symbols[ten_m], 220, 10, FRAME_COLOR);

        write_array(&numbers_symbols[co_mass], 325, 10, 0xFF0000);

        h_km_old = h_km;
        ten_km_old = ten_km;
        one_km_old = one_km;
        h_m_old = h_m;
        ten_m_old = ten_m;
        co_mass_old = co_mass;

        distance_meter_old = distance_meter;
    }

    if (check == 1) {
        // draw analog speed
        // check if speed is plausible
        struct frame_t tmp = calculate_pointer(old_velocity);
        tmp.bg_color = BACKGROUND_COLOR;
        draw_line(tmp);
        if (velocity < (old_velocity + 10) && velocity > (old_velocity - 10)) {
            if(velocity > 395){
                velocity = 395;
            }
            refresh_line(calculate_pointer(velocity),
                         calculate_pointer(old_velocity));
        }

    }
    old_velocity = velocity;

    if (curdirection != direction) {
        if (direction == BACKWARD) {
            clear_display(direction_frame);
            write_array(r_symbol, 430, 10, FRAME_COLOR);
        } else {
            clear_display(direction_frame);
            write_array(v_symbol, 430, 10, FRAME_COLOR);
        }
        curdirection = direction;
    }
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
}

//#############################################################################
// main
//#############################################################################

int main(void) {

    int ticks_per_sec = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                            SYSCTL_OSC_MAIN |
                                            SYSCTL_USE_PLL |
                                            SYSCTL_CFG_VCO_480), CLOCK_FREQUENCY);

//Init global values
    whole_frame = get_frame(0, 479, 0, 271);
    meter_frame = get_frame(80, 250, 0, 45);
    direction_frame = get_frame(430, 479, 0, 45);

//Port Clock Gating Control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
//Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

//Timer0
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP); // 32bit period up mode
    HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;
    HWREG(TIMER0_BASE + TIMER_O_TAILR) = CLOCK_FREQUENCY / 4; // 0.25sec
    TimerIntRegister(TIMER0_BASE, TIMER_A, timer1_watchdog_handler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

//Timer1
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // 32bit periodic down mode
    HWREG(TIMER1_BASE + TIMER_O_TAILR) = 5000000;
    TimerIntRegister(TIMER1_BASE, TIMER_A, timer1_draw_handler);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

//Set Direction
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, (OUTPUT_L | GPIO_PIN_5));
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, OUTPUT_M);
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, INPUT_P);

    initialise_ssd1963();

//Int Init
    GPIOIntRegister(GPIO_PORTP_BASE, s1_event_handler);
    GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_INT_PIN_0, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTP_BASE, GPIO_INT_PIN_0);

    clear_display(whole_frame);
    write_frame();

    //   window_set(tacho_frame);

    draw_circle(240, 271, 200);
    draw_circle(240, 271, 199);
    draw_circle(240, 271, 198);
//write_array(komma_symbol, 180, 10, FRAME_COLOR);
    write_scaled_arr(1,1,kommatar,184,38);

    write_array(k_symbol, 20, 10, FRAME_COLOR);
    write_array(m_symbol, 45, 10, FRAME_COLOR);
    write_array(dp_symbol, 65, 10, FRAME_COLOR);
    write_array(v_symbol, 430, 10, FRAME_COLOR);

    write_array(c_symbol, 265, 10, FRAME_COLOR);
    write_array(o_symbol, 290, 10, FRAME_COLOR);
    write_array(dp_symbol, 305, 10, FRAME_COLOR);

    write_array(k_symbol, 350, 10, FRAME_COLOR);
    write_array(g_symbol, 375, 10, FRAME_COLOR);

    write_array_three_signs(null_tacho,8,256,FRAME_COLOR);
    write_array_three_signs(einhundert,65,120,FRAME_COLOR);
    write_array_three_signs(zweihundert,224,56,FRAME_COLOR);
    write_array_three_signs(dreihundert,383,120,FRAME_COLOR);
    write_array_three_signs(vierhundert,442,256,0xFF0000);

    IntMasterEnable();

//SysTick Config
    SysTickIntEnable();
    SysTickEnable();
    SysTickIntRegister(systick_handler);
    SysTickPeriodSet((ticks_per_sec / 10)); // periodic call: 10/s
//Set Priorities
    IntPrioritySet(FAULT_SYSTICK, 6);
    IntPrioritySet(INT_GPIOP0_TM4C123, 4);
    IntPrioritySet(INT_TIMER1A_TM4C123, 5);
//Start Timer
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);

    while (1) {
        // IDLE
    }
}
