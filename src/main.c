#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "mpp1.h"

//#############################################################################
// Defines
//#############################################################################

#define OUTPUT_L            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
#define OUTPUT_M            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
#define INPUT_P                GPIO_PIN_0 | GPIO_PIN_1
#define DISPLAY_WR          GPIO_PIN_1      // write state
#define DISPLAY_CS          GPIO_PIN_3      // chip select
#define DISPLAY_RS          GPIO_PIN_2      // mode select (command/data)
#define DISPLAY_RST         GPIO_PIN_4
#define SELECT_AND_WRITE    (DISPLAY_CS | DISPLAY_WR)
#define BACKGROUND_COLOR    0x00
#define FRAME_COLOR         0xFF
#define OFFSET                5
#define CLOCK_FREQUENCY     25000000

// due to cycle dependency redefine
// hw_ints.h

#define FAULT_SYSTICK       15
#define INT_GPIOP0_TM4C123  123

#define SPEED_FACTOR        ((double) CLOCK_FREQUENCY * (3.6))

//#############################################################################
// GLOBAL / Typedef
//#############################################################################
static volatile int distance_meter = 0 + OFFSET;
static volatile int measure_call_cnt_velo = 0;
static volatile double velocity;

static struct semaphore_t {
    int counter;
} semaphore;

struct frame_t {
    unsigned int start_x;
    unsigned int end_x;
    unsigned int start_y;
    unsigned int end_y;
};

enum dir {
    FORWARD = 0, BACKWARD = 1
};

static volatile bool direction = FORWARD;

char K[5][5] = {{1, 0, 0, 0, 1},
                {1, 0, 0, 1, 0},
                {1, 0, 1, 0, 0},
                {1,
                    1, 0, 1, 0},
                {1, 0, 0, 0, 1}};

char M[5][5] = {{1, 0, 0, 0, 1},
                {1, 1, 0, 1, 1},
                {1, 0, 1, 0, 1},
                {1,
                    0, 0, 0, 1},
                {1, 0, 0, 0, 1}};

char V[5][5] = {{1, 0, 0, 0, 1},
                {1, 0, 0, 0, 1},
                {0, 1, 0, 1, 0},
                {0,
                    1, 0, 1, 0},
                {0, 0, 1, 0, 0}};

char R[5][5] = {{1, 1, 1, 0, 0},
                {1, 0, 0, 1, 0},
                {1, 1, 1, 0, 0},
                {1,
                    0, 0, 1, 0},
                {1, 0, 0, 0, 1}};

char DP[5][5] = {{0, 0, 0, 0, 0},
                 {0, 0, 1, 0, 0},
                 {0, 0, 0, 0, 0},
                 {0,
                     0, 1, 0, 0},
                 {0, 0, 0, 0, 0}};

char NUMBER[10][5][5] = {{{0, 1, 1, 0, 0}, {1, 0, 0, 1, 0}, {1, 0, 0, 1,
                                                                         0}, {1, 0, 0, 1, 0}, {0, 1, 1, 0, 0}},
                         {{0, 0, 1, 0, 0}, {0,
                                               1, 1, 0, 0}, {0, 0, 1, 0, 0}, {0, 0, 1, 0, 0}, {0, 1, 1, 1, 0}},
                         {{0, 1, 1, 0, 0}, {1, 0, 1, 0, 0}, {0, 0, 1, 0, 0}, {0, 1, 0, 1,
                                                                                          0}, {1, 1, 1, 0, 0}},
                         {{0, 1, 1, 0, 0},
                                           {1, 0, 0, 1, 0}, {0, 0, 1, 0, 0}, {1, 0, 0, 1, 0}, {0, 1,
                                                                                                     1, 0, 0}},
                         {{1, 0, 0, 0, 0}, {1, 0, 1, 0, 0}, {
                                                             1, 1, 1, 1, 0}, {0, 0, 1, 0, 0}, {0, 0, 1, 0, 0}},
                         {{1,
                              1, 1, 0, 0}, {1, 0, 0, 0, 0}, {1, 1, 0, 0, 0}, {0, 0, 1,
                                                                                       0, 0}, {1, 1, 0, 0, 0}},
                         {{0, 0, 1, 1, 0}, {0, 1, 0, 0,
                                                        0}, {1, 1, 1, 0, 0}, {1, 0, 0, 1, 0}, {1, 1, 1, 0, 0}},
                         {{1, 1, 1, 1, 0}, {0, 0, 0, 1, 0}, {0, 0, 1, 0, 0}, {0, 1, 0, 0,
                                                                                          0}, {1, 0, 0, 0, 0}},
                         {{0, 1, 1, 0, 0},
                                           {1, 0, 0, 1, 0}, {0, 1, 1, 0, 0}, {1, 0, 0, 1, 0}, {0, 1,
                                                                                                     1, 0, 0}},
                         {{0, 1, 1, 0, 0}, {1, 0, 0, 1, 0}, {
                                                             0, 1, 1, 1, 0}, {0, 0, 0, 1, 0}, {0, 1, 1, 0, 0}}};

//#############################################################################
// Helper functions
//#############################################################################
void init_semaphore(struct semaphore_t *semaphore, int v) {
    semaphore->counter = v;
}

void lock_semaphore(struct semaphore_t *semaphore) {
    semaphore->counter--;
    if (semaphore->counter < 0) {
        while (semaphore->counter <= 0) {}
    }
}

void unlock_semaphore(struct semaphore_t *semaphore) {
    semaphore->counter++;
}


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
    double radius = 200;
    double rad = 3.14 * (velocity / 400);
    double gk = sin(rad) * radius;
    double ak = abs(cos(rad) * radius);

    if ((int) velocity < (int) 200) {
        return get_frame(200, 200 - ak, 271, 271 - gk);
    } else {
        return get_frame(200, 200 + ak, 271, 271 - gk);
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

    lock_semaphore(&semaphore);
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
    unlock_semaphore(&semaphore);

}

/**
 * write data to display
 * @param data as hex value
 */
void write_data(unsigned char data) {
    lock_semaphore(&semaphore);
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
    unlock_semaphore(&semaphore);
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
void draw_pixel(unsigned int curr_x, unsigned int curr_y, unsigned char color) {

    struct frame_t frame;
    frame.end_y = curr_y;
    frame.start_y = curr_y;
    frame.start_x = curr_x;
    frame.end_x = curr_x;

    window_set(frame);
    write_command(0x2C);
    // draw RGB
    write_data(color);
    write_data(color);
    write_data(color);
}

void draw_line(struct frame_t frame) {
    int x0 = frame.start_x;
    int x1 = frame.end_x;
    int y0 = frame.start_y;
    int y1 = frame.end_y;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */

    while (1) {
        draw_pixel(x0, y0, FRAME_COLOR);
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
                    unsigned char color) {

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
    frame.start_y = 68;
    frame.end_y = 70;

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y),
                   FRAME_COLOR);

    frame.start_x = 400;
    frame.end_x = 402;
    frame.start_y = 70;
    frame.end_y = 271;

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

void s1_event_handler(void) {
    //IntMasterDisable();
    distance_meter++;
    measure_call_cnt_velo++;
    static volatile uint32_t local_velocity;

    // stop timer, get value, reset and start again
    TimerDisable(TIMER0_BASE, TIMER_A);
    static volatile uint32_t time_since_last_call = HWREG(TIMER0_BASE + TIMER_O_TAV);
    HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;
    TimerEnable(TIMER0_BASE, TIMER_A);
    local_velocity = (SPEED_FACTOR/ time_since_last_call);

    if (GPIOPinRead(GPIO_PORTP_BASE, GPIO_INT_PIN_1) == 2) {
        direction = FORWARD;
    } else {
        direction = BACKWARD;
    }

    // set global context
    velocity = local_velocity;

    GPIOIntClear(GPIO_PORTP_BASE, GPIO_INT_PIN_0);
    //IntMasterEnable();
}

void systick_handler(void) {

    toggle_gpio();
    volatile struct frame_t direction_frame = get_frame(403, 479, 71, 270);
    //update display
    static volatile bool curdirection;

    measure_call_cnt_velo = 0;
    if (curdirection != direction) {
        if (direction == BACKWARD) {
            clear_display(direction_frame);
            write_scaled_arr(8, 8, R, 410, 90);
        } else {
            clear_display(direction_frame);
            write_scaled_arr(8, 8, V, 410, 90);
        }

    }
    // draw analog speed
    draw_line(calculate_pointer(local_velocity));
/*
    volatile struct frame_t meter_frame = get_frame(80, 250, 0, 50);
    volatile uint32_t h_km = ((distance_meter / 100000) % 10);
    volatile uint32_t ten_km = ((distance_meter - h_km * 100000) / 10000) % 10;
    volatile uint32_t one_km = ((distance_meter - ten_km * 10000) / 1000) % 10;
    volatile uint32_t h_m = ((distance_meter - one_km * 1000) / 100) % 10;
    volatile uint32_t ten_m = ((distance_meter - h_m * 100) / 10) % 10;
    clear_display(meter_frame);
    write_scaled_arr(5, 6, &NUMBER[h_km], 94, 10);
    write_scaled_arr(5, 6, &NUMBER[ten_km], 124, 10);
    write_scaled_arr(5, 6, &NUMBER[one_km], 154, 10);
    write_scaled_arr(5, 6, &NUMBER[h_m], 184, 10);
    write_scaled_arr(5, 6, &NUMBER[ten_m], 210, 10);
*/

    curdirection = direction;
}
//#############################################################################
// main
//#############################################################################

int main(void) {


    int ticks_per_sec = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                            SYSCTL_OSC_MAIN |
                                            SYSCTL_USE_PLL |
                                            SYSCTL_CFG_VCO_480), CLOCK_FREQUENCY);
    // Init Global Static semaphore
    init_semaphore(&semaphore, 1);

    //Port  Clock Gating Control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    //Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // debug measure pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

    // Timer Init
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP); // 32bit period up mode
    HWREG(TIMER0_BASE + TIMER_O_TAV) = 0;
    TimerEnable(TIMER0_BASE, TIMER_A);

    //Set Direction
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, (OUTPUT_L | GPIO_PIN_5));
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, OUTPUT_M);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, INPUT_P);

    initialise_ssd1963();

    //Int Init
    GPIOIntRegister(GPIO_PORTP_BASE, s1_event_handler);
    GPIOIntTypeSet(GPIO_PORTP_BASE, GPIO_INT_PIN_0, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTP_BASE, GPIO_INT_PIN_0);

    struct frame_t whole_frame = get_frame(0, 479, 0, 271);
    struct frame_t tacho_frame = get_frame(0, 400, 70, 270);
    struct frame_t direction_frame = get_frame(403, 479, 71, 270);
    struct frame_t meter_frame = get_frame(80, 250, 0, 50);

    clear_display(whole_frame);
    write_frame();

    window_set(tacho_frame);
    draw_circle(200, 271, 200);
    draw_circle(200, 271, 199);

    write_scaled_arr(5, 6, K, 10, 10);
    write_scaled_arr(5, 6, M, 47, 10);
    write_scaled_arr(5, 6, DP, 64, 10);
    write_scaled_arr(8, 8, V, 410, 90);

    IntMasterEnable();

    //SysTick config
    SysTickIntEnable();
    SysTickEnable();
    SysTickIntRegister(systick_handler);
    SysTickPeriodSet((ticks_per_sec / 10)); // periodic call: 10/s

    IntPrioritySet(FAULT_SYSTICK, 5);
    IntPrioritySet(INT_GPIOP0_TM4C123, 4);

    while (1) {
        // IDLE
    }
}
