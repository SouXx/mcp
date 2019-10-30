//
// Created by tobi on 24.10.19.
//

#include "main.h"
#include <stdio.h>
#include <tclDecls.h>
#include "../tiva/driverlib/interrupt.h"
#include "../tiva/driverlib/gpio.h"
#include "../tiva/driverlib/pin_map.h"
#include "../tiva/driverlib/rom.h"
#include "../tiva/driverlib/sysctl.c"
#include "../tiva/driverlib/uart.h"
#include "../tiva/inc/hw_memmap.h"


//#############################################################################
// Defines
//#############################################################################

#define OUTPUT_L            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
#define OUTPUT_M            GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
#define DISPLAY_WR          GPIO_PIN_1      // write state
#define DISPLAY_CS          GPIO_PIN_3      // chip select
#define DISPLAY_RS          GPIO_PIN_2      // mode select (command/data)
#define DISPLAY_RST         GPIO_PIN_4
#define SELECT_AND_WRITE    (DISPLAY_CS | DISPLAY_WR)
#define BACKGROUND_COLOR    0x00
#define FRAME_COLOR         0xFF

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
static struct frame_t get_frame(unsigned int start_x, unsigned int end_x, unsigned int start_y, unsigned int end_y) {
    struct frame_t frame;
    frame.start_x = start_x;
    frame.end_x = end_x;
    frame.start_y = start_y;
    frame.end_y = end_y;
    return frame;
}

void wait(void) {
    // TODO: refactor implementation
    volatile int tmp;
    for (tmp = 0; tmp < 10000; tmp++);
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
    write_data(color);
    write_data(color);
    write_data(color);
}

/**
 * draws a rectangle
 * @param delta_x
 * @param delta_y
 * @param color
 */
void draw_rectangle(unsigned int delta_x, unsigned int delta_y, unsigned char color) {

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
 * writes static frame
 */
void write_frame(void) {
    struct frame_t frame;
    frame.start_x = 0;
    frame.start_x = 479;
    frame.start_x = 68;
    frame.start_x = 70;

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y), FRAME_COLOR);

    frame.start_x = 400;
    frame.end_x = 402;
    frame.start_y = 70;
    frame.end_y = 271;

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y), FRAME_COLOR);
}

/**
 * clears display within the given frame
 * @param frame
 */
void clear_display(struct frame_t frame) {

    window_set(frame);
    draw_rectangle((frame.end_x - frame.start_x), (frame.end_y - frame.start_y), BACKGROUND_COLOR);

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
// main
//#############################################################################

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    //Port  Clock Gating Control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    //Set direction
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, OUTPUT_L);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, OUTPUT_M);
    initialise_ssd1963();

    struct frame_t whole_frame = get_frame(0, 479, 0, 271);
    struct frame_t tacho_frame = get_frame(0, 400, 70, 270);
    clear_display(whole_frame);
    write_frame();

    window_set(tacho_frame);
    draw_circle(200, 271, 200);
    draw_circle(200, 271, 199);

    while (1) {
        // IDLE
    }
}

