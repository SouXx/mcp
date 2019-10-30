//
// Created by tobi on 24.10.19.
//

#ifndef MCP_PRAKTIKUM_MAIN_H
#define MCP_PRAKTIKUM_MAIN_H


struct frame_t {
    unsigned int start_x;
    unsigned int end_x;
    unsigned int start_y;
    unsigned int end_y;
};


void clear_display(struct frame_t frame);

void write_frame(void);

void draw_circle(int x0, int y0, int radius);

void draw_rectangle(unsigned int delta_x, unsigned int delta_y, unsigned char color);

void draw_pixel(unsigned int curr_x, unsigned int curr_y, unsigned char color);

void window_set(struct frame_t frame);

void initialise_ssd1963(void);

void write_data(unsigned char data);

void write_command(unsigned char command);

void wait(void);


#endif //MCP_PRAKTIKUM_MAIN_H
