//
// Created by Qian Chen on 5/28/21.
//

#include "referee_UI_update_scheduler.h"
#include "shell.h"

int RefereeUISKD::label_count;
int RefereeUISKD::shape_count;

Referee::ext_client_custom_character_t RefereeUISKD::labels[REFEREE_UI_MAX_LABEL_COUNT];
Referee::graphic_data_struct_t RefereeUISKD::shapes[REFEREE_UI_MAX_SHAPE_COUNT];
RefereeUISKD::SKDThread RefereeUISKD::skdThread;

RefereeUISKD::component_state_t RefereeUISKD::shape_state[REFEREE_UI_MAX_SHAPE_COUNT];
RefereeUISKD::component_state_t RefereeUISKD::label_state[REFEREE_UI_MAX_LABEL_COUNT];

void RefereeUISKD::init(tprio_t SKDThreadPRIO) {
    skdThread.start(SKDThreadPRIO);
}

void RefereeUISKD::add_character(char* name, UI_point start_p, color_t color, uint32_t layer, uint32_t size,
                                 uint32_t weight, char* string) {
    if(label_count >= REFEREE_UI_MAX_LABEL_COUNT) {
        return;
    }
    int char_length = strlen(string);
    if(char_length > 30) {
        return;
    }
    Referee::ext_client_custom_character_t Charact{};
    Charact.grapic_data_struct.graphic_name[0] = (uint8_t) name[0];
    Charact.grapic_data_struct.graphic_name[1] = (uint8_t) name[1];
    Charact.grapic_data_struct.graphic_name[2] = (uint8_t) name[2];
    Charact.grapic_data_struct.operate_tpye =
    Charact.grapic_data_struct.graphic_tpye = LABEL_CHAR;
    Charact.grapic_data_struct.color = color;
    Charact.grapic_data_struct.layer = layer;
    Charact.grapic_data_struct.start_x = start_p.x;
    Charact.grapic_data_struct.start_y = start_p.y;
    Charact.grapic_data_struct.start_angle = size;
    Charact.grapic_data_struct.end_angle = char_length;
    Charact.grapic_data_struct.width = weight;
    for (int i = 0; i < char_length; i++) {
        Charact.data[i] = (uint8_t) string[i];
    }
    RefereeUISKD::labels[label_count] = Charact;
    RefereeUISKD::label_count += 1;
    chThdSleepMilliseconds(100);
}

void RefereeUISKD::add_circle(char *name, uint32_t layer, color_t color, UI_point center,
                              uint32_t radius, uint32_t line_width) {
    if(shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t circle{};
    circle.graphic_name[0] = name[0];
    circle.graphic_name[1] = name[1];
    circle.graphic_name[2] = name[2];
    circle.graphic_tpye = CIRCLE;
    circle.color = color;
    circle.layer = layer;
    circle.start_x = center.x;
    circle.start_y = center.y;
    circle.radius = radius;
    circle.width = line_width;
    RefereeUISKD::shapes[shape_count] = circle;
    shape_count += 1;
}

void RefereeUISKD::add_rect(char *name, uint32_t layer, color_t color, UI_point start_p, UI_point end_p,
                            uint32_t line_width) {
    if(shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t rect{};
    rect.graphic_name[0] = name[0];
    rect.graphic_name[1] = name[1];
    rect.graphic_name[2] = name[2];
    rect.graphic_tpye = RECT;
    rect.color = color;
    rect.layer = layer;
    rect.start_x = start_p.x;
    rect.start_y = start_p.y;
    rect.end_x = end_p.x;
    rect.end_y = end_p.y;
    rect.width = line_width;
    RefereeUISKD::shapes[shape_count] = rect;
    shape_count += 1;
}

void RefereeUISKD::add_line(char *name, uint32_t layer, color_t color, UI_point start_p, UI_point end_p,
                            uint32_t line_width) {
    if(shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t line{};
    line.graphic_name[0] = name[0];
    line.graphic_name[1] = name[1];
    line.graphic_name[2] = name[2];
    line.graphic_tpye = LINE;
    line.color = color;
    line.layer = layer;
    line.start_x = start_p.x;
    line.start_y = start_p.y;
    line.end_x = end_p.x;
    line.end_y = end_p.y;
    line.width = line_width;
    RefereeUISKD::shapes[shape_count] = line;
    shape_count += 1;
}

void RefereeUISKD::add_int(char *name, uint32_t layer, color_t color, UI_point start_p,
                           uint32_t font_size, int data) {
    if(shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t label_int{};
    label_int.graphic_name[0] = name[0];
    label_int.graphic_name[1] = name[1];
    label_int.graphic_name[2] = name[2];
    label_int.graphic_tpye = LABEL_INTEGER;
    label_int.layer = layer;
    label_int.color = color;
    label_int.start_x = start_p.x;
    label_int.start_y = start_p.y;
    label_int.start_angle = font_size;
    label_int.width = font_size / 10;
    label_int.start_angle = font_size;

    label_int.radius = data >> 22U;
    label_int.end_x = data >> 11U;
    label_int.end_y = data;
    shapes[shape_count] = label_int;
    shape_count += 1;
}

void RefereeUISKD::add_float(char *name, uint32_t layer, color_t color, UI_point start_p,
                             uint32_t font_size, float data) {
    if(shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t label_float{};
    label_float.graphic_name[0] = name[0];
    label_float.graphic_name[1] = name[1];
    label_float.graphic_name[2] = name[2];
    label_float.graphic_tpye = LABEL_INTEGER;
    label_float.layer = layer;
    label_float.color = color;
    label_float.start_x = start_p.x;
    label_float.start_y = start_p.y;
    label_float.start_angle = font_size;
    label_float.width = font_size / 10;
    label_float.start_angle = font_size;

    label_float.radius = (uint32_t)(data * 1000) >> 22U;
    label_float.end_x = (uint32_t)(data * 1000) >> 11U;
    label_float.end_y = data * 1000;
    label_float.end_angle = 2;
    shapes[shape_count] = label_float;
    shape_count += 1;
}

void RefereeUISKD::remove_all() {
    if(shape_count == 0) return;
    shape_count = 0;
    label_count = 0;
    Referee::remove_all();
}

void RefereeUISKD::remove_layer(uint32_t layer) {
    if(shape_count == 0) return;
    int step_offset = 0;
    for(int i = 0; i < shape_count; i++) {
        while(shapes[i+step_offset].layer == layer) {
            step_offset++;
            shape_count--;
        }
        if (i+step_offset < shape_count) shapes[i] = shapes[i+step_offset];
    }
    step_offset = 0;
    for(int i = 0; i < label_count; i++) {
        while(labels[i+step_offset].grapic_data_struct.layer != layer) {
            step_offset++;
            label_count--;
        }
        if(i+step_offset < label_count) labels[i] = labels[i+step_offset];
    }
    Referee::remove_layer(layer);
}

void RefereeUISKD::echo_shapes() {
    for (int i = 0; i<shape_count; i++) {
        char name[3] = {shapes[i].graphic_name[0], shapes[i].graphic_name[1], shapes[i].graphic_name[2]};
        Shell::printf("%s ", name);
    }
    Shell::printf(SHELL_NEWLINE_STR);
}

void RefereeUISKD::echo_titles() {
    for (int i = 0; i < label_count; i++) {
        for(int j = 0; j < labels[i].grapic_data_struct.end_angle; j++) {
            Shell::printf("%d ", labels[i].data[j]);
        }
        Shell::printf(SHELL_NEWLINE_STR);
    }
}

void RefereeUISKD::SKDThread::main() {
    setName("REF_UI_SKD");
    while (!shouldTerminate()) {
        // Send data from buffer.
        for (int i = 0; i < shape_count; i++) {
            if(shape_state[i] == FINISHED) {
                shapes[i].operate_tpye = 2;
                Referee::set_graphic(shapes[i]);
            }
            while(shape_state[i]!=FINISHED) {
                shape_state[i] = Referee::set_graphic(shapes[i])? FINISHED : shape_state[i];
            };
        }
        for (int i = 0; i < label_count; i++) {
            if(label_state[i]==FINISHED) {
                labels[i].grapic_data_struct.operate_tpye = 2;
                Referee::set_title(labels[i]);
            }
            while(label_state[i]!=FINISHED) {
                label_state[i] = Referee::set_title(labels[i]) ? FINISHED : label_state[i];
            }
        }
        sleep(TIME_MS2I(400));
    }
}

int RefereeUISKD::strlen(char *s)
{
    int i=0;
    while(*s!='\0') {
        i++;
        s++;
    }
    return i;
}