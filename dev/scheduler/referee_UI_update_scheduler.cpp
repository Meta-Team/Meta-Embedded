//
// Created by Qian Chen on 5/28/21.
//

#include "referee_UI_update_scheduler.h"
#include "shell.h"

int RefereeUISKD::label_count = 0;
int RefereeUISKD::shape_count = 0;

Referee::ext_client_custom_character_t RefereeUISKD::labels[REFEREE_UI_MAX_LABEL_COUNT];
Referee::graphic_data_struct_t RefereeUISKD::shapes[REFEREE_UI_MAX_SHAPE_COUNT];
unsigned long RefereeUISKD::shape_last_revision_time[REFEREE_UI_MAX_SHAPE_COUNT];
RefereeUISKD::SKDThread RefereeUISKD::skdThread;

RefereeUISKD::component_state_t RefereeUISKD::shape_state[REFEREE_UI_MAX_SHAPE_COUNT];
RefereeUISKD::component_state_t RefereeUISKD::label_state[REFEREE_UI_MAX_LABEL_COUNT];
unsigned long RefereeUISKD::label_last_revision_time[REFEREE_UI_MAX_LABEL_COUNT];

void RefereeUISKD::init(tprio_t SKDThreadPRIO) {
    skdThread.start(SKDThreadPRIO);
}

void RefereeUISKD::add_character(char *name, UI_point start_p, color_t color, uint32_t layer, uint32_t size,
                                 uint32_t weight, char *string) {
    if (label_count >= REFEREE_UI_MAX_LABEL_COUNT) {
        return;
    }
    int char_length = strlen(string);
    if (char_length > 30) {
        return;
    }
    Referee::ext_client_custom_character_t Charact{};
    Charact.grapic_data_struct.graphic_name[0] = (uint8_t) name[0];
    Charact.grapic_data_struct.graphic_name[1] = (uint8_t) name[1];
    Charact.grapic_data_struct.graphic_name[2] = (uint8_t) name[2];
    Charact.grapic_data_struct.operate_type = ADD_OP;
    Charact.grapic_data_struct.graphic_type = LABEL_CHAR;
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
    Shell::printf("interval: %d" SHELL_NEWLINE_STR, RefereeUISKD::label_count);
    chThdSleepMilliseconds(100);
}

void RefereeUISKD::add_circle(char *name, uint32_t layer, color_t color, UI_point center,
                              uint32_t radius, uint32_t line_width) {
    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t circle{};
    circle.graphic_name[0] = name[0];
    circle.graphic_name[1] = name[1];
    circle.graphic_name[2] = name[2];

    circle.operate_type = ADD_OP;
    circle.graphic_type = CIRCLE;
    circle.layer = layer;

    circle.color = color;

    circle.start_x = center.x;
    circle.start_y = center.y;

    circle.radius = radius;
    circle.width = line_width;

    shapes[shape_count] = circle;
    shape_state[shape_count] = REVISING;
    shape_count += 1;
}

void RefereeUISKD::add_rect(char *name, uint32_t layer, color_t color, UI_point start_p, UI_point end_p,
                            uint32_t line_width) {
    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t rect{};
    rect.graphic_name[0] = name[0];
    rect.graphic_name[1] = name[1];
    rect.graphic_name[2] = name[2];

    rect.operate_type = ADD_OP;
    rect.graphic_type = RECT;
    rect.layer = layer;

    rect.color = color;

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
    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t line{};
    line.operate_type = ADD_OP;

    line.graphic_name[0] = name[0];
    line.graphic_name[1] = name[1];
    line.graphic_name[2] = name[2];
    line.graphic_type = LINE;
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
    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t label_int{};
    label_int.operate_type = ADD_OP;

    label_int.graphic_name[0] = name[0];
    label_int.graphic_name[1] = name[1];
    label_int.graphic_name[2] = name[2];
    label_int.graphic_type = LABEL_INTEGER;
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
    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return;
    }
    Referee::graphic_data_struct_t label_float{};
    label_float.operate_type = ADD_OP;

    label_float.graphic_name[0] = name[0];
    label_float.graphic_name[1] = name[1];
    label_float.graphic_name[2] = name[2];

    label_float.graphic_type = LABEL_INTEGER;
    label_float.layer = layer;
    label_float.color = color;
    label_float.start_x = start_p.x;
    label_float.start_y = start_p.y;
    label_float.start_angle = font_size;
    label_float.width = font_size / 10;
    label_float.start_angle = font_size;

    label_float.radius = (uint32_t) (data * 1000) >> 22U;
    label_float.end_x = (uint32_t) (data * 1000) >> 11U;
    label_float.end_y = data * 1000;
    label_float.end_angle = 2;
    shapes[shape_count] = label_float;
    shape_count += 1;
}

void RefereeUISKD::remove_all() {
    if (shape_count == 0) return;
    shape_count = 0;
    label_count = 0;
    Referee::remove_all();
}

void RefereeUISKD::remove_layer(uint32_t layer) {
    if (shape_count == 0) return;
    int skip_offset = 0;
    for (int i = 0; i < shape_count; i++) {
        while (shapes[i + skip_offset].layer == layer) {
            skip_offset++;
            shape_count--;
        }
        shapes[i] = shapes[i + skip_offset];
        shape_state[i] = shape_state[i + skip_offset];
    }

    skip_offset = 0;
    for (int i = 0; i < label_count; i++) {
        while (labels[i + skip_offset].grapic_data_struct.layer == layer) {
            skip_offset++;
            label_count--;
        }
        labels[i] = labels[i + skip_offset];
        label_state[i] = label_state[i + skip_offset];
    }

    Referee::remove_layer(layer);
}

void RefereeUISKD::remove_shape(char *name) {
    if (shape_count == 0) return;
    int step_offset = 0;
    for (int i = 0; i < shape_count; i++) {
        if (shapes[i].graphic_name[0] == (uint8_t) name[0] &&
            shapes[i].graphic_name[1] == (uint8_t) name[1] &&
            shapes[i].graphic_name[2] == (uint8_t) name[2]) {
            step_offset++;
            shape_count--;
            shapes[i].operate_type = DEL_OP;
            while (!Referee::set_graphic(shapes[i])) {
                chThdSleepMilliseconds(50);
            }
        }
        shapes[i] = shapes[i + step_offset];
        shape_state[i] = shape_state[i + step_offset];
    }
}

void RefereeUISKD::remove_chara(char *name) {
    if (label_count == 0) return;
    int step_offset = 0;
    for (int i = 0; i < label_count; i++) {
        if (labels[i].grapic_data_struct.graphic_name[0] == (uint8_t) name[0] &&
            labels[i].grapic_data_struct.graphic_name[1] == (uint8_t) name[1] &&
            labels[i].grapic_data_struct.graphic_name[2] == (uint8_t) name[2]) {
            labels[i].grapic_data_struct.operate_type = DEL_OP;
            step_offset++;
            label_count--;
            labels[i].grapic_data_struct.operate_type = DEL_OP;
            while (!Referee::set_title(labels[i])) {
                chThdSleepMilliseconds(50);
            }
        }
        labels[i] = labels[i + step_offset];
        label_state[i] = label_state[i + step_offset];
    }
}

void RefereeUISKD::echo_shapes() {
    for (int i = 0; i < shape_count; i++) {
        char name[3] = {shapes[i].graphic_name[0], shapes[i].graphic_name[1], shapes[i].graphic_name[2]};
        Shell::printf("%s " SHELL_NEWLINE_STR, name);
    }
}

void RefereeUISKD::echo_titles() {
    for (int i = 0; i < label_count; i++) {
        for (int j = 0; j < labels[i].grapic_data_struct.end_angle; j++) {
            Shell::printf("%d " SHELL_NEWLINE_STR, labels[i].data[j]);
        }

    }
}

void RefereeUISKD::revise_character(char *name, char *string, color_t color) {

    for (int i = 0; i < label_count; i++) {
        if (labels[i].grapic_data_struct.graphic_name[0] == (uint8_t) name[0] &&
            labels[i].grapic_data_struct.graphic_name[1] == (uint8_t) name[1] &&
            labels[i].grapic_data_struct.graphic_name[2] == (uint8_t) name[2]) { // find object
            if (WITHIN_RECENT_TIME(label_last_revision_time[i], 1000)) return;
            int char_length = strlen(string);
            if (char_length > 30) {
                return;
            } // char length validation
            labels[i].grapic_data_struct.color = color;
            // revise the status in RefereeSKD
            // no need to revise the operation type in struct as it has been revised in SKDThread
            for (int j = 0; j < char_length; j++) {
                labels[i].data[j] = (uint8_t) string[j];
            }
            label_state[i] = REVISING;
            label_last_revision_time[i] = SYSTIME;
            return;
        }
    }
    chThdSleepMilliseconds(100);
}

void RefereeUISKD::revise_shape_loc(char *name, UI_point point, color_t color) {
    for (int i = 0; i < shape_count; i++) {
        if (shapes[i].graphic_name[0] == (uint8_t) name[0] &&
            shapes[i].graphic_name[1] == (uint8_t) name[1] &&
            shapes[i].graphic_name[2] == (uint8_t) name[2]) { // find object
            if (WITHIN_RECENT_TIME(shape_last_revision_time[i], 100)) return;
            if (shapes[i].graphic_type == RECT) {
                int width = 0;
                int height = 0;
                width = (int) shapes[i].end_x - (int) shapes[i].start_x;
                height = (int) shapes[i].end_y - (int) shapes[i].start_y;
                shapes[i].end_x = (uint32_t) (point.x + width);
                shapes[i].end_y = (uint32_t) (point.y + height);
            }
            shapes[i].start_x = point.x;
            shapes[i].start_y = point.y;
            shapes[i].color = color;
            shape_state[i] = REVISING;
            shape_last_revision_time[i] = SYSTIME;
        }
    }
}

void RefereeUISKD::revise_line(char *name, UI_point start_p, UI_point end_p) {
    for (int i = 0; i < shape_count; i++) {
        if (shapes[i].graphic_name[0] == (uint8_t) name[0] &&
            shapes[i].graphic_name[1] == (uint8_t) name[1] &&
            shapes[i].graphic_name[2] == (uint8_t) name[2]) { // find object
            if (WITHIN_RECENT_TIME(shape_last_revision_time[i], 100)) return;
            if (shapes[i].graphic_type != LINE) return;
            shapes[i].start_x = start_p.x;
            shapes[i].start_y = start_p.y;
            shapes[i].end_x = end_p.x;
            shapes[i].end_y = end_p.y;
            shape_state[i] = REVISING;
            shape_last_revision_time[i] = SYSTIME;
        }
    }
}

void RefereeUISKD::SKDThread::main() {
    setName("REF_UI_SKD");
    while (!shouldTerminate()) {
        // Send data from buffer.
        for (int i = 0; i < shape_count; i++) {
            if (shape_state[i] == FINISHED) {
                shapes[i].operate_type = MODIFY_OP;
            }
            while (shape_state[i] != FINISHED) {
                shape_state[i] = Referee::set_graphic(shapes[i]) ? FINISHED : shape_state[i];
                sleep(TIME_MS2I(50));
            }
        }
        for (int i = 0; i < label_count; i++) {
            if (label_state[i] == FINISHED) {
                labels[i].grapic_data_struct.operate_type = MODIFY_OP;
            }
            while (label_state[i] != FINISHED) {
                label_state[i] = Referee::set_title(labels[i]) ? FINISHED : label_state[i];
                sleep(TIME_MS2I(100));
            }
        }
        sleep(TIME_MS2I(200));
    }
}

int RefereeUISKD::strlen(char *s) {
    int i = 0;
    while (*s != '\0') {
        i++;
        s++;
    }
    return i;
}