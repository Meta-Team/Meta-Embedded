//
// Created by Qian Chen on 5/28/21.
//

#include "referee_UI_update_scheduler.h"
#include "shell.h"

int RefereeUISKD::label_count = 0;
Referee::ext_client_custom_character_t RefereeUISKD::labels[REFEREE_UI_MAX_LABEL_COUNT];
RefereeUISKD::component_state_t RefereeUISKD::label_state[REFEREE_UI_MAX_LABEL_COUNT];
unsigned long RefereeUISKD::label_last_revision_time[REFEREE_UI_MAX_LABEL_COUNT];

int RefereeUISKD::shape_count = 0;
Referee::graphic_data_struct_t RefereeUISKD::shapes[REFEREE_UI_MAX_SHAPE_COUNT];
RefereeUISKD::component_state_t RefereeUISKD::shape_state[REFEREE_UI_MAX_SHAPE_COUNT];
unsigned long RefereeUISKD::shape_last_revision_time[REFEREE_UI_MAX_SHAPE_COUNT];

RefereeUISKD::SKDThread RefereeUISKD::skdThread;

MUTEX_DECL(RefereeUISKD::buffer_mutex);

bool RefereeUISKD::invoke_ui_delete_all = false;
bool RefereeUISKD::invoke_ui_delete_layer = false;
uint32_t RefereeUISKD::layer_deleting = 0;

void RefereeUISKD::init(tprio_t SKDThreadPRIO) {
    skdThread.start(SKDThreadPRIO);
}

bool RefereeUISKD::add_label(const char *name, ui_point_t start_p, color_t color, uint32_t layer, uint32_t size,
                             uint32_t weight, char *s) {
    if (label_count >= REFEREE_UI_MAX_LABEL_COUNT) {
        return false;
    }

    chMtxLock(&buffer_mutex);
    {
        int char_length = strlen(s);
        if (char_length > 30) {
            char_length = 30;
        }
        Referee::ext_client_custom_character_t &label = labels[label_count];
        label.grapic_data_struct.graphic_name[0] = (uint8_t) name[0];
        label.grapic_data_struct.graphic_name[1] = (uint8_t) name[1];
        label.grapic_data_struct.graphic_name[2] = (uint8_t) name[2];
        label.grapic_data_struct.operate_type = ADD_OP;
        label.grapic_data_struct.graphic_type = LABEL_CHAR;
        label.grapic_data_struct.color = color;
        label.grapic_data_struct.layer = layer;
        label.grapic_data_struct.start_x = start_p.x;
        label.grapic_data_struct.start_y = start_p.y;
        label.grapic_data_struct.start_angle = size;
        label.grapic_data_struct.end_angle = char_length;
        label.grapic_data_struct.width = weight;
        for (int i = 0; i < char_length; i++) {
            label.data[i] = (uint8_t) s[i];
        }
        label_state[label_count] = REVISING;
        label_count += 1;
    }
    chMtxUnlock(&buffer_mutex);

//     Shell::printf("interval: %d" SHELL_NEWLINE_STR, RefereeUISKD::label_count);
//     chThdSleepMilliseconds(200);
    return true;
}

bool RefereeUISKD::add_circle(const char *name, uint32_t layer, color_t color, ui_point_t center,
                              uint32_t radius, uint32_t line_width) {
    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return false;
    }

    chMtxLock(&buffer_mutex);
    {
        Referee::graphic_data_struct_t &circle = shapes[shape_count];
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
        shape_state[shape_count] = REVISING;
        shape_count += 1;
    }
    chMtxUnlock(&buffer_mutex);

    return true;
}

bool RefereeUISKD::add_rect(const char *name, uint32_t layer, color_t color, ui_point_t start_p, ui_point_t end_p,
                            uint32_t line_width) {
    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return false;
    }

    chMtxLock(&buffer_mutex);
    {
        Referee::graphic_data_struct_t rect = shapes[shape_count];
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
        shape_state[shape_count] = REVISING;
        shape_count += 1;
    }
    chMtxUnlock(&buffer_mutex);

    return true;
}

bool RefereeUISKD::add_line(const char *name, uint32_t layer, color_t color, ui_point_t start_p, ui_point_t end_p,
                            uint32_t line_width) {

    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return false;
    }

    chMtxLock(&buffer_mutex);
    {
        Referee::graphic_data_struct_t line = shapes[shape_count];
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
        shape_state[shape_count] = REVISING;
        shape_count += 1;
    }
    chMtxUnlock(&buffer_mutex);

    return true;
}

bool RefereeUISKD::add_int(const char *name, uint32_t layer, color_t color, ui_point_t start_p, uint32_t font_size,
                           int data) {

    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return false;
    }

    chMtxLock(&buffer_mutex);
    {
        Referee::graphic_data_struct_t label_int = shapes[shape_count];
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
        shape_state[shape_count] = REVISING;
        shape_count += 1;
    }
    chMtxUnlock(&buffer_mutex);

    return true;
}

bool RefereeUISKD::add_float(char *name, uint32_t layer, color_t color, ui_point_t start_p, uint32_t font_size,
                             float data) {

    if (shape_count >= REFEREE_UI_MAX_SHAPE_COUNT) {
        return false;
    }

    chMtxLock(&buffer_mutex);
    {
        Referee::graphic_data_struct_t &label_float = shapes[shape_count];
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
        shape_state[shape_count] = REVISING;
        shape_count += 1;
    }
    chMtxUnlock(&buffer_mutex);

    return true;
}

void RefereeUISKD::remove_all() {
    chMtxLock(&buffer_mutex);
    {
        shape_count = 0;
        label_count = 0;
        invoke_ui_delete_all = true;
    }
    chMtxUnlock(&buffer_mutex);
}

void RefereeUISKD::remove_layer(uint32_t layer) {
    chMtxLock(&buffer_mutex);
    {
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

        // TODO: we don't protect against overwriting layer_deleting before it is sent
        invoke_ui_delete_layer = true;
        layer_deleting = layer;
    }
    chMtxUnlock(&buffer_mutex);
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

bool RefereeUISKD::revise_label(const char *name, char *string, color_t color) {
    bool ret = false;

    chMtxLock(&buffer_mutex);
    {
        for (int i = 0; i < label_count; i++) {
            if (labels[i].grapic_data_struct.graphic_name[0] == (uint8_t) name[0] &&
                labels[i].grapic_data_struct.graphic_name[1] == (uint8_t) name[1] &&
                labels[i].grapic_data_struct.graphic_name[2] == (uint8_t) name[2]) { // find object

                // if (WITHIN_RECENT_TIME(label_last_revision_time[i], 1000)) return;
                int char_length = strlen(string);
                if (char_length > 30) {
                    char_length = 30;
                } // char length validation
                labels[i].grapic_data_struct.color = color;
                // revise the status in RefereeSKD
                // no need to revise the operation type in struct as it has been revised in SKDThread
                for (int j = 0; j < char_length; j++) {
                    labels[i].data[j] = (uint8_t) string[j];
                }
                label_state[i] = REVISING;
                label_last_revision_time[i] = SYSTIME;
                ret = true;
                break;
            }
        }
    }
    chMtxUnlock(&buffer_mutex);

    return ret;
}

bool RefereeUISKD::revise_shape_loc(const char *name, ui_point_t point, color_t color) {
    bool ret = false;

    chMtxLock(&buffer_mutex);
    {
        for (int i = 0; i < shape_count; i++) {
            if (shapes[i].graphic_name[0] == (uint8_t) name[0] &&
                shapes[i].graphic_name[1] == (uint8_t) name[1] &&
                shapes[i].graphic_name[2] == (uint8_t) name[2]) { // find object
                // if (WITHIN_RECENT_TIME(shape_last_revision_time[i], 100)) return;
                if (shapes[i].graphic_type == RECT) {
                    int width = (int) shapes[i].end_x - (int) shapes[i].start_x;
                    int height = (int) shapes[i].end_y - (int) shapes[i].start_y;
                    shapes[i].end_x = (uint32_t) (point.x + width);
                    shapes[i].end_y = (uint32_t) (point.y + height);
                }
                shapes[i].start_x = point.x;
                shapes[i].start_y = point.y;
                shapes[i].color = color;
                shape_state[i] = REVISING;
                shape_last_revision_time[i] = SYSTIME;
                ret = true;
                break;
            }
        }
    }
    chMtxUnlock(&buffer_mutex);

    return ret;
}

bool RefereeUISKD::revise_line(const char *name, ui_point_t start_p, ui_point_t end_p) {
    bool ret = false;

    chMtxLock(&buffer_mutex);
    {
        for (int i = 0; i < shape_count; i++) {
            if (shapes[i].graphic_name[0] == (uint8_t) name[0] &&
                shapes[i].graphic_name[1] == (uint8_t) name[1] &&
                shapes[i].graphic_name[2] == (uint8_t) name[2]) { // find object
                // if (WITHIN_RECENT_TIME(shape_last_revision_time[i], 100)) return;
                if (shapes[i].graphic_type != LINE) break;
                shapes[i].start_x = start_p.x;
                shapes[i].start_y = start_p.y;
                shapes[i].end_x = end_p.x;
                shapes[i].end_y = end_p.y;
                shape_state[i] = REVISING;
                shape_last_revision_time[i] = SYSTIME;
                ret = true;
                break;
            }
        }
    }
    chMtxUnlock(&buffer_mutex);

    return ret;
}

void RefereeUISKD::SKDThread::main() {
    setName("REF_UI_SKD");
    while (!shouldTerminate()) {
        unsigned sleep_time;

        chMtxLock(&buffer_mutex);
        {
            // Deleting should be sent first so that adding can be queued
            if (send_deleting_all_if_requested() ||
                send_deleting_layer_if_requested() ||
                send_label_update_if_requested() ||
                send_graphics_updates_if_requested()) {

                sleep_time = SEND_INTERVAL;
            } else {
                sleep_time = 10;
            }
        }
        chMtxUnlock(&buffer_mutex);

        sleep(TIME_MS2I(sleep_time));
    }
}

bool RefereeUISKD::send_deleting_all_if_requested() {
    // Called inside mutex
    if (invoke_ui_delete_all) {
        Referee::set_tx_data_for_deleting_all();
        Referee::send_tx_data();
        invoke_ui_delete_all = false;
        return true;
    } else {
        return false;
    }
}

bool RefereeUISKD::send_deleting_layer_if_requested() {
    // Called inside mutex
    if (invoke_ui_delete_layer) {
        Referee::set_tx_data_for_deleting_layer(layer_deleting);
        Referee::send_tx_data();
        invoke_ui_delete_layer = false;
        return true;
    } else {
        return false;
    }
}

bool RefereeUISKD::send_graphics_updates_if_requested() {
    // Called inside mutex
    bool has_data_to_send = false;
    for (int i = 0; i < shape_count; i++) {
        if (shape_state[i] != FINISHED) {
            if (Referee::add_tx_graphic(shapes[i])) {
                shape_state[i] = FINISHED;
                shapes[i].operate_type = MODIFY_OP;
                has_data_to_send = true;
            } else {  // buffer fulled
                break;
            }
        }
    }
    if (has_data_to_send) {
        Referee::set_tx_data_for_graphics();
        Referee::send_tx_data();
        return true;
    } else {
        return false;
    }
}

bool RefereeUISKD::send_label_update_if_requested() {
    // Called inside mutex
    for (int i = 0; i < label_count; i++) {
        if (label_state[i] != FINISHED) {
            Referee::set_tx_data_for_label(labels[i]);
            label_state[i] = FINISHED;
            labels[i].grapic_data_struct.operate_type = MODIFY_OP;
            Referee::send_tx_data();
            return true;
        }
    }
    return false;
}

int RefereeUISKD::strlen(char *s) {
    int i = 0;
    while (*s != '\0') {
        i++;
        s++;
    }
    return i;
}