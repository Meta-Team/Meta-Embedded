//
// Created by Qian Chen on 5/28/21.
//

#ifndef META_INFANTRY_REFEREE_UI_UPDATE_SCHEDULER_H
#define META_INFANTRY_REFEREE_UI_UPDATE_SCHEDULER_H

#include "referee_interface.h"

#define REFEREE_UI_MAX_SHAPE_COUNT 30
#define REFEREE_UI_MAX_LABEL_COUNT 10

class RefereeUISKD {
public:
    enum shape_t {
        LINE,
        RECT,
        CIRCLE,
        ELLIPSE,
        ARC,
        LABEL_FLOAT,
        LABEL_INTEGER,
        LABEL_CHAR,
        NONE
    };

    enum operate_type_t {
        NULL_OP,
        ADD_OP,
        MODIFY_OP,
        DEL_OP
    };

    enum color_t {
        ACCENT_COLOR,
        YELLOW,
        GREEN,
        ORANGE,
        PURPLE,
        PINK,
        CYAN,
        BLACK,
        WHITE
    };

    struct ui_point_t {
        uint32_t x;
        uint32_t y;
    };

    static void init(tprio_t SKDThreadPRIO);

    static bool add_label(const char* name, ui_point_t start_p, color_t color, uint32_t layer, uint32_t size, uint32_t weight, char *s);

    static bool add_circle(const char name[3], uint32_t layer, color_t color, ui_point_t center, uint32_t radius, uint32_t line_width);

    static bool add_line(const char name[3], uint32_t layer, color_t color, ui_point_t start_p, ui_point_t end_p, uint32_t line_width);

    static bool add_rect(const char name[3], uint32_t layer, color_t color, ui_point_t start_p, ui_point_t end_p, uint32_t line_width);

    static bool add_float(char name[3], uint32_t layer, color_t color, ui_point_t start_p, uint32_t font_size, float data);

    static bool add_int(const char name[3], uint32_t layer, color_t color, ui_point_t start_p, uint32_t font_size, int data);

    static bool revise_label(const char *name, char *string, color_t color);

    static bool revise_shape_loc(const char name[3], ui_point_t point, color_t color);

    static bool revise_line(const char name[3], ui_point_t start_p, ui_point_t end_p);

    static void remove_layer(uint32_t layer);

    static void remove_all();

    static void echo_shapes();

    static void echo_titles();

public:

    enum component_state_t{
        FINISHED,
        REVISING,
    };

    static component_state_t shape_state[REFEREE_UI_MAX_SHAPE_COUNT];
    static Referee::graphic_data_struct_t shapes[REFEREE_UI_MAX_SHAPE_COUNT];
    static long unsigned shape_last_revision_time[REFEREE_UI_MAX_SHAPE_COUNT];
    static int shape_count;

    static component_state_t label_state[REFEREE_UI_MAX_LABEL_COUNT];
    static Referee::ext_client_custom_character_t labels[REFEREE_UI_MAX_LABEL_COUNT];
    static long unsigned label_last_revision_time[REFEREE_UI_MAX_LABEL_COUNT];
    static int label_count;

    static mutex_t buffer_mutex;

    static int strlen(char *s);

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
    private:
        static constexpr unsigned SEND_INTERVAL = 110;
        void main() final;
    };
    static SKDThread skdThread;

    static bool invoke_ui_delete_all;

    static bool invoke_ui_delete_layer;
    static uint32_t layer_deleting;

    static bool send_deleting_all_if_requested();
    static bool send_deleting_layer_if_requested();
    static bool send_graphics_updates_if_requested();
    static bool send_label_update_if_requested();
};


#endif //META_INFANTRY_REFEREE_UI_UPDATE_SCHEDULER_H
