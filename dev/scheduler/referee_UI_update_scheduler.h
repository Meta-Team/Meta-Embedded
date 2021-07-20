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

    struct UI_point {
        uint32_t x;
        uint32_t y;
    };

    static void init(tprio_t SKDThreadPRIO);

    static void add_character(char* name, UI_point start_p, color_t color, uint32_t layer, uint32_t size, uint32_t weight, char *string);

    static void add_circle(char name[3], uint32_t layer, color_t color, UI_point center, uint32_t radius, uint32_t line_width);

    static void add_line(char name[3], uint32_t layer, color_t color, UI_point start_p, UI_point end_p, uint32_t line_width);

    static void add_rect(char name[3], uint32_t layer, color_t color, UI_point start_p, UI_point end_p, uint32_t line_width);

    static void add_float(char name[3], uint32_t layer, color_t color, UI_point start_p, uint32_t font_size, float data);

    static void add_int(char name[3], uint32_t layer, color_t color, UI_point start_p, uint32_t font_size, int data);

    static void revise_character(char name[3], char *string, color_t color);

    static void revise_shape_loc(char name[3], UI_point point, color_t color);

    static void revise_line(char name[3], UI_point start_p, UI_point end_p);

    static void remove_layer(uint32_t layer);

    static void remove_shape(char name[3]);

    static void remove_chara(char name[3]);

    static void remove_all();

    static void echo_shapes();

    static void echo_titles();
private:
    enum component_state_t{
        WAITING,
        REVISING,
        FINISHED
    };

    static component_state_t shape_state[REFEREE_UI_MAX_SHAPE_COUNT];
    static Referee::graphic_data_struct_t shapes[REFEREE_UI_MAX_SHAPE_COUNT];
    static int shape_count;

    static component_state_t label_state[REFEREE_UI_MAX_LABEL_COUNT];
    static Referee::ext_client_custom_character_t labels[REFEREE_UI_MAX_LABEL_COUNT];
    static int label_count;
    static int strlen(char *s);

    class SKDThread : public chibios_rt::BaseStaticThread<512> {
    private:
        int SKD_THREAD_INTERVAL = 100;
        void main() final;
    };
    static SKDThread skdThread;
};


#endif //META_INFANTRY_REFEREE_UI_UPDATE_SCHEDULER_H
