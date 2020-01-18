//
// Created by zzb on 2020/1/18.
//

/*
 * This logical part will specify the automated procedure of grabbing bullets, rescuing other vehicles
 * This class will control the following classes:
 *      engineer_skd
 *          engineer_interface
 *      air_tank_interface
 */
#ifndef META_INFANTRY_ENGINEER_FUNCTIONS_LOGIC_H
#define META_INFANTRY_ENGINEER_FUNCTIONS_LOGIC_H


class engineer_functions_logic {
public :
    enum grab_mode_t{
        mode1,
        mode2
    };

    static int grabMode;

    static void set_mode(int mode_);

    static void grab();


};


#endif //META_INFANTRY_ENGINEER_FUNCTIONS_LOGIC_H
