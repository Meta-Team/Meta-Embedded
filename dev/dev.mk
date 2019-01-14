# ===========================================================================
# COMMON INCLUDES AND SOURCES
#
#	Add common modules and utilities into this section.
#
# ===========================================================================


DEV_COMMON_CSRC =
DEV_COMMON_CPPSRC = common/port_to_string.cpp \
			        debug/serial_shell.cpp \
			        debug/shell_debug_commands.cpp \
			        debug/led.cpp
DEV_COMMON_INC = . \
		         common \
		         debug

# ===========================================================================
# MAIN MODULES
# ===========================================================================

DEV_MAIN_CSRC =
DEV_MAIN_CPPSRC = interfaces/remote_interpreter.cpp \
			      main.cpp
DEV_MAIN_INC = interfaces


# ===========================================================================
# REMOTE_INTERPRETER MODULES
#
#	Include remote_interpreter sources and the unit test modules.
#
# ===========================================================================

DEV_REMOTE_INTERPRETER_CSRC =
DEV_REMOTE_INTERPRETER_CPPSRC = interfaces/remote_interpreter.cpp \
                                interfaces/remote_interpreter_unit_test.cpp
DEV_REMOTE_INTERPRETER_INC = interfaces

# ===========================================================================
# CHASSIS_INTERFACE MODULES
#
#	Include chassis interface sources and the unit test modules.
#
# ===========================================================================
DEV_CHASSIS_INTERFACE_CSRC =
DEV_CHASSIS_INTERFACE_CPPSRC = interfaces/can_interface.cpp \
							   interfaces/chassis_interface.cpp \
                               interfaces/chassis_interface_unit_test.cpp
DEV_CHASSIS_INTERFACE_INC =  interfaces

# ===========================================================================
# GIMBAL_INTERFACE MODULES
#
#	Include CANInterface, GimbalInterface, GimbalFeedbackModule and the unit
#   test modules.
#
# ===========================================================================

DEV_GIMBAL_INTERFACE_CSRC =
DEV_GIMBAL_INTERFACE_CPPSRC = interfaces/can_interface.cpp \
                              interfaces/gimbal_interface.cpp \
                              interfaces/gimbal_interface_unit_test.cpp \
                              debug/gimbal_feedback_module.cpp
DEV_GIMBAL_INTERFACE_INC = interfaces

# ===========================================================================
# GIMBAL MODULES
#
#	Include CANInterface, GimbalInterface, GimbalController,
# 	GimbalFeedbackModule and the unit test and adjustment modules.
#
# ===========================================================================

DEV_GIMBAL_CSRC =
DEV_GIMBAL_CPPSRC = interfaces/can_interface.cpp \
                    interfaces/gimbal_interface.cpp \
                    module/pid_controller.cpp \
                    control/gimbal_controller.cpp \
					debug/gimbal_feedback_module.cpp \
                    control/gimbal_controller_unit_test.cpp
DEV_GIMBAL_INC = interfaces \
				 module \
				 control


# ===========================================================================
# RULES
#
# 	Add common components and files specifized by parameter DEV_MODULE to
#   the aggregate list of Makefiles.
#
# ===========================================================================
ALLCSRC += $(DEV_COMMON_CSRC) \
           $(DEV_$(DEV_MODULE)_CSRC)
ALLCPPSRC += $(DEV_COMMON_CPPSRC) \
		     $(DEV_$(DEV_MODULE)_CPPSRC)
ALLINC  += $(DEV_COMMON_INC) \
		   $(DEV_$(DEV_MODULE)_INC)