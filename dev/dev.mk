# ===========================================================================
# Common Includes and Sources

DEV_COMMON_CSRC =
DEV_COMMON_CPPSRC = common/port_to_string.cpp \
			        debug/button_monitor.cpp \
			        debug/serial_shell.cpp \
			        debug/serial_shell_commands.cpp \
			        debug/led.cpp
DEV_COMMON_INC = . \
		         common \
		         debug

DEV_MAIN_CSRC =
DEV_MAIN_CPPSRC = interfaces/remote_interpreter.cpp \
				  interfaces/gimbal_process_function.cpp \
				  interfaces/send_currents_functions.cpp \
			      main.cpp
DEV_MAIN_INC = interfaces



DEV_REMOTE_INTERPRETER_CSRC =
DEV_REMOTE_INTERPRETER_CPPSRC = debug/unit_test_common.cpp \
						        interfaces/remote_interpreter.cpp \
                                interfaces/remote_interpreter_unit_test.cpp
DEV_REMOTE_INTERPRETER_INC = interfaces


# Rules
ALLCSRC += $(DEV_COMMON_CSRC) \
           $(DEV_$(DEV_MODULE)_CSRC)
ALLCPPSRC += $(DEV_COMMON_CPPSRC) \
		     $(DEV_$(DEV_MODULE)_CPPSRC)
ALLINC  += $(DEV_COMMON_INC) \
		   $(DEV_$(DEV_MODULE)_INC)