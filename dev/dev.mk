# Common files and directories
DEV_COMMON_CSRC = 

DEV_COMMON_CPPSRC = $(CHIBIOS)/dev/common/port_to_string.cpp \
			        $(CHIBIOS)/dev/debug/button_monitor.cpp \
			        $(CHIBIOS)/dev/debug/serial_shell.cpp \
			        $(CHIBIOS)/dev/debug/serial_shell_commands.cpp \
			        $(CHIBIOS)/dev/debug/led.cpp

DEV_COMMON_INC = $(CHIBIOS)/dev \
		         $(CHIBIOS)/dev/common \
		         $(CHIBIOS)/dev/debug

# Remote Interpreter
REMOTE_INTERPRETER_CSRC =
REMOTE_INTERPRETER_CPPSRC = $(CHIBIOS)/dev/remote_interpreter/remote_interpreter.cpp
REMOTE_INTERPRETER_INC = $(CHIBIOS)/dev/remote_interpreter

UNIT_TEST_REMOTE_INTERPRETER_CSRC = $(REMOTE_INTERPRETER_CSRC)
UNIT_TEST_REMOTE_INTERPRETER_CPPSRC = $(REMOTE_INTERPRETER_CPPSRC) \
                                      $(CHIBIOS)/dev/debug/unit_test_common.cpp \
                                      $(CHIBIOS)/dev/remote_interpreter/remote_interpreter_unit_test.cpp
UNIT_TEST_REMOTE_INTERPRETER_INC = $(REMOTE_INTERPRETER_INC)


# Main
UNIT_TEST_NONE_CSRC = $(REMOTE_INTERPRETER_CSRC)
UNIT_TEST_NONE_CPPSRC = $(REMOTE_INTERPRETER_CPPSRC) \
	                    $(CHIBIOS)/dev/main.cpp
UNIT_TEST_NONE_INC = $(REMOTE_INTERPRETER_INC)

# Rules
ALLCSRC += $(DEV_COMMON_CSRC) \
           $(UNIT_TEST_$(UNIT_TEST)_CSRC)
ALLCPPSRC += $(DEV_COMMON_CPPSRC) \
		   $(UNIT_TEST_$(UNIT_TEST)_CPPSRC)
ALLINC  += $(DEV_COMMON_INC) \
		   $(UNIT_TEST_$(UNIT_TEST)_INC)