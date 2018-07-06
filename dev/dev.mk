DEVCSRC = 

DEVCPPSRC = $(CHIBIOS)/dev/common/port_to_string.cpp \
			$(CHIBIOS)/dev/debug/button_monitor.cpp \
			$(CHIBIOS)/dev/debug/serial_shell.cpp \
			$(CHIBIOS)/dev/debug/serial_shell_commands.cpp \
			$(CHIBIOS)/dev/main.cpp

DEVINC = $(CHIBIOS)/dev \
		 $(CHIBIOS)/dev/common \
		 $(CHIBIOS)/dev/debug

# Shared variables
ALLCSRC += $(DEVCSRC)
ALLCPPSRC += $(DEVCPPSRC)
ALLINC  += $(DEVINC)
