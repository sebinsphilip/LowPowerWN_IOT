# For TMote Sky (emulated in Cooja) use the following target
ifndef TARGET
	TARGET = sky
else
	# For Zolertia Firefly (testbed) use the following target and board
	# Don't forget to make clean when you are changing the board
	ifeq ($(TARGET), zoul)
		BOARD	?= firefly
		LDFLAGS += -specs=nosys.specs # For Zolertia Firefly only
	endif
endif


DEFINES=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = app app-collect


PROJECT_SOURCEFILES += my_collect.c

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_RIME = 1
CONTIKI ?= ../../contiki
include $(CONTIKI)/Makefile.include
