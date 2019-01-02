#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#
VERBOSE := 1
PROJECT_NAME := ssd-display
CFLAGS += -I$(PROJECT_PATH)/components/misc -I$(PROJECT_PATH)/components/ssd1306 -I$(PROJECT_PATH)/components/i2c -DDEBUG=1
include $(IDF_PATH)/make/project.mk

