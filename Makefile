#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#
PROJECT_NAME := esp-aliyun-dc1

EXTRA_COMPONENT_DIRS := $(realpath ../../..)

SDKCONFIG_DEFAULTS := sdkconfig_$(chip).defaults

include $(IDF_PATH)/make/project.mk