
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = hugui

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ..\e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./calibration.c \
		./pid_regulator.c \
		./mass_computation.c \

#Header folders to include
INCDIR += 

# Compiler without optimisation options here. Comment to add TOF 
#USE_OPT = -ggdb -fomit-frame-pointer -falign-functions=16
#USE_OPT += -fno-strict-aliasing
#USE_OPT += -fstack-protector-all -L .

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile
