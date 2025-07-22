###-----------------------------------------------------------------------------
# Source files
CXX_SOURCES = OA.cpp  # Replace with your C++ source files

# Include paths
INCLUDE = -I"E:/VIII_sem_project_robotics/Webots/include" \
          -I"$(WEBOTS_HOME)/include/controller/cpp" \
          -I"C:/Python/Python310/include"  # Path to Python headers

# Linker flags
LDFLAGS = -L"C:/Python310/libs"  # Path to Python libraries (e.g., libpython310.a)
LDLIBS = -lpython310  # Link against Python 3.10

# Do not modify below line
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
