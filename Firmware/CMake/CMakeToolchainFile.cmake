INCLUDE(CMakeForceCompiler)

# the name of the target operating system
set(CMAKE_SYSTEM_NAME ARM)

set(CMAKE_COMPILER_IS_GNUCC ON)

# which compilers to use for C and C++
CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)
set(CMAKE_ASM_COMPILER arm-none-eabi-as)

# here is the target environment located
set(CMAKE_FIND_ROOT_PATH usr/arm-none-eabi)

# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment, search 
# programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
