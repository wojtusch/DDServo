if (CMAKE_COMPILER_IS_GNUCC)

  # add system specific compiler flags
  if (CMAKE_SYSTEM_NAME MATCHES "ARM")
    # compile for ARM Cortex M3
    set(CMAKE_C_FLAGS "-mthumb -mcpu=cortex-m3 -mfix-cortex-m3-ldrd")
    add_definitions(-DSTM32F10X_MD)
    add_definitions(-DUSE_STDPERIPH_DRIVER)
  endif ()

  # -Winline: warn if inlining of a inlinable method failed
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0 -Wall -Wextra -Winline")

  # Release build
  set(CMAKE_C_FLAGS_RELEASE "-O2 -DRELEASE")
  # Release build with debug symbols
  set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g -O2 -DRELEASE")
  # Debug Build
  set(CMAKE_C_FLAGS_DEBUG "-g -O0 -DDEBUG")

  # set default build type
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
  endif ()

  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined -Wl,--as-needed ${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS "-Wl,--no-undefined -Wl,--as-needed ${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,--no-undefined -Wl,--as-needed ${CMAKE_EXE_LINKER_FLAGS}")
  endif ()

endif ()
