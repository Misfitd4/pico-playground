if (TARGET tinyusb_device AND (TARGET pico_audio_i2s OR TARGET pico_audio_pwm))
    project(picoSid-synth C CXX)

    set(RE_SID16_SOURCES
            lib/reSID16/envelope.cc
            lib/reSID16/extfilt.cc
            lib/reSID16/filter.cc
            lib/reSID16/pot.cc
            lib/reSID16/sid.cc
            lib/reSID16/version.cc
            lib/reSID16/voice.cc
            lib/reSID16/wave.cc
            lib/reSID16/wave6581__ST.cc
            lib/reSID16/wave6581_P_T.cc
            lib/reSID16/wave6581_PS_.cc
            lib/reSID16/wave6581_PST.cc
            lib/reSID16/wave8580__ST.cc
            lib/reSID16/wave8580_P_T.cc
            lib/reSID16/wave8580_PS_.cc
            lib/reSID16/wave8580_PST.cc
    )

    add_library(reSID16 STATIC ${RE_SID16_SOURCES})
    target_include_directories(reSID16 PUBLIC
            ${CMAKE_CURRENT_LIST_DIR}/lib/reSID16
            ${CMAKE_CURRENT_LIST_DIR}/src
    )
    target_compile_definitions(reSID16 PRIVATE VERSION=\"0.16-picoSid\")

    if (DEFINED SID_CHIP_MODEL)
        set(_SID_LEFT_MODEL_DEFAULT "${SID_CHIP_MODEL}")
        set(_SID_RIGHT_MODEL_DEFAULT "${SID_CHIP_MODEL}")
    else()
        set(_SID_LEFT_MODEL_DEFAULT "6581")
        set(_SID_RIGHT_MODEL_DEFAULT "8580")
    endif()

    set(SID_LEFT_MODEL "${_SID_LEFT_MODEL_DEFAULT}" CACHE STRING "Left channel SID model (6581 or 8580)")
    set(SID_RIGHT_MODEL "${_SID_RIGHT_MODEL_DEFAULT}" CACHE STRING "Right channel SID model (6581 or 8580)")

    if (NOT SID_LEFT_MODEL STREQUAL "6581" AND NOT SID_LEFT_MODEL STREQUAL "8580")
        message(FATAL_ERROR "SID_LEFT_MODEL must be 6581 or 8580")
    endif()
    if (NOT SID_RIGHT_MODEL STREQUAL "6581" AND NOT SID_RIGHT_MODEL STREQUAL "8580")
        message(FATAL_ERROR "SID_RIGHT_MODEL must be 6581 or 8580")
    endif()

    set(PICOSID_AUDIO_BACKEND "I2S" CACHE STRING "Audio backend: I2S or PWM")
    string(TOUPPER "${PICOSID_AUDIO_BACKEND}" PICOSID_AUDIO_BACKEND_UPPER)
    if (PICOSID_AUDIO_BACKEND_UPPER STREQUAL "I2S")
        set(PICOSID_USE_I2S 1)
        set(PICOSID_USE_PWM 0)
    elseif (PICOSID_AUDIO_BACKEND_UPPER STREQUAL "PWM")
        set(PICOSID_USE_I2S 0)
        set(PICOSID_USE_PWM 1)
    else()
        message(FATAL_ERROR "PICOSID_AUDIO_BACKEND must be I2S or PWM")
    endif()

    add_executable(picoSid-synth
            src/main.cpp
    )

    target_include_directories(picoSid-synth PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/src
            ${CMAKE_CURRENT_LIST_DIR}/lib
            ${CMAKE_CURRENT_LIST_DIR}
    )

    target_link_libraries(picoSid-synth
            pico_stdlib
    )

    pico_enable_stdio_usb(picoSid-synth 1)
    pico_enable_stdio_uart(picoSid-synth 0)

    pico_add_extra_outputs(picoSid-synth)
endif()
