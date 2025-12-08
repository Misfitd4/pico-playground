if (TARGET pico_scanvideo_dpi AND TARGET tinyusb_device AND TARGET pico_audio_i2s)
    get_filename_component(SIDDLER_ROOT ${CMAKE_CURRENT_LIST_DIR}/../.. ABSOLUTE)
    set(SIDKICK_ROOT ${SIDDLER_ROOT}/apps/picoSid-synth)
    set(SIDKICK_RESID_DIR ${SIDKICK_ROOT}/lib/reSID16)

    if (NOT TARGET reSID16)
        add_library(reSID16 STATIC
                ${SIDKICK_RESID_DIR}/envelope.cc
                ${SIDKICK_RESID_DIR}/extfilt.cc
                ${SIDKICK_RESID_DIR}/filter.cc
                ${SIDKICK_RESID_DIR}/pot.cc
                ${SIDKICK_RESID_DIR}/sid.cc
                ${SIDKICK_RESID_DIR}/version.cc
                ${SIDKICK_RESID_DIR}/voice.cc
                ${SIDKICK_RESID_DIR}/wave.cc
                ${SIDKICK_RESID_DIR}/wave6581__ST.cc
                ${SIDKICK_RESID_DIR}/wave6581_P_T.cc
                ${SIDKICK_RESID_DIR}/wave6581_PS_.cc
                ${SIDKICK_RESID_DIR}/wave6581_PST.cc
                ${SIDKICK_RESID_DIR}/wave8580__ST.cc
                ${SIDKICK_RESID_DIR}/wave8580_P_T.cc
                ${SIDKICK_RESID_DIR}/wave8580_PS_.cc
                ${SIDKICK_RESID_DIR}/wave8580_PST.cc
        )
        target_include_directories(reSID16 PUBLIC
                ${SIDKICK_ROOT}/lib
                ${SIDKICK_RESID_DIR}
                ${SIDKICK_ROOT}/src
        )
        target_compile_definitions(reSID16 PRIVATE VERSION=\"0.16-siddler\")
        target_compile_features(reSID16 PRIVATE cxx_std_17)
    endif()

    if (NOT TARGET siddler_sid_engine)
        add_library(siddler_sid_engine STATIC
                ${SIDKICK_ROOT}/src/sid_engine.cpp
                ${SIDKICK_ROOT}/src/exodecr.c
        )
        target_include_directories(siddler_sid_engine PUBLIC
                ${SIDKICK_ROOT}/lib
                ${SIDKICK_ROOT}/src
                ${SIDKICK_RESID_DIR}
        )
        target_link_libraries(siddler_sid_engine PUBLIC reSID16 pico_stdlib)
        target_compile_features(siddler_sid_engine PUBLIC cxx_std_17)
    endif()

    set(SIDDLER_REFRESH_HZ "60" CACHE STRING "Default refresh rate for siddler_pico output (50 or 60)")

    add_executable(siddler_pico
            siddler_pico.c
            usb_descriptors.c
            siddler_audio.cpp
    )

    target_include_directories(siddler_pico PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            ${SIDKICK_ROOT}/lib
            ${SIDKICK_ROOT}/src
            ${SIDKICK_RESID_DIR}
    )

    target_link_libraries(siddler_pico PRIVATE
            pico_multicore
            pico_stdlib
            pico_scanvideo_dpi
            tinyusb_device
            tinyusb_board
            pico_audio_i2s
            siddler_sid_engine
    )

    target_compile_definitions(siddler_pico PRIVATE
            CFG_TUSB_MCU=OPT_MCU_RP2040
            PICO_AUDIO_I2S_DMA_IRQ=1
            PICO_AUDIO_I2S_PIO=1
            SIDDLER_DEFAULT_REFRESH_HZ=${SIDDLER_REFRESH_HZ}
    )

    pico_enable_stdio_usb(siddler_pico 1)
    pico_enable_stdio_uart(siddler_pico 0)

    pico_add_extra_outputs(siddler_pico)
endif ()
