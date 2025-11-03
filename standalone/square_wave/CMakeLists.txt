add_executable(square_wave
        square_wave.c
)

target_link_libraries(square_wave PRIVATE pico_stdlib)

pico_add_extra_outputs(square_wave)
