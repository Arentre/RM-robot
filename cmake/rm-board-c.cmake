# RM-C: mechanisms, CRSF, BMI088, auto-aim and USB debugging.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    User/bsp/c/adc.c
    User/bsp/c/buzzer.c
    User/bsp/c/can.c
    User/bsp/c/crc32.c
    User/bsp/c/delay.c
    User/bsp/c/flash.c
    User/bsp/c/fric.c
    User/bsp/c/gpio.c
    User/bsp/c/i2c.c
    User/bsp/c/laser.c
    User/bsp/c/led.c
    User/bsp/c/mm.c
    User/bsp/c/pwm.c
    User/bsp/c/rand.c
    User/bsp/c/servo.c
    User/bsp/c/spi.c
    User/bsp/c/uart.c
    User/bsp/c/usb.c
    User/component/ahrs.c
    User/component/ballistics.c
    User/component/cmd.c
    User/component/crc8.c
    User/component/crc16.c
    User/component/error_detect.c
    User/component/filter.c
    User/component/limiter.c
    User/component/mixer.c
    User/component/pid.c
    User/component/ladrc.c
    User/component/gimbal_ff.c
    User/component/user_math.c
    User/device/bmi088.c
    User/device/can.c
    User/device/crsf.c
    User/device/nav_link.c
    User/device/ai.c
    User/module/chassis.c
    User/module/config.c
    User/module/control_params.c
    User/module/gimbal.c
    User/module/shoot.c
    User/task/ai.c
    User/task/atti_esti.c
    User/task/can.c
    User/task/command.c
    User/task/ctrl_chassis.c
    User/task/ctrl_gimbal.c
    User/task/ctrl_shoot.c
    User/task/info.c
    User/task/init.c
    User/task/rc.c
    User/task/user_task.c
)
target_include_directories(stm32cubemx INTERFACE
    ${PROJECT_SOURCE_DIR}/User
    ${PROJECT_SOURCE_DIR}/Middlewares/Third_Party/Protocol
)
target_compile_definitions(stm32cubemx INTERFACE
    DEV_BOARD_C
    "__packed=__attribute__((__packed__))"
)
# FreeRTOS statistics and CLI output use floating-point printf.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,-u,_printf_float)
