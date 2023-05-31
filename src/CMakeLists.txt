add_library(maritime_ec
	mixer.c
	util.c
	estimation.c
	pid_controller.c
	att_controller.c
	angvel_controller.c
	position_controller.c
	velocity_controller.c
)

target_include_directories(maritime_ec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
