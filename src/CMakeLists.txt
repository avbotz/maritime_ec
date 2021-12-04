add_library(maritime_ec
	mixer.cpp
	estimation.cpp
	pid_controller.cpp
	att_controller.cpp
	angvel_controller.cpp
	position_controller.cpp
	velocity_controller.cpp
)

target_include_directories(maritime_ec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
target_include_directories(maritime_ec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include/matrix)
