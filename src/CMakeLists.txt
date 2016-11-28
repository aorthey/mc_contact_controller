set(controller_SRC
  mc_myfirst_controller.cpp
)

set(controller_HDR
  mc_myfirst_controller.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
#target_link_libraries(${PROJECT_NAME} mc_control)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
message(${MC_RTC_LIBDIR})
install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)
