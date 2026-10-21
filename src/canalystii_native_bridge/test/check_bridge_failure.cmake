execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env
    "ROS_DOMAIN_ID=212"
    "ROS_LOG_DIR=${LOG_DIR}"
    "${BRIDGE}" --ros-args
      -p "channel_ids:=[0, 1]"
      -p "bus_names:=[usb_failure_test0, usb_failure_test1]"
      -p "shutdown_grace_period_s:=0.0"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE errors
  TIMEOUT 10
)
if(NOT "${result}" STREQUAL "1")
  message(FATAL_ERROR "Expected transport failure exit code 1, got ${result}\n${output}${errors}")
endif()
if(NOT "${output}${errors}" MATCHES "RX transfer failed on channel 1 \\(status 1\\)")
  message(FATAL_ERROR "Expected injected USB RX failure was not reported\n${output}${errors}")
endif()
