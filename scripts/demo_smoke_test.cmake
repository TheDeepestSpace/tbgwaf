# ctest driver for demo_smoke_test (issue #240): runs the app headlessly
# through the Demo Scenes flow with recording on, then replays the written
# log with tactics_repro_replay. Invoked by CMakeLists.txt with -DXVFB,
# -DAPP, -DREPLAY and -DLOG.
file(REMOVE "${LOG}")
execute_process(
  COMMAND ${XVFB} --auto-servernum "--server-args=-screen 0 1280x1024x24" ${APP}
  RESULT_VARIABLE app_result
)
if(NOT app_result EQUAL 0)
  message(FATAL_ERROR "tactics_app (demo smoke) exited with ${app_result}")
endif()
if(NOT EXISTS "${LOG}")
  message(FATAL_ERROR "the app did not write the repro log ${LOG}")
endif()
execute_process(COMMAND ${REPLAY} "${LOG}" RESULT_VARIABLE replay_result)
if(NOT replay_result EQUAL 0)
  message(FATAL_ERROR "tactics_repro_replay failed on ${LOG} (exit ${replay_result})")
endif()
