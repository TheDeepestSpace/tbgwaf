# ctest driver for builder_smoke_test: runs the app headlessly through the
# Builder flow into a hotseat game (TBGWAF_SMOKE_BUILDER) with recording on,
# then replays the written log with tactics_repro_replay. Invoked by
# CMakeLists.txt with -DXVFB, -DAPP, -DREPLAY and -DLOG.
file(REMOVE "${LOG}")
execute_process(
  COMMAND ${XVFB} --auto-servernum "--server-args=-screen 0 1280x1024x24" ${APP}
  RESULT_VARIABLE app_result
)
if(NOT app_result EQUAL 0)
  message(FATAL_ERROR "tactics_app (builder smoke) exited with ${app_result}")
endif()
if(NOT EXISTS "${LOG}")
  message(FATAL_ERROR "the app did not write the repro log ${LOG}")
endif()
file(READ "${LOG}" log_text)
if(NOT log_text MATCHES "\"builder_scene\"")
  message(FATAL_ERROR "${LOG} is not a Builder log (no builder_scene header)")
endif()
if(NOT log_text MATCHES "\"op\":\"commit\"")
  message(FATAL_ERROR "${LOG} records no commit input (the scripted round did not run)")
endif()
execute_process(COMMAND ${REPLAY} "${LOG}" RESULT_VARIABLE replay_result)
if(NOT replay_result EQUAL 0)
  message(FATAL_ERROR "tactics_repro_replay failed on ${LOG} (exit ${replay_result})")
endif()
