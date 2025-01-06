set(BOARD_REVISIONS "runbmc_v2_2os"
                    "runbmc_v2_1os"
                    "runbmc_v3_2os"
                    "runbmc_v3_2os_xip"
                    "runbmc_v3_1os")
if(NOT DEFINED BOARD_REVISION)
  set(BOARD_REVISION "runbmc_v3_2os")
else()
  if(NOT BOARD_REVISION IN_LIST BOARD_REVISIONS)
    message(FATAL_ERROR "${BOARD_REVISION} is not a valid revision for lsqsh_evb.
                          Accepted revisions: ${BOARD_REVISIONS}")
  endif()
endif()
