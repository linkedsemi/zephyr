set(BOARD_REVISIONS "2os"
                    "2os_xip"
                    "1os")
if(NOT DEFINED BOARD_REVISION)
  set(BOARD_REVISION "2os")
else()
  if(NOT BOARD_REVISION IN_LIST BOARD_REVISIONS)
    message(FATAL_ERROR "${BOARD_REVISION} is not a valid revision for lsqsh_evb.
                          Accepted revisions: ${BOARD_REVISIONS}")
  endif()
endif()
