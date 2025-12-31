set(BOARD_REVISIONS "2os"
                    "2os_xip"
                    "1os"
                    "1os_xip"
                    "smp")
if(NOT DEFINED BOARD_REVISION)
  set(BOARD_REVISION "1os")
else()
  if(NOT BOARD_REVISION IN_LIST BOARD_REVISIONS)
    message(FATAL_ERROR "${BOARD_REVISION} is not a valid revision for lsqsh_evb.
                          Accepted revisions: ${BOARD_REVISIONS}")
  endif()
endif()

list(APPEND EXTRA_DTC_FLAGS "-Wno-unique_unit_address_if_enabled")
