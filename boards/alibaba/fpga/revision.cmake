set(BOARD_REVISIONS "zynq"
                    "acku"
                    "acku-sc"
                    "acku-xip")
if (NOT DEFINED BOARD_REVISION)
  set(BOARD_REVISION "acku")
else ()
  if (NOT (BOARD_REVISION STREQUAL "zynq") 
      AND NOT (BOARD_REVISION STREQUAL "acku") 
      AND NOT (BOARD_REVISION STREQUAL "acku-sc")
      AND NOT (BOARD_REVISION STREQUAL "acku-xip"))
    message(FATAL_ERROR "Invalid board revision, ${BOARD_REVISION}, valid revisions are: zynq, acku, acku-sc, acku-xip")
  endif()
endif()
