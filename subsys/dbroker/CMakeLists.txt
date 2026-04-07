# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
	dbroker.c
	socketpool.c
)
zephyr_library_include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/basu/zephyr/include
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/dbus-broker/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/dbus-broker/zephyr/include
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/dbus-broker/subprojects/libcstdaux-1/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/dbus-broker/subprojects/libcrbtree-3/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/dbus-broker/subprojects/libclist-3/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/dbus-broker/subprojects/libcdvar-1/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/dbus-broker/subprojects/libcutf8-1/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/basu/include
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/basu/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/basu/src/libsystemd
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/basu/src/libsystemd/sd-bus
	${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lib/basu/src/basic
)

zephyr_library_link_libraries(
	dbus_broker
	basu
)

if(CONFIG_DBUS_BROKER)
	zephyr_compile_definitions(
		CONFIG_DBUS_BROKER_PORT=${CONFIG_DBUS_BROKER_PORT}
		CONFIG_DBUS_BROKER_STACK_SIZE=${CONFIG_DBUS_BROKER_STACK_SIZE}
		CONFIG_DBUS_BROKER_PRIORITY=${CONFIG_DBUS_BROKER_PRIORITY}
		CONFIG_DBUS_BROKER_SOCKETPOOL_SIZE=${CONFIG_DBUS_BROKER_SOCKETPOOL_SIZE}
		CONFIG_DBUS_BROKER_SOCKETPOOL_BUFFER_SIZE=${CONFIG_DBUS_BROKER_SOCKETPOOL_BUFFER_SIZE}
	)
endif()

zephyr_include_directories(include)
