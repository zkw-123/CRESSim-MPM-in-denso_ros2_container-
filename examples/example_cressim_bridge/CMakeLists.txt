set(BRIDGE_TARGET_NAME "cressim_bridge")
set(TEST_TARGET_NAME "test_cressim_bridge")

add_library(${BRIDGE_TARGET_NAME} SHARED
    cressim_bridge.cpp
)

target_include_directories(${BRIDGE_TARGET_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${BRIDGE_TARGET_NAME} PRIVATE
    Eigen3::Eigen
    crmpm_engine
)

set_target_properties(${BRIDGE_TARGET_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_executable(${TEST_TARGET_NAME}
    test_bridge.cpp
)

target_include_directories(${TEST_TARGET_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(${TEST_TARGET_NAME} PRIVATE
    ${BRIDGE_TARGET_NAME}
)

set_target_properties(${TEST_TARGET_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)