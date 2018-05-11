#--------------------------------------------------------------------
# Inviwo ITK Module
ivw_module(ITK)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/itkprocessor.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/imagedemondeformation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/itkimagelowpass.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/itktest.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/meshtovector.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/volumedemondeformation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/volumegradient.h
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/itklowpass.h
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/itktypes.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/itkprocessor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/imagedemondeformation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/itkimagelowpass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/itktest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/meshtovector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/volumedemondeformation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processors/volumegradient.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/itklowpass.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/itktypes.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})


#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
    #${CMAKE_CURRENT_SOURCE_DIR}/glsl/itkprocessor.frag
)
ivw_group("Shader Files" ${SHADER_FILES})


#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/itk-types-test.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tests/unittests/itk-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Add shader directory to pack
# ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)

target_link_libraries(${${mod}_target} PUBLIC ${ITK_LIBRARIES})