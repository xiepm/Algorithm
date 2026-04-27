cmake_minimum_required(VERSION 2.8)
project(CmpHansCobotAlgorithm)

SET(CMAKE_DUILD_TYPE Release)

#add_definitions(-std=c++11)
#add_definitions(-std=c11)
set(CMAKE_CXX_FLAGS    "${CMAKE_CXX_FLAGS} -s -Wall -Wextra -fvisibility=hidden -fvisibility-inlines-hidden")


include_directories(
                src
                src/dynamics
                src/kinematics
                external/Components
		        external/Platforms/Linux
                external/eigen3
        )

aux_source_directory(. cds)
aux_source_directory(src srcs)
aux_source_directory(src/dynamics srcs)
aux_source_directory(src/kinematics srcs)



add_library(CmpHansCobotAlgorithm SHARED ${srcs} ${cds} 
                                        ElfinCollaborativeLib.c
                                        algorithmInterface.cpp)
target_include_directories(CmpHansCobotAlgorithm PUBLIC .)



aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/HRCInterface INTERFACE)
add_library(cobot SHARED ${srcs} ${INTERFACE})
# ���õ������ż�������
set_target_properties(cobot PROPERTIES 
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1
)
# ������̬��ķ��ű��ⲿ��������
target_compile_options(cobot PRIVATE -fvisibility=hidden)
# ����ͷ�ļ�·��
target_include_directories(cobot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/HRCInterface)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set_target_properties(cobot PROPERTIES LINK_FLAGS "-Wl,--export-dynamic")
endif()


# ����һ��test demo���ֶ�����bug��
# ��������ִ���ļ���
include_directories(test
                    HRCInterface)
                    

aux_source_directory(test TEST)
add_executable(testDemo ${TEST} ${srcs} ${INTERFACE})
target_include_directories(testDemo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/HRCInterface)
target_include_directories(testDemo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/test)
