cmake_minimum_required(VERSION 3.12)
set(CXX_STANDARD 11)

project(convertkf VERSION 1.0.0)

set(HavokPath "E:/dev/libs/cpp/hk2010_2_0_r1")
set(HavokLibPath "${HavokPath}/Lib/win32_net_9-0/release_multithreaded")
set(HavokInclPath "${HavokPath}/Source")
set(NifLibPath "E:/dev/libs/cpp/niflib")
set(NifLibLibPath "${NifLibPath}/lib")
set(NifLibInclPath "${NifLibPath}/include")

add_executable(${PROJECT_NAME} impl.cpp main.cpp)

target_link_directories(
    ${PROJECT_NAME} 
    PRIVATE
        ${HavokLibPath})

file(GLOB_RECURSE HavokLibs ${HavokLibPath}/*.lib)
target_link_libraries(
    ${PROJECT_NAME} 
    PRIVATE
        ${HavokLibs}
        ${NifLibLibPath}/niflib_static.lib
)

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
        ${HavokInclPath}
        ${NifLibInclPath}
)

target_compile_definitions(
	${PROJECT_NAME}
	PRIVATE
		_UNICODE
)