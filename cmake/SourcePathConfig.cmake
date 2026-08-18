#   Define the data path for program

set( SHADER_FILE_PATH_GEN1  "${SMROBOT_DATA_ROOT}/shader_gen1/" CACHE PATH "OpenGL Shader Program Path for SMRobotGen1" FORCE )
set( SHADER_FILE_PATH_GEN2  "${SMROBOT_DATA_ROOT}/shader_gen2/" CACHE PATH "OpenGL Shader Program Path for SMRobotGen2" FORCE )

set( PROJECT_SOURCE_PATH  "${PROJECT_SOURCE_DIR}/" CACHE PATH "Project Source Path" FORCE )
set( SOURCE_FILE_PATH  "${PROJECT_SOURCE_DIR}/" CACHE PATH "Project Source Path" FORCE )


set( DATA_PATH  "${SMROBOT_DATA_ROOT}/" CACHE PATH "Robot data path" FORCE )
set( DATA_MODEL_PATH  "${SMROBOT_DATA_ROOT}/model/" CACHE PATH "Robot model(xml) path" FORCE )
set( DATA_ROBOT_PATH  "${SMROBOT_DATA_ROOT}/robot/" CACHE PATH "Robot data(rbt) path" FORCE )
set( DATA_SYSTEM_PATH  "${SMROBOT_DATA_ROOT}/system/" CACHE PATH "Robot system(rbtsys) path" FORCE )
set( DATA_PCL_PATH  "${SMROBOT_DATA_ROOT}/pcl/" CACHE PATH "Point Cloud Map Path" FORCE )

configure_file( include/data_path.h.in include/data_path.h @ONLY )
include_directories( ${PROJECT_BINARY_DIR}/include )



#   Define the version of the project that is set in the CMake file could be used in program.
configure_file( include/config.h.in include/config.h @ONLY )

