cmake_minimum_required(VERSION 3.20)

set(TARGET_NAME CollisionDetectorConfig)

set(SOURCES
    CollisionDetectorCommandController.cpp
    CollisionDetectorCommandController.h
    CollisionDetectorConfigModuleController.cpp
    CollisionDetectorConfigModuleController.h
    CollisionDetectorDocumentFacade.cpp
    CollisionDetectorDocumentFacade.h
    CollisionDetectorConfigWidget.cpp
    CollisionDetectorConfigWidget.h
    CollisionDetectorQueryDialog.cpp
    CollisionDetectorQueryDialog.h
    CollisionDetectorWorkbenchController.cpp
    CollisionDetectorWorkbenchController.h
    CollisionDetectorWorkbenchDocumentFacade.cpp
    CollisionDetectorWorkbenchDocumentFacade.h
    CollisionDetectorsController.cpp
    CollisionDetectorsController.h
    CollisionDetectorsViewModel.h
    CollisionDetectorsWidget.cpp
    CollisionDetectorsWidget.h
    CollisionDocumentEventPublisher.cpp
    CollisionDocumentEventPublisher.h
    CollisionLegacyPairDocumentFacade.cpp
    CollisionLegacyPairDocumentFacade.h
    CollisionLegacyPairWorkbenchController.cpp
    CollisionLegacyPairWorkbenchController.h
    CollisionLegacyPairsViewModel.h
    CollisionPairWorkflowController.cpp
    CollisionPairWorkflowController.h
    CollisionSelectionSetDocumentFacade.cpp
    CollisionSelectionSetDocumentFacade.h
    CollisionSelectionSetDocumentStorage.cpp
    CollisionSelectionSetDocumentStorage.h
    CollisionSelectionSetViewController.cpp
    CollisionSelectionSetViewController.h
    CollisionSelectionSetWorkbenchController.cpp
    CollisionSelectionSetWorkbenchController.h
    CollisionSelectionSetsController.cpp
    CollisionSelectionSetsController.h
    CollisionSelectionSetsViewModel.h
    CollisionSelectionSetsWidget.cpp
    CollisionSelectionSetsWidget.h
    CollisionWorkbenchEventCoordinator.cpp
    CollisionWorkbenchEventCoordinator.h
    CollisionWorkbenchModuleController.cpp
    CollisionWorkbenchModuleController.h
    CollisionWorkbenchPanel.cpp
    CollisionWorkbenchPanel.h
)

add_library(${TARGET_NAME} STATIC ${SOURCES})
add_library(RobotQtModulesCollisionDetectorConfig ALIAS ${TARGET_NAME})
add_library(SMRobotWorkbenchCollisionConfig::${TARGET_NAME} ALIAS ${TARGET_NAME})

target_include_directories(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:${PACKAGE_NAME}/include/${TARGET_NAME}>
)

target_link_libraries(${TARGET_NAME}
    PUBLIC
        $<BUILD_INTERFACE:CollisionLinkModelSetup>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCollisionConfig::CollisionLinkModelSetup>
        $<BUILD_INTERFACE:CollisionRuntimeResults>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCollisionConfig::CollisionRuntimeResults>
        $<BUILD_INTERFACE:RobotQtModulesShared>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCommon::RobotQtModulesShared>
    PRIVATE
        $<BUILD_INTERFACE:RobotQtModulesCoreWidgets>
        $<INSTALL_INTERFACE:SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets>
)

target_compile_features(${TARGET_NAME}
    PUBLIC
        cxx_std_17
)

set_target_properties(${TARGET_NAME}
    PROPERTIES
        FOLDER SMRobotWorkbenchCollisionConfig/CollisionConfigWorkbench
        DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
        RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)

if(ExportPacakge${PACKAGE_NAME})
    set(${TARGET_NAME}_INSTALL_PUBLIC_DEPENDENCIES
        SMRobotWorkbenchCommon::RobotQtModulesShared
        SMRobotWorkbenchCommon::RobotQtModulesCoreWidgets
        SMRobotWorkbenchCollisionConfig::CollisionLinkModelSetup
        SMRobotWorkbenchCollisionConfig::CollisionRuntimeResults
    )

    set(${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/install_headers)
    file(REMOVE_RECURSE ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR})
    file(MAKE_DIRECTORY ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME})

    foreach(header_ IN LISTS SOURCES)
        if(header_ MATCHES "\\.h$")
            configure_file(
                ${CMAKE_CURRENT_SOURCE_DIR}/${header_}
                ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}/${TARGET_NAME}/${header_}
                COPYONLY)
        endif()
    endforeach()
    unset(header_)

    function_InstallTarget(
        TARGET_NAME ${TARGET_NAME}
        PACKAGE_NAME ${PACKAGE_NAME}
        PACKAGE_INTERFACE_HEADERS ${${TARGET_NAME}_INSTALL_HEADER_STAGING_DIR}
    )
endif()
