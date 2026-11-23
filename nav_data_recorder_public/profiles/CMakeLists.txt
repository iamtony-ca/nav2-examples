cmake_minimum_required(VERSION 3.12)
project(nav_data_profiles NONE)

find_package(ament_cmake REQUIRED)

# Every sub-directory is one profile, installed to share/nav_data_profiles/profiles/<profile>/.
# CONFIGURE_DEPENDS: a new profile directory is picked up by the next build without touching this file.
file(GLOB entries LIST_DIRECTORIES true CONFIGURE_DEPENDS
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(entry ${entries})
  if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${entry})
    install(DIRECTORY ${entry} DESTINATION share/${PROJECT_NAME}/profiles)
  endif()
endforeach()

ament_package()
