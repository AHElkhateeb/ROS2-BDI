cmake_minimum_required(VERSION 3.5)
project(ros2_bdi_behaviours)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(ros2_bdi_interfaces REQUIRED)
find_package(ros2_bdi_utils REQUIRED)

include_directories(include)

set(BEHAVIOURS-SOURCES
  src/ContractNetResponder.cpp
  src/ContractNetInitiator.cpp  
  src/ConcreteContractNetResponderStates.cpp
  src/ConcreteContractNetInitiatorStates.cpp
  src/ACLMessage.cpp
  src/conversations_client.cpp
  src/MessageTemplate.cpp
  src/ConcreteOutgoingMessageStates.cpp
  src/OutgoingMessage.cpp
  src/OrganizerBehaviour.cpp
  src/TransporterBehaviour.cpp
)

add_library(${PROJECT_NAME} SHARED ${BEHAVIOURS-SOURCES})
ament_target_dependencies(${PROJECT_NAME} ros2_bdi_interfaces ros2_bdi_utils)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
