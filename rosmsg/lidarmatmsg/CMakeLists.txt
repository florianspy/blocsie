cmake_minimum_required(VERSION 3.5)

project(lidarmatmsg)
  
# Get the information about this package's buildtime dependencies
  find_package(catkin REQUIRED
    COMPONENTS message_generation std_msgs)

  # Declare the message files to be built
  add_message_files(FILES
    Scanmat.msg
    Cammat.msg
  )


  # Declare the service files to be built


  # Actually generate the language-specific message and service files
  generate_messages(DEPENDENCIES std_msgs)

  # Declare that this catkin package's runtime dependencies
  catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs 
  )







