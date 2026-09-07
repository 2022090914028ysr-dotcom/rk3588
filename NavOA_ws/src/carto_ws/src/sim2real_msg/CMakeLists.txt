cmake_minimum_required(VERSION 3.0.2)
project(sim2real_msg)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(FILES KidnapStatus.msg)

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
