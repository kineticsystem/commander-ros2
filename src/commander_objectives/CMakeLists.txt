cmake_minimum_required(VERSION 3.16)
project(commander_objectives)

find_package(ament_cmake REQUIRED)

# The objectives, and the subtrees they are built from, are plain XML files:
# install them where the commander server can find them. Both folders are
# listed in the `behavior_trees` parameter of the server.
#
#   objectives  what a client can ask for by name
#   subtrees    the reusable building blocks the objectives are made of
install(DIRECTORY objectives subtrees DESTINATION share/${PROJECT_NAME}/)

ament_package()
