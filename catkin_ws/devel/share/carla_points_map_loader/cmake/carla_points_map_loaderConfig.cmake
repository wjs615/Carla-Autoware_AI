# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(carla_points_map_loader_CONFIG_INCLUDED)
  return()
endif()
set(carla_points_map_loader_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(carla_points_map_loader_SOURCE_PREFIX /home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/src/carla_points_map_loader)
  set(carla_points_map_loader_DEVEL_PREFIX /home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/devel)
  set(carla_points_map_loader_INSTALL_PREFIX "")
  set(carla_points_map_loader_PREFIX ${carla_points_map_loader_DEVEL_PREFIX})
else()
  set(carla_points_map_loader_SOURCE_PREFIX "")
  set(carla_points_map_loader_DEVEL_PREFIX "")
  set(carla_points_map_loader_INSTALL_PREFIX /home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/install)
  set(carla_points_map_loader_PREFIX ${carla_points_map_loader_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'carla_points_map_loader' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(carla_points_map_loader_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(carla_points_map_loader_INCLUDE_DIRS "")
  set(_include_dirs "")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'CARLA Simulator Team <carla.simulator@gmail.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${carla_points_map_loader_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'carla_points_map_loader' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'carla_points_map_loader' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/src/carla_points_map_loader/${idir}'.  ${_report}")
    endif()
    _list_append_unique(carla_points_map_loader_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND carla_points_map_loader_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND carla_points_map_loader_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT carla_points_map_loader_NUM_DUMMY_TARGETS)
      set(carla_points_map_loader_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::carla_points_map_loader::wrapped-linker-option${carla_points_map_loader_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR carla_points_map_loader_NUM_DUMMY_TARGETS "${carla_points_map_loader_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::carla_points_map_loader::wrapped-linker-option${carla_points_map_loader_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND carla_points_map_loader_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND carla_points_map_loader_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND carla_points_map_loader_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/siruu/Desktop/ros_ws/carla-autoware/catkin_ws/devel/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ymc/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/xsens_driver/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/wf_simulator/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lattice_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/waypoint_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/waypoint_maker/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/way_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vlg22c_cam/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vision_ssd_detect/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vision_segment_enet_detect/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vision_lane_detect/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vision_darknet_detect/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vision_beyond_track/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vel_pose_diff_checker/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vehicle_socket/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vehicle_sim_model/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vehicle_model/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vehicle_gazebo_simulation_launcher/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vehicle_gazebo_simulation_interface/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vehicle_engage_panel/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vehicle_description/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/trafficlight_recognizer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_waypoint_follower/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_utilities/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_simulation_package/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_local_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_global_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_direct_controller/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_kf_contour_track/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_ros_helpers/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ff_waypoint_follower/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/dp_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_simu/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_gnss_localizer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/op_utility/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_euclidean_cluster_detect/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vector_map_server/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/road_occupancy_processor/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/costmap_generator/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/object_map/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/naive_motion_predict/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lanelet_aisan_converter/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/map_file/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/libvectormap/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lane_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/imm_ukf_pda_track/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/decision_maker/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vector_map/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vector_map_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/vectacam/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/udon_socket/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/twist_generator/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/twist_gate/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/twist_filter/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/twist2odom/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_point_pillars/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/tvm_utility/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/tablet_socket/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/runtime_manager/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/mqtt_socket/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/tablet_socket_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/system_monitor/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/state_machine_lib/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/sound_player/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/sick_lms5xx/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/sick_ldmrs_tools/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/sick_ldmrs_driver/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/sick_ldmrs_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/sick_ldmrs_description/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/points2image/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/rosinterface/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/rosbag_controller/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/pure_pursuit/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/points_preprocessor/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/mpc_follower/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_localizer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/emergency_handler/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_health_checker/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/as/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ros_observer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/roi_object_filter/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/range_vision_fusion/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/pos_db/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/points_downsampler/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/pixel_cloud_fusion/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/pcl_omp_registration/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/pc2_downsampler/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/oculus_socket/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/obj_db/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/nmea_navsat/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ndt_tku/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ndt_cpu/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/multi_lidar_calibrator/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/microstrain_driver/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/memsic_imu/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/marker_downsampler/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/map_tools/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/map_tf_generator/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/log_tools/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ll2_global_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_shape_estimation/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_naive_l_shape_detect/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_fake_perception/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lidar_apollo_cnn_seg_detect/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/libwaypoint_follower/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lgsvl_simulator_bridge/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/lanelet2_extension/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/kvaser/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/kitti_launch/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/kitti_player/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/kitti_box_publisher/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/javad_navsat_driver/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/integrated_viewer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/image_processor/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/hokuyo/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/graph_tools/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/gnss_localizer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/gnss/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/glviewer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/gazebo_world_description/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/gazebo_imu_description/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/gazebo_camera_description/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/garmin/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/freespace_planner/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/fastvirtualscan/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ekf_localizer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ds4_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/ds4_driver/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/detected_objects_visualizer/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/decision_maker_panel/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/data_preprocessor/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/custom_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/carla_autoware_bridge/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/calibration_publisher/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_system_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_rviz_plugins/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_pointgrey_drivers/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_driveworks_interface/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_connector/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_camera_lidar_calibrator/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/astar_search/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/amathutils_lib/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_map_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_launcher_rviz/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_launcher/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_lanelet2_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_external_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_driveworks_gmsl_interface/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_config_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_can_msgs/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_build_flags/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/autoware_bag_tools/lib;/home/siruu/Desktop/ros_ws/autoware.ai.openplanner/install/adi_driver/lib;/home/siruu/catkin_ws/devel/lib;/opt/ros/melodic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(carla_points_map_loader_LIBRARY_DIRS ${lib_path})
      list(APPEND carla_points_map_loader_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'carla_points_map_loader'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND carla_points_map_loader_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(carla_points_map_loader_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${carla_points_map_loader_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "rospy")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 carla_points_map_loader_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${carla_points_map_loader_dep}_FOUND)
      find_package(${carla_points_map_loader_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${carla_points_map_loader_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(carla_points_map_loader_INCLUDE_DIRS ${${carla_points_map_loader_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(carla_points_map_loader_LIBRARIES ${carla_points_map_loader_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${carla_points_map_loader_dep}_LIBRARIES})
  _list_append_deduplicate(carla_points_map_loader_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(carla_points_map_loader_LIBRARIES ${carla_points_map_loader_LIBRARIES})

  _list_append_unique(carla_points_map_loader_LIBRARY_DIRS ${${carla_points_map_loader_dep}_LIBRARY_DIRS})
  list(APPEND carla_points_map_loader_EXPORTED_TARGETS ${${carla_points_map_loader_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${carla_points_map_loader_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
