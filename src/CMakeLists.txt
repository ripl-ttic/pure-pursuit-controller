add_definitions(
    #-ggdb3
    -std=gnu99
    )

set(pp_controller_sources
  main.c
  mp_control_utils.c
  mp_prediction.c
  )

add_executable(pp-controller ${pp_controller_sources})

pods_use_pkg_config_packages(pp-controller
  gthread-2.0
  lcm bot2-core
  bot2-param-client
  bot2-lcmgl-client
  lcmtypes_bot2-core
  lcmtypes_rrtstar
  lcmtypes_obstacle-detector
  lcmtypes_robot-status
  lcmtypes_ripl-general
  check-gridmap
  )

pods_install_executables(pp-controller)
