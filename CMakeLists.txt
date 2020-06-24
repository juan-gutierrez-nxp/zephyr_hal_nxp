if(CONFIG_HAS_MCUX OR CONFIG_HAS_IMX_HAL)
  zephyr_library()
endif()

add_subdirectory_ifdef(
  CONFIG_HAS_MCUX
  mcux
  )

add_subdirectory_ifdef(
  CONFIG_HAS_IMX_HAL
  imx
  )

add_subdirectory_ifdef(
  CONFIG_HAS_S32R45_HAL
  s32r45
  )
