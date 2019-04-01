############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


# TODO:
#  NuttX cmake support custom boards
#   library aliases? nuttx::arch, etc
#   apps handle builtin commands
#

add_subdirectory(nuttx)

set(NUTTX_CONFIG_DIR ${PX4_BOARD_DIR}/nuttx-config)

# If the board provides a Kconfig Use it or create an empty one
if(EXISTS ${NUTTX_CONFIG_DIR}/Kconfig)
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/configs/dummy/Kconfig
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_CONFIG_DIR}/Kconfig ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		DEPENDS
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
			${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
		)
else()
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/configs/dummy/Kconfig
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		COMMAND ${CMAKE_COMMAND} -E touch ${NUTTX_DIR}/configs/dummy/Kconfig
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		DEPENDS
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
			${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
		)
endif()

###############################################################################
# NuttX configure
###############################################################################

# copy NuttX config directory
add_custom_command(
	OUTPUT
		${NUTTX_DIR}/arch/arm/include/math.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/include/board.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
		${PX4_BINARY_DIR}/NuttX/nuttx-config/scripts/script.ld
		${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/include/nuttx/config.h
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${NUTTX_CONFIG_DIR}/ ${PX4_BINARY_DIR}/NuttX/nuttx-config
	COMMAND ${CMAKE_COMMAND} -E make_directory ${PX4_BINARY_DIR}/NuttX/nuttx-config/src
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/math.h ${NUTTX_DIR}/arch/arm/include/math.h # copy arm math.h into NuttX source
	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SRC_DIR}/nsh_romfsimg.h ${PX4_BINARY_DIR}/NuttX/nuttx-config/include/nsh_romfsimg.h
	COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
	DEPENDS
		${NUTTX_SRC_DIR}/math.h
		${NUTTX_SRC_DIR}/nsh_romfsimg.h
		${NUTTX_CONFIG_DIR}/include/board.h
		${NUTTX_CONFIG_DIR}/scripts/script.ld
		${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy.stamp
		${CMAKE_CURRENT_BINARY_DIR}/apps_copy.stamp
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Copying NuttX config ${NUTTX_CONFIG}"
)

# NuttX defconfig
#  cmake should trigger reconfigure if defconfig changes
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${NUTTX_DEFCONFIG})
file(STRINGS ${NUTTX_DEFCONFIG} config_expanded REGEX "# Automatically generated file; DO NOT EDIT.")
if (NOT config_expanded)
	# copy compressed PX4 defconfig into nuttx and inflate
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/.config
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
		COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/px4_nuttx_make_olddefconfig.sh ${NUTTX_DIR}
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		DEPENDS
			${NUTTX_DEFCONFIG}
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX compressed config ${NUTTX_CONFIG} and inflating"
	)
else()
	# copy uncompressed PX4 defconfig into nuttx
	add_custom_command(
		OUTPUT
			${NUTTX_DIR}/.config
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
		COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/nuttx_config.stamp
		DEPENDS
			${NUTTX_DEFCONFIG}
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_copy_config_dir.stamp
			${CMAKE_CURRENT_BINARY_DIR}/nuttx_config_kconfig.stamp
		WORKING_DIRECTORY ${NUTTX_DIR}
		COMMENT "Copying NuttX uncompressed config ${NUTTX_CONFIG}"
	)
endif()

###############################################################################
# NuttX oldconfig
add_custom_target(oldconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y oldconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make oldconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + savedefconfig back to PX4
add_custom_target(oldconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS oldconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make oldconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + copy uncompressed back to PX4
add_custom_target(oldconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS oldconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make oldconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX olddefconfig
add_custom_target(olddefconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y olddefconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make olddefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX olddefconfig + savedefconfig back to PX4
add_custom_target(olddefconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS olddefconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make olddefconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX oldconfig + copy uncompressed back to PX4
add_custom_target(olddefconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS olddefconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make olddefconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX menuconfig
add_custom_target(menuconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y menuconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make menuconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX menuconfig + savedefconfig back to PX4
add_custom_target(menuconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS menuconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make nuttx_menuconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX menuconfig + copy uncompressed back to PX4
add_custom_target(menuconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS menuconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make nuttx_menuconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

###############################################################################
# NuttX qconfig
add_custom_target(qconfig_nuttx
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y qconfig
	DEPENDS ${NUTTX_DIR}/.config
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running NuttX make qconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
	)

# NuttX qconfig + savedefconfig back to PX4
add_custom_target(qconfig
	COMMAND make --no-print-directory --silent -C ${NUTTX_DIR} CONFIG_ARCH_BOARD_CUSTOM=y savedefconfig
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/defconfig ${NUTTX_DEFCONFIG}
	COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_DIR}/.config
	DEPENDS qconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make qconfig then savedefconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)

# NuttX qconfig + copy uncompressed back to PX4
add_custom_target(qconfig_uncompressed
	COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_DIR}/.config ${NUTTX_DEFCONFIG}
	DEPENDS qconfig_nuttx
	WORKING_DIRECTORY ${NUTTX_DIR}
	COMMENT "Running make qconfig then saving uncompressed defconfig for ${NUTTX_CONFIG}"
	USES_TERMINAL
)
