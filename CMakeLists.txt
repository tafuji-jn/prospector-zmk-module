cmake_minimum_required(VERSION 3.20.0)

# FORCE traditional approach to ensure BLE API linking works
# The modern zmk_library approach may not link BLE symbols correctly
zephyr_include_directories(include)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

# Add status advertisement support - EXACTLY like rgbled-widget
if(CONFIG_ZMK_STATUS_ADVERTISEMENT)
        target_sources(app PRIVATE src/status_advertisement.c)
endif()

# Add GATT-based status service (keyboard side)
if(CONFIG_PROSPECTOR_STATUS_GATT_SERVICE)
        target_sources(app PRIVATE src/status_gatt_service.c)
endif()

# Add scanner mode support
if(CONFIG_PROSPECTOR_MODE_SCANNER)
        target_sources(app PRIVATE src/status_scanner.c)
endif()

# BLE Central / USB HID dongle mode
if(CONFIG_PROSPECTOR_DONGLE_MODE)
        target_sources(app PRIVATE src/hid_central.c)
        target_sources(app PRIVATE src/usb_hid_forwarder.c)
        # v18: Fix keyboard's invalid Y coordinate for ECDH.
        # Wrap bt_pub_key_is_valid (skip validation),
        # bt_dh_key_gen (compute correct Y from X before ECDH),
        # mbedtls_ecp_check_pubkey (safety net bypass).
        zephyr_link_libraries(
                -Wl,--wrap=bt_pub_key_is_valid
                -Wl,--wrap=bt_dh_key_gen
                -Wl,--wrap=mbedtls_ecp_check_pubkey
                -Wl,--wrap=bt_crypto_f5
                -Wl,--wrap=bt_crypto_f6
        )
endif()

# DISABLED FOR DEBUG: Custom display/LVGL drivers may be causing boot issues
# Display_test works without these - use Zephyr's built-in drivers instead
# if(CONFIG_SHIELD_PROSPECTOR_ADAPTER OR CONFIG_SHIELD_PROSPECTOR_SCANNER)
#         add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR}/drivers/display)
#         add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR}/modules/lvgl)
# endif()

if(CONFIG_SHIELD_PROSPECTOR_ADAPTER)
        if(CONFIG_DT_HAS_ZMK_BEHAVIOR_CAPS_WORD_ENABLED)
                target_sources(app PRIVATE src/events/caps_word_state_changed.c)
                set_source_files_properties(
                        ${APPLICATION_SOURCE_DIR}/src/behaviors/behavior_caps_word.c
                        TARGET_DIRECTORY app
                        PROPERTIES HEADER_FILE_ONLY ON)
                target_sources(app PRIVATE src/behaviors/behavior_caps_word.c)
        endif()

        target_sources(app PRIVATE src/events/split_central_status_changed.c)
        target_sources(app PRIVATE src/split/bluetooth/central_status_changed_observer.c)
endif()

# Scanner shield sources are handled by boards/shields/prospector_scanner/CMakeLists.txt