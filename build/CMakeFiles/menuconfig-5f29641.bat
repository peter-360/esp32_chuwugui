cd /D D:\0_xitian\software\ESP32\0_code\uart_echo\build || exit /b
D:\.espressif\python_env\idf4.0_py3.7_env\Scripts\python.exe D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/tools/kconfig_new/confgen.py --kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/Kconfig --sdkconfig-rename D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/sdkconfig.rename --config D:/0_xitian/software/ESP32/0_code/uart_echo/sdkconfig --env-file D:/0_xitian/software/ESP32/0_code/uart_echo/build/config.env --env IDF_TARGET=esp32 --dont-write-deprecated --output config D:/0_xitian/software/ESP32/0_code/uart_echo/sdkconfig || exit /b
D:\.espressif\tools\cmake\3.13.4\bin\cmake.exe -E env "COMPONENT_KCONFIGS=D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/app_trace/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/bt/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/driver/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/efuse/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp-tls/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp32/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_adc_cal/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_common/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_eth/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_event/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_gdbstub/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_http_client/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_http_server/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_https_ota/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_https_server/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esp_wifi/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/espcoredump/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/fatfs/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/freemodbus/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/freertos/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/heap/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/libsodium/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/log/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/lwip/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/mbedtls/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/mdns/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/mqtt/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/newlib/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/nvs_flash/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/openssl/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/pthread/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/spi_flash/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/spiffs/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/tcpip_adapter/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/unity/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/vfs/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/wear_levelling/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/wifi_provisioning/Kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/wpa_supplicant/Kconfig" "COMPONENT_KCONFIGS_PROJBUILD=D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/app_update/Kconfig.projbuild D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/bootloader/Kconfig.projbuild D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/esptool_py/Kconfig.projbuild D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/components/partition_table/Kconfig.projbuild" IDF_CMAKE=y KCONFIG_CONFIG=D:/0_xitian/software/ESP32/0_code/uart_echo/sdkconfig IDF_TARGET=esp32 D:/.espressif/tools/mconf/v4.6.0.0-idf-20190628/mconf-idf.exe D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/Kconfig || exit /b
D:\.espressif\python_env\idf4.0_py3.7_env\Scripts\python.exe D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/tools/kconfig_new/confgen.py --kconfig D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/Kconfig --sdkconfig-rename D:/0_xitian/software/ESP32/0_code/esp-idf-v4.0.1/sdkconfig.rename --config D:/0_xitian/software/ESP32/0_code/uart_echo/sdkconfig --env-file D:/0_xitian/software/ESP32/0_code/uart_echo/build/config.env --env IDF_TARGET=esp32 --output config D:/0_xitian/software/ESP32/0_code/uart_echo/sdkconfig || exit /b
