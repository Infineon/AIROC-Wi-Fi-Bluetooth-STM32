Project setup in IDE
1. Check if following preprocessor defines in IDE settings exists if no than add for CM7:
  - CY_USING_HAL
  - CYBSP_WIFI_CAPABLE
  - HAVE_SNPRINTF
  - CY_STORAGE_WIFI_DATA=".whd_fw"
   This must to be added for Debug and Realese.
  
2. In STm32CubeMxIde press "Convert to C++" to switch project to C++ for CM7. 

3. Define section ".whd_fw" in linker script for CM7. 
    -If you using STM32CubeIDE copy fully file STM32H747XIHX_FLASH.ld from packs and add it to current project file

    -If you using EWARM copy only:
define symbol __ICFEDIT_region_RAM_start__     = 0x20000000;
define symbol __ICFEDIT_region_RAM_end__       = 0x2001FFFF;

 from file stm32h747xx_flash_CM7.icf from packs and add it to current project file


