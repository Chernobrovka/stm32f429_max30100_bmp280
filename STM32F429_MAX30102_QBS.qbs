import qbs
import qbs.FileInfo
import qbs.ModUtils

CppApplication {

    name: "STM32F429_MAX30102_QBS"        // The name of the output file (without the suffix, it depends on the purpose)

        type: [
            "application",
            "bin",
            "hex",               // Type - application, i.e. executable file.
        ]

    Depends {
            name: "cpp"         // This product depends on the C ++ compiler
        }

    consoleApplication: true
    cpp.positionIndependentCode: false
    cpp.executableSuffix: ".elf"

    property string Home: "D:/Projects/Qt/Uart_ringbuffer_FIFO"
    property string HAL: Home + "/Drivers/STM32F4xx_HAL_Driver"
    property string CMSIS: Home + "/Drivers/CMSIS"
    property string Inc: Home + "/Core/Inc"
    property string Src: Home + "/Core/Src"
    property string startup: Home + "/Core/Startup"

    Group {
        name: "HAL"
        files: [
            HAL + "/Src/*.c",
            HAL + "/Inc/*.h",
        ]
    }

    Group {
        name: "CMSIS"
        files: [
            CMSIS + "/Include/*.h",
            CMSIS + "/Device/ST/STM32F4xx/Source/Templates/*",
            CMSIS + "/Device/ST/STM32F4xx/Include/*.h",
        ]
    }

    Group {
        name: "Inc"
        files: [
            Inc + "/*.h",
        ]
    }

    Group {
        name: "Src"
        files: [
            Src + "/*.c",
        ]
    }

    Group {
        name: "startup"
        files: [
            startup + "/*.s",
        ]
    }

    Group {
        name: "LD"
        files: [
            Home + "/*.ld",
        ]
    }

    cpp.includePaths: [
        CMSIS + "/Include",
        CMSIS + "/Device/ST/STM32F4xx/Include",
        Inc,
        HAL + "/Inc",
    ]

    cpp.defines: [
            "USE_HAL_DRIVER",
            "STM32F429xx",
            "__weak=__attribute__((weak))",
            "__packed=__attribute__((__packed__))",
    ]

    cpp.commonCompilerFlags: [
        "-mcpu=cortex-m4",
        "-mthumb",
    ]

    cpp.driverFlags: [
        "-mcpu=cortex-m4",
        "-mthumb",
        "-Xlinker",
        "--gc-sections",
        "-specs=nosys.specs",
        "-specs=nano.specs",
        "-Wl,-Map=" + path + "QT_Uart_FIFO.map",
    ]

    cpp.linkerFlags: [
        "--start-group",
        "-T" + path + "/STM32F429ZITX_FLASH.ld",
    ]

    Properties {
        condition: qbs.buildVariant === "debug"
        cpp.debugInformation: true
        cpp.optimization: "none"
    }

    Properties {
        condition: qbs.buildVariant === "release"
        cpp.debugInformation: false
        cpp.optimization: "small"        // Types of optimizations: "none", "fast", "small"
    }

    Properties {
        condition: cpp.debugInformation
        cpp.defines: outer.concat("DEBUG")
    }

    Group {
        // Properties for the produced executable
        fileTagsFilter: product.type
        qbs.install: true
    }

    // Create a .bin file
    Rule {
        id: binDebugFrmw
        condition: qbs.buildVariant === "debug"
        inputs: ["application"]

        Artifact {
            fileTags: ["bin"]
            filePath: input.baseDir + "/" + input.baseName + ".bin"
        }

        prepare: {
            var objCopyPath = "arm-none-eabi-objcopy"
            var argsConv = ["-O", "binary", input.filePath, output.filePath]
            var cmd = new Command(objCopyPath, argsConv)
            cmd.description = "converting to BIN: " + FileInfo.fileName(
                        input.filePath) + " -> " + input.baseName + ".bin"

            // Write to the nor memory by qspi
            var argsFlashingQspi =
                    [
                     // "-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/interface/stlink-v2-1.cfg",
                     "-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/board/st_nucleo_f4.cfg",
                     //"-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/target-st/stm32f4x.cfg",
                     "-c", "init",
                     "-c", "reset init",
                     //"-c", "flash write_bank 1 " + output.filePath + " 0",
                     "-c", "flash write_bank 0 " + output.filePath + " 0",
                     "-c", "reset",
                     "-c", "shutdown"
                    ]

            var cmdFlashQspi = new Command("openocd", argsFlashingQspi);
            cmdFlashQspi.description = "Wrtie to the NOR QSPI"

            // Write to the internal memory
            var argsFlashingInternalFlash =
                    [
                     // "-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/interface/stlink-v2-1.cfg",
                     "-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/board/st_nucleo_f4.cfg",
                     //"-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/target-st/stm32f4x.cfg",
                     "-c", "init",
                     "-c", "reset init",
                     //"-c", "flash write_image erase " + input.filePath + " 0x08006978 elf",
                     "-c", "flash write_image erase " + input.filePath,
                     "-c", "reset",
                     "-c", "shutdown"
                   ]

            var cmdFlashInternalFlash = new Command("openocd", argsFlashingInternalFlash);
            cmdFlashInternalFlash.description = "Wrtie to the internal flash";

            return [cmd, cmdFlashQspi, cmdFlashInternalFlash] // cmdFlashQspi, cmdFlashInternalFlash
        }
    }

    Rule {
           id: binFrmw
           condition: qbs.buildVariant === "release"
           inputs: ["application"]

           Artifact {
               fileTags: ["bin"]
               filePath: input.baseDir + "/" + input.baseName + ".bin"
           }

           prepare: {
               var objCopyPath = "arm-none-eabi-objcopy"
               var argsConv = ["-O", "binary", input.filePath, output.filePath]
               var cmd = new Command(objCopyPath, argsConv)
               cmd.description = "converting to BIN: " + FileInfo.fileName(
                           input.filePath) + " -> " + input.baseName + ".bin"

               // Запись в nor память по qspi
               // Write to the nor memory by qspi
               var argsFlashingQspi =
               [           "-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/board/st_nucleo_f4.cfg",
                           "-c", "init",
                           "-c", "reset init",
                           "-c", "flash write_bank 0 " + output.filePath + " 0",
                           "-c", "reset",
                           "-c", "shutdown"
               ]

               var cmdFlashQspi = new Command("openocd", argsFlashingQspi);
               cmdFlashQspi.description = "Wrtie to the NOR QSPI"

               // Запись во внутреннюю память
               // Write to the internal memory
               var argsFlashingInternalFlash =
               [           "-f", "D:/Programs/OpenOCD-20230202-0.12.0/share/openocd/scripts/board/st_nucleo_f4.cfg",
                           "-c", "init",
                           "-c", "reset init",
                           "-c", "flash write_image erase " + input.filePath,
                           "-c", "reset",
                           "-c", "shutdown"
               ]

               var cmdFlashInternalFlash = new Command("openocd", argsFlashingInternalFlash);
               cmdFlashInternalFlash.description = "Wrtie to the internal flash"

               return [cmd, cmdFlashQspi, cmdFlashInternalFlash]
           }
       }

    Rule {
        id: hexFrmw
        condition: qbs.buildVariant === "release"
        inputs: ["application"]

        Artifact {
            fileTags: ["hex"]
            filePath: input.baseDir + "/" + input.baseName + ".hex"
        }

        prepare: {
            var objCopyPath = "arm-none-eabi-objcopy"
            var argsConv = ["-O", "ihex", input.filePath, output.filePath]
            var cmd = new Command(objCopyPath, argsConv)
            cmd.description = "converting to HEX: " + FileInfo.fileName(
                        input.filePath) + " -> " + input.baseName + ".hex"

            return [cmd]
        }
    }
}
