# Reproducible build

## Firmware

Open `firmware/USER/stm32_Balance_Car.uvprojx`, select target
`stm32RCT6_Balance_Car`, and rebuild with ARMCC 5.06u7. The project deliberately
uses ARMCC optimization level 1. Its linker load-region maximum is 64 KiB, so a
future oversized image fails at link time instead of producing unsafe firmware.
The command-line full rebuild is:

```bat
"D:\DIP\MDK-ARM\core\UV4\UV4.exe" -r firmware\USER\stm32_Balance_Car.uvprojx -t stm32RCT6_Balance_Car
```

After rebuilding, verify the exact HEX range:

```bat
python test_tools\check_hex_range.py firmware\OBJ\stm32_Balance_Car_L.hex
```

## Android

Use JDK 17, Android platform 35/build-tools 35.0.0, and Gradle 8.10.2:

```bat
cd android
gradlew.bat test assembleDebug
```

The debug APK is created under `android/app/build/outputs/apk/debug`.
The project contains the v1.1 development debug key under `android/signing`
so v1.2 can be installed as an update without removing the previous app.
