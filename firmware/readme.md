This firmware is based on the firmware for the EmonTx V4. Though its a little more complicated to flash.

Due to the use of optocouplers for safe isolation of the FTDI port the maximum usable baud rate is 9600
This requires the use of a custom bootloader and makes uploading sketches slightly more complicated.
You can find more details in EmonMicroCM.ino