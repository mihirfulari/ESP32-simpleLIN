# ESP32-simpleLIN
A simple emulated implementation of the LIN bus protocol. This uses the UART hardware stack along with software bit-banged break field generation to fully emulate the LIN protocol.
This library is more reliable compared to an emulated software LIN library because this uses the UART hardware stack for half of the implementation, relieving the CPU of continuous bit-banging.
This library is tested for:
- ESP32-S3.

If you test this library successfully on any other microcontroller, please mail to mihirfulari15@gmail.com or add a pull request with your own implementation.
