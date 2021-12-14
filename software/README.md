Connect DAC1/1 to ADC1/5. DAC1/1 outputs a somewhat sinusoidal signal and it is read with the ADC1/1. The signal is processed using the CMSIS DSP libraries. Data is sent to the computer via USART (usb cable serial) with a baud rate of 115200. Use Putty or similar tools to read the signal and transfer it to matlab.

Usage: Build all and hope for the best
