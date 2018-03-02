# Open-Air Acoustic Delay-line Memory using a uC

![Assembled prototype board used for the experiment](https://github.com/mickymuis/delayline/blob/master/images/assembled.jpg "Assembled prototype")

This project tries to reconstruct [Delay Line Memory](https://en.wikipedia.org/wiki/Delay_line_memory) using a micro-controller (e.g. Arduino). Main inspriration is drawn from the work by [Joseph Allen](http://jhallenworld.blogspot.nl/2014/01/acoustic-delay-line-memory.html) and [Stephen Cass](https://spectrum.ieee.org/geek-life/hands-on/build-a-delayline-memory-out-of-mostly-thin-air). If you want more theoretical information about delay lines, I recommend reading [1] by Auerbach and of course my own essay that you can find in the ```essay/``` folder of this repository.

# Concept

A delay-line memory stores bits of information by encoding them with acoustic pulses in some medium, in this case: air. These pulses are generated between two transducers: a speaker on the transmitting end and a microphone on the receiving end. Say we have the ability to detect a pulse every 0.6 ms. Sound travels roughly 20 cm in this time, so we take a waveform with a wave length of 20 cm. If we take the speaker and microphone to be one meter apart, we can build a pipeline of five of these pulses resulting in a 5-bit storage. As soon as a bit is detected by the microphone/micro-controller, we immediately send it back through the speaker. This generates an infinite cycle that can be compared to the refresh cycle of [DRAM](https://en.wikipedia.org/wiki/Dynamic_random-access_memory). So now at any time 5 bits float in the air and one is always in the device itself (remember we have to storage this bit in the device to be able transmit it again), resulting in a total capacity of (an impressive) 6 bits.

We call the above method a *bit serial operation*, because all bits are injected sequentially into the medium (air) creating a one-dimensional array. Another idea is to actually create a two-dimensional array by adding multiple sine waves together for each pulse. These sines can be differentiated by the receiving end using [Fourier analysis](https://en.wikipedia.org/wiki/Fourier_analysis). While this method is much more complex (and demand a very fast micro-controller), it could potentially store a two-dimonsional piece of data, say an image, in thin air!

# Getting started

In this repository I share my progress on implementing delay-lines on prototype boards. The code you will find is written for the Atmel SAMD21, an ARM Cortex M0+ processor. It can be found on the Arduino Zero or M0+ and many others, including the M0+ from Adafruit. All boards should give you the same results.

The connection schematic is very simple: the speaker is connected through an amp (PAM8403) to DAC0. Other amplifiers will do as well. The microphone I use is amplified by the MAX4466 chip (I use the one from [Adafruit](https://www.adafruit.com/product/1063)) and is connected to pin PA5 (Arduino calls this one A4). I added some high-pass filters (around 800 Hz) to protect the speaker. You can change it according to your speaker or just leave it out. I placed all components on a 'shield' (see image) that fits the Arduino Zero, but you could also use a breadboard (note that the Adafruit M0+ is much more manageble that the Arduino, connection-wise. It's also even cheaper.).

# Source code

Currently there are two different programs you can try. ```delayline_samd21.ino``` implements a real serial delay-line memory. It uses the serial interface from Arduino to show you the contents of the memory and lets you change parameters and flip the bits using the same interface. You can calibrate it by sending single characters over the serial interface:
* 'a' and 'z' increase/decrease the number of bits. This depends on the distance between the speaker and microphone.
* 's' and 'x' increase/decrease the amplitude treshold of the detected signal. A high treshold with a large volume gives a better SNR, but also kills your ears.
* 'd' and 'c' increase/decrease the wave length of the pulses. Long wave lengths tend to be easier to detect, but less bits fit in the same space.
* Any number i to flip the i-th bit. 
It can be a hassle to get it stable, even the slightest movement may throw it into chaotic oscillation. I have come as far as 5 bits in roughly 3 meters.

The file ```fftloop_samd21.ino``` contains some preliminary work on the parallel operating mode. It uses the [CMSIS-DSP](http://www.keil.com/pack/doc/cmsis/DSP/html/index.html) library to do hardware FFT on the M0+ and software [DDS](http://www.radio-electronics.com/info/rf-technology-design/pll-synthesizers/direct-digital-synthesizer-dds-tutorial.php) to produce the sine waves. Currently, it is hard-coded to create a 1x5 bit array, which is not technically a delay-line memory. It makes far less noise than the other program, though. 

# Feedback
I am really interested in your feedback, even if you did not try one of my demos on your Arduino/Feather/etc. Let me know what you think!

# References
[1] I. L. Auerbach, J. P. Eckert, R. F. Shaw and C. B. Sheppard, *Mercury Delay Line Memory Using a Pulse Rate of Several Megacycles,* in Proceedings of the IRE, vol. 37, no. 8, pp. 855-861, Aug. 1949.
