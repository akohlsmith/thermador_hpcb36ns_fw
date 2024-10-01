# Thermador HPCB36NS Replacement Control Board

This repo contains the STM32G474 source code for a replacement PCB I designed for the Thermador HPCB36NS/01 range hood. I could find the wiring diagram for the fan module but unfortunately the diagram did not include any explantion of how the control signals worked. The original control PCB had already been replaced once a couple of years earlier and for the same failure, and I was not looking at replacing it again, as the PCB was becoming difficult to source and even when it was available, it cost $300.

My original request for help on understanding how the fan module operated was posted on [Reddit](https://www.reddit.com/r/appliancerepair/comments/17rnkx2/thermador_range_hood_control_board_failed_just/) around September 2023. Unfortunately since the control board was failing I could not take direct measurements of how it operated its control outputs. I could, however make some estimated guesses.

I ended up designing my own control board, intended to be a straight-up replacement of the original PCB. Through some incredible failure on my part around the reading a damned ruler, my PCB ended up being too large to be a simple replacement. I ended up cutting some components off of the original board so that it was just a correctly-fitting "dummy" and wiring the signals from the original PCB to my own PCB. Ugly, but functional. Story of my life. :-)

# Theory of Operation

There are five outputs on the original control board which drive the fan module and lights. These signals were all driven directly by a Microchip PIC processor on the original PCB, and I could see that the original PCB was powered by 5V, including the PIC, so I knew that the outputs would have to be compatible with the PIC I/O structure (i.e. 5V and up to 20mA total current draw). There is also a thermistor which was used for the "automatic" fan function which I never used, so I left that off entirely. I could theoretically use the STM32's internal die temperature diode for a similar feature, but that's not something I'm planning on ever doing. Similarly, there is also a run timer which turns on the "filter" LED to indicate that the range hood filter should be cleaned, but I also don't use that feature.

Since I wasn't 100% sure how these needed to be driven, I elected to use a pair of 74LS125 buffer/driver ICs which would serve three functions: provide 3.3V to 5V level translation, do the "heavy lifting" in case the output signals needed to drive heavier loads than the STM32 might be able to do on its own, and finally, to act as "sacrificial" components in case something went wrong.

How the outputs were used ended up being pretty simple: Three of the outputs select the fan speed. Only one output is active at a time. The last two outputs are for the range hood lights, with one output being active for "high" and the other for "low" illumination. All of the outputs are push-pull (i.e. they output 5V when "active" and 0V when "inactive").

## Switches and Outputs

The firmware is set up in a way that allows me to easily change the function of any switch or output. It's perhaps a little over-engineered but that's the way I like it. The buttons and outputs can be remapped by editing the `switches` or `outputs` arrays at the top of `main.c`, without any change requred in the rest of the code. This was handy during the initial figuring out of which output mapped to which fan or light selection.

## Periperhal Initialization

TIM2 is used as a differential PWM timer for the beeper, and TIM15 is used for LED dimming (a feature I never really used). I do set up ADC1 for the temperature sensor as explained above, but that feature is unused, and USART1 is set up as a debug UART which was helpful during development. The GPIO is then initialized according to the `switches` and `outputs` arrays (see above). The initial system state (fans off, lights off) is set and that's about it for initialization. 

## Main Loop

The main loop is very simple: look at each switch using `check_switch()` and delay for 50msec before doing it all again. That's it. The `check_switch()` function debounces a switch and, if its state has settled on a logic low (i.e. it's being pressed) then execute the function associated with that specific switch. For this firmware, that means updating the outputs and beeping.

## Power Saving

I have a `LOW_POWER` macro that, if defined, puts the STM32 to sleep for the 50ms delay but I wasn't able to get it working under the time constraint I had, so I just left it running at full power. Since I leave the range hood lights on pretty much 24/7 I didn't think the few mA of power draw the STM32 had was going to make much of a difference, and the small LDO I'm using to power the STM32 didn't seem to have any trouble either, so there wasn't much incentive to try to get power savings to work.
