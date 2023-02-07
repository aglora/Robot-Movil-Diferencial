# ServoTimer1
light weight, low cpu overhead, servo driver using atmega count/timer1

This library uses the atmega 16 bit counter/timer1 to calculate PWM with low cpu overhead. the library only gets interrupts
when a channel needs to change its output to match the specified frequencey.
        
<pre>   interrupt     interrupt                  
      |             |                       
      \/            \/                                                                                                
______|`````|_________________________      //channel 1                                             
____________|```````|_________________      //channel 2                                             
____________________|```|_____________      //channel 3                                             
            ^           ^
            |           |
          interrupt    interrupt</pre>
          
for 3 channels there are only 4 interrupts so the total interrupts is 1+ the total channels. each interrupt by defualt takes
about 8 clock cycles, 4 cycles entering and 4 cycles exiting. each interrupt handle requires about 12 clock cycles to complete
the handle giving a total of about 20 cpu cycles per interrupt so 4 channels would equal 5*20 cycles per a 20000us interval.
with a clock rate of 16MHz that about 6.25us per a 20000us interval 0.03% of the CPU's time.
