# ARD_FlightStick
******Disclaimer:
Use this code on your own risk, check that it does what you want it to do.
Not responsible for any injuries/accidents of any kind.
***There are some known safty critical issues in the current vrsion so, plase, take the time and read the released notes document.

Detailed Project information:
This project is absed on an arduino nano board that should be used for RC usage only!
* Supports up to 4 analog inpus (Ail,Elev,Rud,Thr) and up to 12(4 of them are occupied as axis trimmers)  multiplexed digital inputs for press buttons.
   The SW distinguish between short and long button press.
* Support Head tracker input (using interrupts) currently for only one axis (Comecial/DIY headtracker tat generate PPM output) but, can be easly expended to support more.
* Out puts a PPM signal that can be :
  1. Connected to your exist RC Tranmitter trainer input As a wired "Buddy box" 
  2. Connected to your exist RC Tranmitter trainer input As a wireless "Buddy box" (connect the output PPM signal to any PPM input RC TX module - and bind i to a PPM out RC RX          then connect the output PPM sinal from th RX to your RC transmitter PPM input (just like he wired "buddy box is connected) for exmaple watch the folowing movie:
     https://youtu.be/Y_BxH1m_36o
  3.Connect the PPM output from the arduino to any PPM input RC TX module and bind it to your model directly, for example watch the following movies:
    https://youtu.be/RK5dqsT2C_o
    https://youtu.be/BuTpP5eRi34
    *Warnng: here are no expos or dual rates implemented in the code so the controls might be very sensitive.
  
  4.Easy calibration process - while holding 3 specific buttons combination while turning the power on, the code enters axis calibration mode and after rotating all axis               (Ail,Elev,Rud,Thr) to their limits and pressing a button, the axis limit will be stored in the arduino's inernal Non-volatile memory (EEPROM memory) and will be used as         the axis limits every time the flight stick is turrned on.
  5. The tirmms inputs are also saved within the intrnal Non-volatile memory (EEPROM memory).

  Feel free to conat me for qustion and suggestion or........ implement them yourself :)
  **Never forget to remove your props before tasting anthing new !!!!!!!!!!!!!!!!
  *Happy and safty flying !!!!!!!!


  






