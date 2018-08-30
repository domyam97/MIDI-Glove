# MIDI-Glove
This project was an attempt to make a musical instrument you could wear on your hand. This glove consisted had an 9-DOF Inertial Measurement Unit (accelerometer, gyroscope, and magnetometer) and four bend sensors as inputs.

I used Adafruit’s Feather module with a small packet radio onboard as my microcontroller and wireless communications hub and I wrote Arduino code to collect the user’s hand orientation and movements, as well as which fingers were bent. 

The glove transmitted this data to an Arduino Uno with a packet radio receiver which was decoding the data in real time and translating it into MIDI notes. These notes were then sent to a digital audio workstation like Ableton Live or Fruity Loops that allowed these notes to be played on a virtual instrument. 

The angle of the hand determined the pitch of the note while the movements determined its velocity and throw to the left or right. Each finger activated a different MIDI channel which could each be programmed to a different instrument. 

The result was an intuitive device that could control up to four instruments at once in real time. One of the largest
challenges was compressing the gesture data as much as I could to get fast enough transfer rates. I eventually had gotten the entire packet to five bytes.
