This is a modified version of Waveshare's e-Paper library for the 1.54" V2 Display from [here](
    https://github.com/waveshare/e-Paper/blob/master/Arduino/epd1in54_V2)
The functional modifications are:
* Support for changing pin assignment
* Fix compilation error of fonts with newest ESP-IDF
* Added 32 and 48 px fonts
* Remove demo-image to reduce space needs
* Remove *.ino file
* Use human-readable defines instead of raw hex-codes for display command.
