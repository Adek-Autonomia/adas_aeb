#include <fcntl.h> //file controls
#include <errno.h> //error functions
#include <termios.h> //POSIX terminal control
#include <unistd.h> //write, read, close functions
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <string>

#pragma once

/*
* @brief Class used to handle all of COM port communication
*
* @param 
*/
class Setup {
    private:
        int serial_port;
        char read_buf[256];
    public:

        int SetEverything();
        char* ReadInput();

};