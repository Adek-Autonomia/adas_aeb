#include "adas_aeb/tof_aeb/setup_port.h"

/**
 * @brief Set everything necessary to handle port operations
 * 
 */
void Setup::SetEverything()
{
    serial_port = open("/dev/ttyACM0", O_RDWR);

    //Check for errors
    if (serial_port < 0){
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;

    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tchetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; //clear parity bit
    tty.c_cflag &= ~CSTOPB; //one stop bit
    tty.c_cflag &= ~CSIZE; //clear all size bits
    tty.c_cflag |= CS8; //8 bits per byte
    tty.c_cflag &= ~CRTSCTS; //Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; //Turn on READ & ignore ctrl lines

    tty.c_lflag &= ICANON; //Read every byte, not lines separated by /n
    tty.c_lflag &= ~ECHO; //Disable echo
    tty.c_lflag &= ~ECHOE; //Disable erasue
    tty.c_lflag &= ~ECHONL; //Disable new-line echo
    tty.c_lflag &= ~ISIG; //Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); //Turn off s/w flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); //Disable any special handling

    tty.c_oflag &= ~OPOST; //No special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; //Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 40; //Wait up to VTIME s, returning as soon as any data is received
    tty.c_cc[VMIN] = 0; // Wait until programm receives VMIN characters;

    //Baud rate 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

/**
 * @brief Reads a single line of input
 * 
 * @return char* of read characters
 */
char* Setup::ReadInput() {

    int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

    if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
        return read_buf;
    }

    printf("Read %i bytes. Received message: %s", num_bytes, read_buf);

    return read_buf;
}