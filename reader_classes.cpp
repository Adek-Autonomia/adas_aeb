#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>
#include <vector>
#include <fcntl.h> //file controls
#include <errno.h> //error functions
#include <termios.h> //POSIX terminal control
#include <unistd.h> //write, read, close functions

/**
* @brief Tokenize a given string
*
* @param input_s String to be tokenized
* @param delimiter Characted used to divide given string (by default ",")
*/
class Tokenizer {

    private:
        std::string input;
        std::string del;


    public:
        Tokenizer(std::string input_s = "", std::string delimiter = ",") {
            input = input_s;
            del = delimiter;
        }

/**
 * @brief Tokenizes a string passed to the class
 *  
 * @return std::vector<std::string> 
 */
        std::vector<std::string> tokenize(){
            std::vector<std::string> substrings;
            int start = 0;
            int end = input.find(del);
            while (end != -1) {
                substrings.push_back(input.substr(start, end - start));
                start = end + del.size();
                end = input.find(del, start);
            }
            substrings.push_back(input.substr(start, end - start));

            //std::cout << "\nTokenization complete, passed delimiter: " << del << "Results:\n";
            for (int i = 0; i < substrings.size(); i++){
                std::cout << substrings[i] << "\t" << i << "\n";
            }
            //std::cout << "\nDone\n";

            return substrings;
        }

/**
 * @brief Set the string value
 * 
 * @param var New string value
 */
        void set_input(std::string var){
            input = var;
        }
};

/**
* @brief Pipeline used to tokenize string and then convert it's characters to numbers
*
* @param input_string String containing data from sensor
* @param delimiter Character used to distinguish numbers in the string (by default ",")
*/
class Pipeline : public Tokenizer {
    private:
        std::vector<std::string> input;
        std::string del = ",";

        Tokenizer tokenizer;

/**
 * @brief Converts strings of numbers to vectors of floats
 * 
 * @param input Input vector of strings containing values to be numerized
 * 
 * @return std::vector<float> 
 */
        std::vector<float> convert_to_num(std::vector<std::string> input){
            //std::cout << "\nStarting conversion to num\n";
            std::vector<float> numerics;
            int i = 0;
            for(int i=0; i < 2; i++){
                if (std::stof(input[i])){
                    numerics.push_back(std::stof(input[i]));
                }
                else{
                    continue;
                }
            }

            std::cout << "Converted to num, result: \n";
            for(int i=0; i < numerics.size(); i++){
                std::cout << numerics[i] << " " << i << "\n";
            }
            std::cout << "\n";

            return numerics;
        }

    public:
        Pipeline(std::vector<std::string> input_string, std::string delimiter = ",") {
            input = input_string;
            del = delimiter;
        }

/**
 * @brief Run the pipeline and convert given strings to vectors of floats
 * 
 * @return std::vector<std::vector<float>> 
 */
        std::vector<std::vector<float>> ProcessData() {
            std::vector<std::vector<float>> valuable_data;

            for (int i = 0; i < input.size(); i++){
                if (input[i].size() != 16){
                    input.erase(input.begin()+i);
                }
            }

            for(int line = 0; line < input.size(); line ++){
                std::cout << input[line] << "\t" << input[line].size() << std::endl;
                tokenizer.set_input(input[line]);
                valuable_data.push_back(convert_to_num(tokenizer.tokenize()));
            }

            for (int i = 0; i<valuable_data.size(); i++){
                if (valuable_data[i].size() == 1){
                    valuable_data[i].insert(valuable_data[i].begin(), 0);
                }

                std::cout << valuable_data[i][1] << std::endl;
            }

            return valuable_data;
        }

};

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

/**
 * @brief Set everything necessary to handle port operations
 * 
 */
        int SetEverything(){
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
        char* ReadInput() {

            int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

            if (num_bytes < 0) {
                printf("Error reading: %s", strerror(errno));
                return read_buf;
            }

            printf("Read %i bytes. Received message: %s", num_bytes, read_buf);

            return read_buf;
        }

};


int main() {

    Setup my_port;
    my_port.SetEverything();


    std::string data = my_port.ReadInput();
    std::vector<std::string> first_try;
    std::vector<std::vector<float>> second_try;

    Tokenizer first_token(data, "\n");

    first_try = first_token.tokenize();

    Pipeline pipeline(first_try);
    
    second_try = pipeline.ProcessData();


}