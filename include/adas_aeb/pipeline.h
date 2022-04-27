#include <string.h>
#include <string>
#include <vector>
#include "tokenizer.h"

#pragma once

/**
* @brief Pipeline used to tokenize string and then convert it's characters to numbers
*
* @param input_string String containing data from sensor
* @param delimiter Character used to distinguish numbers in the string (by default ",")
*/
class Pipeline : public Tokenizer {
    private:
        std::vector<std::string> input;
        std::string delimiter;
        std::vector<float> convert_to_num(std::vector<std::string>);
            

    public:
        Pipeline(std::vector<std::string>, std::string = ",");
        ~Pipeline();
        std::vector<std::vector<float>> ProcessData();

};