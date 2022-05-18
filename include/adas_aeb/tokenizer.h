#include <string.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>

#pragma once


/**
* @brief Tokenize a given string
*
*/
class Tokenizer {

    private:

    public:
        Tokenizer();
        ~Tokenizer();
        static std::vector<std::string> tokenize(std::string* ptr_input, std::string = ",");
};