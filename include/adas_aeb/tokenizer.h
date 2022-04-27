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
        std::string input;

    public:
        Tokenizer();
        ~Tokenizer();
        static std::vector<std::string> tokenize(std::string, std::string = ",");
};