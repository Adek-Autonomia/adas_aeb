#include "tokenizer.h"



/**
* @brief Initialize a Tokenizer that tokenizes (duh) a given string
*
*/
Tokenizer::Tokenizer() 
{
}


/**
 * @brief Default destructor. It does nothing right now.
 * 
 */
Tokenizer::~Tokenizer()
{

}

/**
 * @brief Tokenizes a string passed to the class
 *  
 * @return std::vector<std::string> 
 */
    std::vector<std::string> Tokenizer::tokenize(std::string* ptr_input, std::string delimiter){
    std::vector<std::string> substrings;
    int start = 0;
    std::string input = *ptr_input;
    std::cout << "\nInput:" << input << "\n";
    int end = input.find(delimiter);

    while (end != -1) {
        substrings.push_back(input.substr(start, end - start));
        start = end + delimiter.size();
        end = input.find(delimiter, start);
    }
    substrings.push_back(input.substr(start, end - start));

    std::cout << "\nTokenization complete, passed delimiter: " << delimiter << "Results:\n";
    for (int i = 0; i < substrings.size(); i++){
        std::cout << substrings[i] << "\t" << i << "\n";
    }
    //std::cout << "\nDone\n";

    return substrings;
}