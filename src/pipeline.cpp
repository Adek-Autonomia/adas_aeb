#include "pipeline.h"

/**
* @brief Initializes Pipeline used to tokenize string and then convert it's characters to numbers
*
* @param input_vector Vector of strings containing data from sensor
* @param delimiter Character used to distinguish numbers in the string (by default ",")
*/
Pipeline::Pipeline(std::vector<std::string> input_vector, std::string delimiter) {
    input = input_vector;
    delimiter = delimiter;
}

/**
 * @brief Default destructor. Does nothing.
 * 
 */
Pipeline::~Pipeline(){

}


/**
 * @brief Converts strings of numbers to vectors of floats
 * 
 * @param input Input vector of strings containing values to be numerized
 * 
 * @return std::vector<float> 
 */
std::vector<float> Pipeline::convert_to_num(std::vector<std::string> input){
    //std::cout << "\nStarting conversion to num\n";
    std::vector<float> numerics;
    int i = 0;
    for(int i=0; i < 2; i++){
        if ( i == 1){
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
    std::cout << "///////////////////////////////////////////////////////////\n";

    return numerics;
}


/**
 * @brief Run the pipeline and convert given strings to vectors of floats
 * 
 * @return std::vector<std::vector<float>> 
 */
std::vector<std::vector<float>> Pipeline::ProcessData() {
    std::vector<std::vector<float>> valuable_data;
    std::cout << "\nSize: " << input.size() << "\n";
    /*
    for (int i = 0; i < 2; i++){
        if (input[i].size() < 15 || input[i].size() > 18){
            std::cout << "\nLine size: " << input[i].size() << "\tDeleting line: " << i << "\n";
            input.erase(input.begin()+i);
            i = i - 1;
        }
    }
    */

   if (input.size() > 1) {
       input.erase(input.begin() + 1, input.end());
   }

    for(int line = 0; line < input.size(); line ++){
        std::cout << "\nVal data vect nr: " << line << "\t";
        valuable_data.push_back(convert_to_num(Tokenizer::tokenize(&input[line])));
        /*
        for (float j : valuable_data[line]){
            std::cout << j ;
            break;
        }
        */
    }


    for (int i = 0; i < valuable_data.size(); i++){
        if (valuable_data[i].size() == 1){
            valuable_data[i].insert(valuable_data[i].begin(), 0);
        }

        std::cout << valuable_data[i][1] << "\n";
    }

    return valuable_data;
}

/**
 * @brief Sets the input vector to the given value
 * 
 * @param input_vector 
 */
void Pipeline::set_input(std::vector<std::string> input_vector) {
    input = input_vector;
}