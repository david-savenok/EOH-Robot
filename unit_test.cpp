#include <string>
#include <iostream>
#include <vector>

using namespace std;

typedef struct Instruction {
    string command; //This needs to be fixed
    char instruction_type;
} Instruction;

Instruction parseInstructionSet(string& instruction_set) {
    Instruction result;
    //parse the instruction set and remove the first Instruction
    //returns a COMMAND, INSTRUCTION TYPE, and the truncated INSTRUCTION SET
    char instruction_type;
    string command = "";
    size_t i = 0;
    if (instruction_set.at(i) == '/') {
        i++;
        instruction_type = instruction_set.at(i);
        i += 2;
        while (instruction_set.at(i) != '*') {
            command += instruction_set.at(i);
            i++;
        }
        if (instruction_set.at(i) == '*') i++;
        instruction_set = instruction_set.substr(i);
    }

    result.command = command;
    result.instruction_type = instruction_type;

    return result;
}

int main() {
    string instruction_set = "/T*COMMAND*/B*COMMAND2*/";
    Instruction ret = parseInstructionSet(instruction_set);
    cout << ret.instruction_type << " *** " << ret.command << endl << instruction_set << endl;
}