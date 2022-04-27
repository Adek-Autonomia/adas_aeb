#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

int main() {
    SerialPort serialPort("/dev/ttyACM0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);

    serialPort.SetTimeout(-1);
    serialPort.Open();

    std::string readData;
    serialPort.Read(readData);
    std::cout << readData << "\n";

    serialPort.Close();
}
