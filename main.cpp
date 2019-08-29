#include <iostream>
#include <MavProxy.h>

int main() {
    MavProxy proxy("/dev/ttyS0", 115200);
    proxy.run();
    std::cout << "Finish" << std::endl;
    return 0;
}