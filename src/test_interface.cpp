#include "zdxxmp_controller/zdxxmp_controller.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
    ZDXXMPController heatChamberController(false);

    heatChamberController.init("/dev/ttyUSB1", {1,2,3});

    if (!heatChamberController.isInitialized()){
        printf("heatChamberController init failed\n");
        return 1;
    }

    int device_addr = 2;
    // heatChamberController.home(device_addr);
    // heatChamberController.open(device_addr);
    // heatChamberController.close(device_addr);
    heatChamberController.open(1);
    heatChamberController.open(2);
    heatChamberController.open(3);
    
    return 0;
}