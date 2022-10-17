#include "zdxxmp_controller/zdxxmp_controller.hpp"
#include "zdxxmp.c"
#include <thread>
#include <chrono>
#include <algorithm>


bool ZDXXMPController::init(std::string port, std::vector<int> device_addresses){
    isInitialized_ = false;

    // modbus_new_rtu(const char *device, int baud, char parity, int data_bit, int stop_bit);
    mb = modbus_new_rtu(port.c_str(), 9600, 'N', 8, 1);

    modbus_set_debug(mb, TRUE);
    modbus_set_response_timeout(mb, 0.5, 0);

    if (mb == NULL)
    {
        modbus_free(mb);
        printf("new rtu failed: %s\n", modbus_strerror(errno));
        return false;
    }

    int ret_status = modbus_connect(mb);

    if (ret_status == -1)
    {
        modbus_close(mb);
        modbus_free(mb);
        printf("connect failed: %s\n", modbus_strerror(errno));
        printf("You may need to start terminal as root or use sudo"); 
        return false;
    }

    for (auto device_addr: device_addresses){
        int ret = stop(mb,device_addr);
        if (ret==-1){
            printf("==WARN== device with address %d not found in port %s\n", device_addr, port.c_str() );
        }
    }

    isInitialized_ = true;
    return true;
}

bool ZDXXMPController::home(int device_addr){
    if (!isInitialized_) return false;

    int ret_status;

    // move fast to near home
    ret_status = move_backwards(mb, device_addr, open_size);
    if (ret_status==-1) {
        printf("==ERROR== no reply from device with address %d\n", device_addr);
        return false;
    }
    ret_status = wait(device_addr, 8.0, {STATE_IDLE, STATE_DOWN_BUTTON_PRESSED});

    // move back a bit till limit switch is not pressed
    while (ret_status==STATE_DOWN_BUTTON_PRESSED){
        ret_status = move_forwards(mb, device_addr, int(open_size*0.05));
        ret_status = wait(device_addr, 1.0);
    }

    ret_status = homing(mb, device_addr);
    
    ret_status = wait(device_addr, 10.0);

    stop(mb, device_addr);

    return ret_status==STATE_IDLE;
}

bool ZDXXMPController::open(int device_addr){
    if (!isInitialized_) return false;

    int ret_status = move_forwards(mb, device_addr, open_size);
    if (ret_status==-1) {
        printf("==ERROR== no reply from device with address %d\n", device_addr);
        return false;
    }

    uint32_t ret = wait(device_addr, 8.0, {STATE_IDLE, STATE_UP_BUTTON_PRESSED});

    stop(mb, device_addr);

    return getState(device_addr) == State::OPEN;
}

bool ZDXXMPController::close(int device_addr){
    if (!isInitialized_) return false;

    int ret_status;
    move_backwards(mb, device_addr, open_size);
    if (ret_status==-1) {
        printf("==ERROR== no reply from device with address %d\n", device_addr);
        return false;
    }

    // wait for close motion complete
    wait(device_addr, 8.0, {STATE_IDLE, STATE_DOWN_BUTTON_PRESSED});

    // release motor lock to let the heat chamber fall by gravity to the motherboard, to close the last few mm gap
    unlock_when_stopped(mb, device_addr);    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // reapply motor lock
    lock_when_stopped(mb, device_addr);

    stop(mb, device_addr);

    return getState(device_addr) == State::CLOSE;
}

uint32_t ZDXXMPController::wait(int device_addr, double timeout_seconds){    
    return wait(device_addr, timeout_seconds, {STATE_IDLE});
}

uint32_t ZDXXMPController::wait(int device_addr, double timeout_seconds, std::vector<uint32_t> target_states){
    double dt_ms = 100;

    int N = int(timeout_seconds*1000/dt_ms);
    if (N<=0) N=1;

    uint32_t ret;

    for (int i=0;i<N;i++){
        ret = getLowLevelState(device_addr);
        if (std::find(target_states.begin(), target_states.end(), ret) != target_states.end()) return ret;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printf("==WARN== timeout waiting device with address %d to reach target state\n", device_addr);
    return ret;
}

ZDXXMPController::State ZDXXMPController::getState(int device_addr){
    if (!isInitialized_) return State::UNKNOWN;

    uint32_t ret;

    ret = getLowLevelState(device_addr);
    if (ret==-1) {        
        printf("==ERROR== no reply from device with address %d\n", device_addr);
        return State::UNKNOWN;
    }

    if (ret==STATE_UNDEFINED){
        return State::UNKNOWN;
    }
    else if (ret==STATE_UP_BUTTON_PRESSED){
        return State::OPEN;
    }
    else if (ret==STATE_DOWN_BUTTON_PRESSED){
        return State::CLOSE;
    }    
    else if (ret == STATE_ACCEL   ||
             ret == STATE_CONST   ||
             ret == STATE_DECCEL  ||
             ret == STATE_HOMING  ){
        return State::MOVING;
    }

    ret = getPosition(device_addr);
    if ( ret<int(open_size*0.05) ){
        return State::CLOSE;
    }
    else if (ret<int(open_size*0.95)){
        return State::HALFOPEN;
    }
    else {
        return State::OPEN;
    }
}

uint32_t ZDXXMPController::getLowLevelState(int device_addr){
    uint32_t ret = read_value(mb, device_addr, REG_CURRENT_STATE);    
    print_state_message(ret);
    return ret;
}

uint32_t ZDXXMPController::getPosition(int device_addr){
    uint32_t ret = read_value(mb, device_addr, REG_CURRENT_POS_H);
    return ret;
}