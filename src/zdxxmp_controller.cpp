#include "zdxxmp_controller/zdxxmp_controller.hpp"
#include <thread>
#include <future>
#include <chrono>
#include <algorithm>

extern "C"{
    #include "zdxxmp.h"
}

ZDXXMPController::ZDXXMPController(bool sim_mode) : sim_mode_(sim_mode){    
}

ZDXXMPController::~ZDXXMPController(){
    for (auto device_addr: device_addresses_){
        _unlock_when_stopped(device_addr);        
    }    
}

bool ZDXXMPController::init(std::string port, std::vector<int> device_addresses){
    isInitialized_ = false;

    device_addresses_ = device_addresses;

    if (sim_mode_) {
        for (auto device_addr: device_addresses){
            fake_state_[device_addr] = State::OPEN;
        }
        isInitialized_ = true;
        return true;
    }

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
        int ret = _stop(device_addr);
        if (ret==-1){
            printf("==ERROR== device with address %d not found in port %s\n", device_addr, port.c_str() );
            return false;
        }
        // take over control i.e. cannot move by hand now
        _lock_when_stopped(device_addr);
    }    

    isInitialized_ = true;
    return true;
}

bool ZDXXMPController::homeAll(){
    // home then open all heatchambers in parrallel
    auto init_task = [&](int device_addr) -> bool {
        bool ok;

        ok = home(device_addr);
        if (!ok) return false;

        ok = open(device_addr);
        if (!ok) return false;

        return true;
    };

    std::vector<std::future<bool>> init_results;    
    for (auto device_addr: device_addresses_){
        init_results.push_back( std::async(std::launch::async, init_task, device_addr) );
    }

    bool allOK = true;
    for (unsigned int i=0;i<init_results.size();++i){
        bool ok = init_results[i].get();
        if (!ok) printf("==ERROR== device with address %d failed homing and init\n", device_addresses_[i]);
        allOK &= ok;
    }
    if (!allOK) return false;
    
    return true;
}

bool ZDXXMPController::home(int device_addr){
    if (!isInitialized_) return false;

    if (sim_mode_) {
        fake_state_[device_addr] = State::CLOSE;
        return true;
    }

    int ret_status;

    // move fast to near home
    ret_status = _move_backwards(device_addr, open_size);
    if (ret_status==-1) {
        printf("==ERROR== no reply from device with address %d\n", device_addr);
        return false;
    }
    ret_status = wait(device_addr, 8.0, {STATE_IDLE, STATE_DOWN_BUTTON_PRESSED});

    for (int trial=0; trial<3; ++trial){
        // whatever go wrong, move back a bit till limit switch is not pressed, then retry homing
        while (ret_status==STATE_DOWN_BUTTON_PRESSED || ret_status==STATE_ERROR_RETURNING){
            ret_status = _move_forwards(device_addr, int(open_size*0.05));
            ret_status = wait(device_addr, 1.0);
        }

        // go home
        ret_status = _homing(device_addr);    
        ret_status = wait(device_addr, 10.0, {STATE_IDLE,STATE_ERROR_RETURNING} );

        if (ret_status==STATE_IDLE) break;
    }

    _stop(device_addr);

    return ret_status==STATE_IDLE;
}

bool ZDXXMPController::open(int device_addr){
    if (!isInitialized_) return false;

    if (sim_mode_) {
        fake_state_[device_addr] = State::OPEN;
        return true;
    }

    if (getState(device_addr)==State::OPEN) return true;

    int ret_status = _move_forwards(device_addr, open_size);
    if (ret_status==-1) {
        printf("==ERROR== no reply from device with address %d\n", device_addr);
        return false;
    }

    wait(device_addr, 8.0, {STATE_IDLE, STATE_UP_BUTTON_PRESSED});

    _stop(device_addr);

    return getState(device_addr) == State::OPEN;
}

bool ZDXXMPController::close(int device_addr){
    if (!isInitialized_) return false;

    if (sim_mode_) {
        fake_state_[device_addr] = State::CLOSE;
        return true;
    }

    if (getState(device_addr)==State::CLOSE) return true;

    int ret_status = _move_backwards(device_addr, open_size);
    if (ret_status==-1) {
        printf("==ERROR== no reply from device with address %d\n", device_addr);
        return false;
    }

    // wait for close motion complete
    wait(device_addr, 8.0, {STATE_IDLE, STATE_DOWN_BUTTON_PRESSED});

    // release motor lock to let the heat chamber fall by gravity to the motherboard, to close the last few mm gap
    _unlock_when_stopped(device_addr);    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // reapply motor lock
    _lock_when_stopped(device_addr);

    _stop(device_addr);

    return getState(device_addr) == State::CLOSE;
}

int ZDXXMPController::wait(int device_addr, double timeout_seconds){    
    return wait(device_addr, timeout_seconds, {STATE_IDLE});
}

int ZDXXMPController::wait(int device_addr, double timeout_seconds, std::vector<uint32_t> target_states){
    double dt_ms = 100;

    int N = int(timeout_seconds*1000/dt_ms);
    if (N<=0) N=1;

    int ret;

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

    if (sim_mode_) {        
        return fake_state_[device_addr];
    }

    int ret;

    // guess state from button
    auto buttonState = getButtonState(device_addr);
    if (buttonState[1]) return State::OPEN;
    if (buttonState[2]) return State::CLOSE;

    // guess state from last/current motion cmd's status
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

    // guess state from encoder value
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

int ZDXXMPController::getLowLevelState(int device_addr){
    int ret = _read_value(device_addr, REG_CURRENT_STATE);
    if (ret>=0) ret = ret & 0x00FF; // 0xFF00 is button state bytes(?) 0x00FF is motion state bytes
    print_state_message(ret);
    return ret;
}

// vector of len=3: state of button home up down
std::vector<uint8_t> ZDXXMPController::getButtonState(int device_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    uint8_t * ret = read_button_states(mb, device_addr);
    auto output = std::vector<uint8_t>(3);
    for (std::size_t i=0; i<3; ++i){
        output[i] = ret[i];
    }
    return output;
}

int ZDXXMPController::getPosition(int device_addr){
    int ret = _read_value(device_addr, REG_CURRENT_POS_H);
    return ret;
}

// wrapper around similarly named funcs in zdxxmp.h
int ZDXXMPController::_read_value(int device_addr, int reg_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return read_value(mb, device_addr, reg_addr);
}

int ZDXXMPController::_homing(int device_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return homing(mb, device_addr);
}

int ZDXXMPController::_goto_position(int device_addr, uint32_t position){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return goto_position(mb, device_addr, position);
}

int ZDXXMPController::_move_forwards(int device_addr, uint32_t steps){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return move_forwards(mb, device_addr, steps);
}

int ZDXXMPController::_move_backwards(int device_addr, uint32_t steps){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return move_backwards(mb, device_addr, steps);
}

int ZDXXMPController::_stop(int device_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return stop(mb, device_addr);
}

int ZDXXMPController::_lock_when_stopped(int device_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return lock_when_stopped(mb, device_addr);
}

int ZDXXMPController::_unlock_when_stopped(int device_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return unlock_when_stopped(mb, device_addr);
}

int ZDXXMPController::_flash_parameters(int device_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return flash_parameters(mb, device_addr);
}

int ZDXXMPController::_change_address(int old_device_addr, int new_device_addr){
    std::lock_guard<std::mutex> lock(bus_mutex);
    return change_address(mb, old_device_addr, new_device_addr);
}