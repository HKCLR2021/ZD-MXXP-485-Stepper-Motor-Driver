#include <string>
#include <vector>
#include <map>
#include <mutex>

struct _modbus;
typedef struct _modbus modbus_t;

class ZDXXMPController{
    public:
        enum State{
            OPEN     = 0,
            CLOSE    = 1,
            HALFOPEN = 2,
            MOVING   = 3,
            UNKNOWN  = 4
        };

        ZDXXMPController(bool sim_mode=false);
        ~ZDXXMPController();
        
        bool init(std::string port, std::vector<int> device_addresses);
        bool isInitialized() { return isInitialized_;}
        
        bool homeMultiple(std::vector<int> device_addresses);
        bool homeAll();
        bool home(int device_addr);
        bool open(int device_addr);
        bool close(int device_addr);
        State getState(int device_addr);
        int getLowLevelState(int device_addr);
        std::vector<uint8_t> getButtonState(int device_addr);
        int getPosition(int device_addr);
    
        int _goto_position (int device_addr, uint32_t position);
        int _move_forwards (int device_addr, uint32_t steps);
        int _move_backwards(int device_addr, uint32_t steps);
        int _stop          (int device_addr);

        int _lock_when_stopped  (int device_addr);
        int _unlock_when_stopped(int device_addr);
        
    private:
        int wait(int device_addr, double timeout_seconds);
        int wait(int device_addr, double timeout_seconds, std::vector<uint32_t> target_states);

        // wrapper around similarly named funcs in zdxxmp.h
        int _read_value(int device_addr, int reg_addr);

        int _homing        (int device_addr);
        
        int _flash_parameters   (int device_addr);
        int _change_address     (int old_device_addr, int new_device_addr);


        // steps, slightly less than 53838 which is (23 gear reduction)*(360/1.8 step per angle)*(133/360 degrees to turn)
        static const int open_size = 49000; 

        modbus_t *mb;
        bool isInitialized_ = false;

        std::mutex bus_mutex;
        std::vector<int> device_addresses_;

        bool sim_mode_ = true;
        std::map<int, State> fake_state_;
};