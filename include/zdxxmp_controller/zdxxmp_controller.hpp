#include <string>
#include <vector>

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
        
        bool init(std::string port, std::vector<int> device_addresses);
        bool isInitialized() { return isInitialized_;}
        
        bool home(int device_addr);
        bool open(int device_addr);
        bool close(int device_addr);
        State getState(int device_addr);
        uint32_t getLowLevelState(int device_addr);
        uint32_t getPosition(int device_addr);
    
    private:
        uint32_t wait(int device_addr, double timeout_seconds);
        uint32_t wait(int device_addr, double timeout_seconds, std::vector<uint32_t> target_states);

        // steps, slightly less than 53838 which is (23 gear reduction)*(360/1.8 step per angle)*(133/360 degrees to turn)
        static const int open_size = 49000; 

        modbus_t *mb;
        bool isInitialized_ = false;
};