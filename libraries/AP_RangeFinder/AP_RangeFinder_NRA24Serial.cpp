#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_SERIAL_ENABLED

#include <ctype.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_RangeFinder_NRA24Serial.h"

extern const AP_HAL::HAL& hal;

bool AP_RangeFinder_NRA24_Serial::detect(uint8_t serial_instance){
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// A handler for incoming measurements; writes data to reading_cm on success
inline bool if_data_frame(uint8_t *buf, float &reading_cm)
{
    
    if(nullptr == buf){
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NRA24_Serial if_data_frame() fail1");
        return false;
    }


    uint8_t *payload = (buf+4);
    uint8_t *msg_id = (buf+2);

    // We expect Message ID to match "Target output information (Target Info)"
    if((msg_id[0] + (((uint16_t)msg_id[1]) << 8)) != 0x70c){
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NRA24_Serial if_data_frame() fail2");
        return false;
    }

    reading_cm = static_cast<float>((static_cast<uint16_t>(payload[2]) << 8) + payload[3]) * 0.01;

    return true;
}

bool AP_RangeFinder_NRA24_Serial::checksum_ok(uint8_t *buf)
{
#define NRA_CHECKSUM_LEN 7
    uint16_t sum;
    uint8_t i;

    if(NULL == buf){
        return false;
    }

    sum = 0;
    for(i = 0; i < NRA_CHECKSUM_LEN; ++i){
        sum += (uint16_t)buf[i];
    }
    sum = sum & (uint16_t)(0xFF);
    if((uint8_t)sum == buf[NRA_CHECKSUM_LEN]){
        return true;
    }else{
        if(params.debug){
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NRA24UART CHKSM FAIL");
        }
    }

    return false;
#undef NRA_CHECKSUM_LEN
}

bool AP_RangeFinder_NRA24_Serial::get_reading(float &reading_cm)
{
    
    if(uart == nullptr)
        return false;

    uint32_t nbytes = uart->available();
    while(nbytes-->0){
        uint8_t c = uart->read();

        switch(_reading_state){
        case Status::WAITING:{            
            if(c == 0xAA){
                buffer_count = 0;
                linebuf[buffer_count] = c;
                _reading_state = Status::GET_HEAD_ONCE;
            }
            break;
        }
        case Status::GET_HEAD_ONCE:{
            if(c == 0xAA){
                buffer_count++;
                linebuf[buffer_count] = c;
                _reading_state = Status::WAITING_FOR_TAIL;
            }else{
                buffer_count++;
                linebuf[buffer_count] = 0xAA;
                buffer_count++;
                linebuf[buffer_count] = c;
                _reading_state = Status::WAITING_FOR_TAIL;
            }
            break;
        }
        case Status::WAITING_FOR_TAIL:{
            buffer_count++;
            linebuf[buffer_count] = c;
            if(c == 0x55)
                _reading_state=Status::GET_TAIL_ONCE;
            break;
        }

        case Status::GET_TAIL_ONCE:{
            if(c == 0x55){
                buffer_count++;
                linebuf[buffer_count] = c;
                _reading_state = Status::GET_ONE_FRAME;
            }
            break;
        }
        case Status::GET_ONE_FRAME:{
            _reading_state = Status::WAITING;
            if(params.crc && checksum_ok(linebuf) == false){
                return false;
            }
            if(/*checksum_ok(linebuf) &&*/ if_data_frame(linebuf, reading_cm)){
                return true;
            }
            break;
        }

        default:
            break;
        }
        if(buffer_count>sizeof(linebuf)){
            buffer_count = 0;
            _reading_state = Status::WAITING;
        }
    }
    return true;
}

#endif  // AP_RANGEFINDER_NRA24_SERIAL_ENABLED
