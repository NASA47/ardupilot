#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_NRA24_SERIAL_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"
#define NRA24_SERIAL_BAUD_RATE 115200

class AP_RangeFinder_NRA24_Serial : public AP_RangeFinder_Backend_Serial
{

public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return new AP_RangeFinder_NRA24_Serial(_state, _params);
    }

    static bool detect(uint8_t serial_instance);
    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;
    uint32_t initial_baudrate(uint8_t serial_instance) const override{
        return NRA24_SERIAL_BAUD_RATE;
    }
protected:
    enum class Status {
        WAITING=0,
        GET_HEAD_ONCE,
        GET_HEAD,
        GET_TAIL_ONCE,
        GET_TAIL,
        GET_ONE_FRAME,
        WAITING_FOR_TAIL,
    }_reading_state;

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

private:
    // get a reading
    bool get_reading(float &reading_m) override;
    bool checksum_ok(uint8_t *buf);

    uint8_t linebuf[50];
    uint8_t buffer_count=0;
};

#endif // AP_RANGEFINDER_NRA24_SERIAL_ENABLED
