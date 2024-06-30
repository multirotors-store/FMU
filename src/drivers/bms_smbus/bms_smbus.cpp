/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bms_smbus.h
 *
 * Battery monitor connected via SMBus (I2C).
 * Designed for RDDRONE BBMS776
 * Reference to bms_smbus
 *
 * @author LongNH <Longnh2181@fpt.com>
 */

#include "bms_smbus.hpp"
#include <lib/atmosphere/atmosphere.h>

extern "C" __EXPORT int bms_smbus_main(int argc, char *argv[]);

BMS_SMBUS::BMS_SMBUS(const I2CSPIDriverConfig &config, SMBus *interface) :
    I2CSPIDriver(config),
    _interface(interface)
{
    int32_t battsource = 1;
    int32_t batt_device_type = static_cast<int32_t>(SMBUS_DEVICE_TYPE::UNDEFINED);

    param_set(param_find("BAT1_SOURCE"), &battsource);
    param_get(param_find("BAT1_SMBUS_MODEL"), &batt_device_type);


    //TODO: probe the device and autodetect its type
    if ((SMBUS_DEVICE_TYPE)batt_device_type == SMBUS_DEVICE_TYPE::BMS772) {
        _device_type = SMBUS_DEVICE_TYPE::BMS772;

    } else {
        //default
        _device_type = SMBUS_DEVICE_TYPE::UNDEFINED;
    }

    _interface->init();
    // unseal() here to allow an external config script to write to protected flash.
    // This is neccessary to avoid bus errors due to using standard i2c mode instead of SMbus mode.
    // The external config script should then seal() the device.
}

BMS_SMBUS::~BMS_SMBUS()
{
    orb_unadvertise(_batt_topic);
    perf_free(_cycle);

    if (_interface != nullptr) {
        delete _interface;
    }

    int32_t battsource = 0;
    param_set(param_find("BAT1_SOURCE"), &battsource);
}

void BMS_SMBUS::RunImpl()
{
    int ret = PX4_OK;

    // Temporary variable for storing SMBUS reads.
    uint16_t result;

    // Read data from sensor.
    battery_status_s new_report = {};

    // TODO(hyonlim): this driver should support multiple SMBUS going forward.
    new_report.id = 1;

    // Set time of reading.
    new_report.timestamp = hrt_absolute_time();

    new_report.connected = true;

    ret |= _interface->read_word(BMS_SMBUS_VOLTAGE, result, false);

    ret |= get_cell_voltages();

    for (int i = 0; i < _cell_count; i++) {
        new_report.voltage_cell_v[i] = _cell_voltages[i];
    }

    // Convert millivolts to volts.
    new_report.voltage_v = ((float)result) / 1000.0f;
    new_report.voltage_filtered_v = new_report.voltage_v;

    // Read current.
    ret |= _interface->read_word(BMS_SMBUS_CURRENT, result, false);

    new_report.current_a = (((float)(*(int16_t *)&result)) / 1000.0f) * _c_mult;
    new_report.current_filtered_a = new_report.current_a;

    // Read average current.
    ret |= _interface->read_word(BMS_SMBUS_AVERAGE_CURRENT, result, false);

    float average_current = (((float)(*(int16_t *)&result)) / 1000.0f) * _c_mult;
    new_report.current_average_a = average_current;

    // If current is high, turn under voltage protection off. This is neccessary to prevent
    // a battery from cutting off while flying with high current near the end of the packs capacity.
    // set_undervoltage_protection(average_current);

    // Read run time to empty (minutes).
    ret |= _interface->read_word(BMS_SMBUS_RUN_TIME_TO_EMPTY, result, false);
    new_report.time_remaining_s = result * 60;

    // Read average time to empty (minutes).
    ret |= _interface->read_word(BMS_SMBUS_AVERAGE_TIME_TO_EMPTY, result, false);
    new_report.average_time_to_empty = result;

    // Read remaining capacity.
    ret |= _interface->read_word(BMS_SMBUS_REMAINING_CAPACITY, result, false);

    // Calculate total discharged amount in mah.
    new_report.discharged_mah = _batt_startup_capacity - (float)result * _c_mult;

    // Read Relative SOC.
    ret |= _interface->read_word(BMS_SMBUS_RELATIVE_SOC, result, false);

    // Normalize 0.0 to 1.0
    new_report.remaining = (float)result / 100.0f;

    // Read Max Error
    ret |= _interface->read_word(BMS_SMBUS_MAX_ERROR, result, false);
    new_report.max_error = result;

    // Read battery temperature and covert to Celsius.
    ret |= _interface->read_word(BMS_SMBUS_TEMP, result, false);
    new_report.temperature = ((float)result / 10.0f) + atmosphere::kAbsoluteNullCelsius;

    // Only publish if no errors.
    if (ret == PX4_OK) {
        new_report.capacity = _batt_capacity;
        new_report.cycle_count = _cycle_count;
        new_report.serial_number = _serial_number;
        new_report.max_cell_voltage_delta = _max_cell_voltage_delta;
        new_report.cell_count = _cell_count;
        // new_report.state_of_health = _state_of_health;

        // Check if max lifetime voltage delta is greater than allowed.
        if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
            new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

        } else if (new_report.remaining > _low_thr) {
            new_report.warning = battery_status_s::BATTERY_WARNING_NONE;

        } else if (new_report.remaining > _crit_thr) {
            new_report.warning = battery_status_s::BATTERY_WARNING_LOW;

        } else if (new_report.remaining > _emergency_thr) {
            new_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

        } else {
            new_report.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;
        }

        new_report.interface_error = perf_event_count(_interface->_interface_errors);

        int instance = 0;
        orb_publish_auto(ORB_ID(battery_status), &_batt_topic, &new_report, &instance);

        _last_report = new_report;
    }
}

void BMS_SMBUS::suspend()
{
    ScheduleClear();
}

void BMS_SMBUS::resume()
{
    ScheduleOnInterval(BMS_SMBUS_MEASUREMENT_INTERVAL_US);
}

int BMS_SMBUS::get_cell_voltages()
{
    // Temporary variable for storing SMBUS reads.
    uint16_t result = 0;
    int ret = PX4_OK;

    ret |= _interface->read_word(BMS_SMBUS_CELL_1_VOLTAGE, result, false);
    // Convert millivolts to volts.
    _cell_voltages[0] = ((float)result) / 1000.0f;

    ret |= _interface->read_word(BMS_SMBUS_CELL_2_VOLTAGE, result, false);
    // Convert millivolts to volts.
    _cell_voltages[1] = ((float)result) / 1000.0f;

    ret |= _interface->read_word(BMS_SMBUS_CELL_3_VOLTAGE, result, false);
    // Convert millivolts to volts.
    _cell_voltages[2] = ((float)result) / 1000.0f;

    ret |= _interface->read_word(BMS_SMBUS_CELL_4_VOLTAGE, result, false);
    // Convert millivolts to volts.
    _cell_voltages[3] = ((float)result) / 1000.0f;

    ret |= _interface->read_word(BMS_SMBUS_CELL_5_VOLTAGE, result, false);
    // Convert millivolts to volts.
    _cell_voltages[4] = ((float)result) / 1000.0f;

    ret |= _interface->read_word(BMS_SMBUS_CELL_6_VOLTAGE, result, false);
    // Convert millivolts to volts.
    _cell_voltages[5] = ((float)result) / 1000.0f;

    _cell_voltages[6] = 0;

    //Calculate max cell delta
    _min_cell_voltage = _cell_voltages[0];
    float max_cell_voltage = _cell_voltages[0];

    for (uint8_t i = 1; (i < _cell_count && i < (sizeof(_cell_voltages) / sizeof(_cell_voltages[0]))); i++) {
        _min_cell_voltage = math::min(_min_cell_voltage, _cell_voltages[i]);
        max_cell_voltage = math::max(max_cell_voltage, _cell_voltages[i]);
    }

    // Calculate the max difference between the min and max cells with complementary filter.
    _max_cell_voltage_delta = (0.5f * (max_cell_voltage - _min_cell_voltage)) +
                  (0.5f * _last_report.max_cell_voltage_delta);

    return ret;
}

void BMS_SMBUS::set_undervoltage_protection(float average_current)
{
    // Disable undervoltage protection if armed. Enable if disarmed and cell voltage is above limit.
    if (average_current > BATT_CURRENT_UNDERVOLTAGE_THRESHOLD) {
        if (_cell_undervoltage_protection_status != 0) {
            PX4_WARN("Disable undervoltage protection");
        }

    } else {
        if (_cell_undervoltage_protection_status == 0) {
            if (_min_cell_voltage > BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD) {
                PX4_WARN("Enable undervoltage protection");
            }
        }
    }
}

int BMS_SMBUS::get_startup_info()
{
    int ret = PX4_OK;

    // Read battery threshold params on startup.
    // TODO: support instances
    param_get(param_find("BAT_CRIT_THR"), &_crit_thr);
    // PX4_INFO("BAT_CRIT_THR: %f", _crit_thr);
    param_get(param_find("BAT_LOW_THR"), &_low_thr);
    // PX4_INFO("BAT_LOW_THR: %f", _low_thr);
    param_get(param_find("BAT_EMERGEN_THR"), &_emergency_thr);
    // PX4_INFO("BAT_EMERGEN_THR: %f", _emergency_thr);
    param_get(param_find("BAT1_C_MULT"), &_c_mult);
    // PX4_INFO("BAT1_C_MULT: %f", _c_mult);

    int32_t cell_count_param = 0;
    param_get(param_find("BAT1_N_CELLS"), &cell_count_param);

    _cell_count = math::min((uint8_t)cell_count_param, MAX_NUM_OF_CELLS);

    ret |= _interface->block_read(BMS_SMBUS_MANUFACTURER_NAME, _manufacturer_name, BMS_SMBUS_MANUFACTURER_NAME_SIZE,
                      false);
    _manufacturer_name[sizeof(_manufacturer_name) - 1] = '\0';

    uint16_t serial_num;
    ret |= _interface->read_word(BMS_SMBUS_SERIAL_NUMBER, serial_num, false);

    uint16_t remaining_cap;
    ret |= _interface->read_word(BMS_SMBUS_REMAINING_CAPACITY, remaining_cap, false);

    uint16_t cycle_count;
    ret |= _interface->read_word(BMS_SMBUS_CYCLE_COUNT, cycle_count, false);

    uint16_t full_cap;
    ret |= _interface->read_word(BMS_SMBUS_FULL_CHARGE_CAPACITY, full_cap, false);

    uint16_t manufacture_date;
    ret |= _interface->read_word(BMS_SMBUS_MANUFACTURE_DATE, manufacture_date, false);

    if (!ret) {
        _serial_number = serial_num;
        _batt_startup_capacity = (uint16_t)((float)remaining_cap * _c_mult);
        _cycle_count = cycle_count;
        _batt_capacity = (uint16_t)((float)full_cap * _c_mult);
        _manufacture_date = manufacture_date;
    }

    return ret;
}

void BMS_SMBUS::getInfo()
{
    uint16_t result = 0;
    int ret = PX4_OK;

    // ToDo: 
    ret |= _interface->read_word(BMS_SMBUS_TEMP, result, false);
    PX4_INFO("BMS_SMBUS_TEMP: %6.4lf", (double)(((float)result / 10.0f) + atmosphere::kAbsoluteNullCelsius));

    ret |= _interface->read_word(BMS_SMBUS_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_VOLTAGE: %6.4lf", (double)(((float)result) / 1000.0f));

    ret |= _interface->read_word(BMS_SMBUS_CURRENT, result, false);
    PX4_INFO("BMS_SMBUS_CURRENT: %8.3lf", (double)(((float)(*(int16_t *)&result))/ 1000.0f * _c_mult));

    ret |= _interface->read_word(BMS_SMBUS_AVERAGE_CURRENT, result, false);
    PX4_INFO("BMS_SMBUS_AVERAGE_CURRENT: %8.3lf", (double)(((float)(*(int16_t *)&result)/ 1000.0f * _c_mult)));

    ret |= _interface->read_word(BMS_SMBUS_MAX_ERROR, result, false);
    PX4_INFO("BMS_SMBUS_MAX_ERROR: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_RELATIVE_SOC, result, false);
    PX4_INFO("BMS_SMBUS_RELATIVE_SOC: %6.4lf", (double)((float)result / 100.0f));

    ret |= _interface->read_word(BMS_SMBUS_ABSOLUTE_SOC, result, false);
    PX4_INFO("BMS_SMBUS_ABSOLUTE_SOC: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_REMAINING_CAPACITY, result, false);
    PX4_INFO("BMS_SMBUS_REMAINING_CAPACITY: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_FULL_CHARGE_CAPACITY, result, false);
    PX4_INFO("BMS_SMBUS_FULL_CHARGE_CAPACITY: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_RUN_TIME_TO_EMPTY, result, false);
    PX4_INFO("BMS_SMBUS_RUN_TIME_TO_EMPTY: %d", result * 60);

    ret |= _interface->read_word(BMS_SMBUS_AVERAGE_TIME_TO_EMPTY, result, false);
    PX4_INFO("BMS_SMBUS_AVERAGE_TIME_TO_EMPTY: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_CYCLE_COUNT, result, false);
    PX4_INFO("BMS_SMBUS_CYCLE_COUNT: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_DESIGN_CAPACITY, result, false);
    PX4_INFO("BMS_SMBUS_DESIGN_CAPACITY: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_DESIGN_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_DESIGN_VOLTAGE: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_MANUFACTURE_DATE, result, false);
    PX4_INFO("BMS_SMBUS_MANUFACTURE_DATE: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_SERIAL_NUMBER, result, false);
    PX4_INFO("BMS_SMBUS_SERIAL_NUMBER: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_CELL_1_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_CELL_1_VOLTAGE: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_CELL_2_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_CELL_2_VOLTAGE: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_CELL_3_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_CELL_3_VOLTAGE: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_CELL_4_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_CELL_4_VOLTAGE: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_CELL_5_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_CELL_5_VOLTAGE: %d", result);

    ret |= _interface->read_word(BMS_SMBUS_CELL_6_VOLTAGE, result, false);
    PX4_INFO("BMS_SMBUS_CELL_6_VOLTAGE: %d", result);

}

void BMS_SMBUS::print_usage()
{
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Smart battery driver for the RDDRONE-BMS772.

### Examples
To write to flash to set parameters. address, number_of_bytes, byte0, ... , byteN
To start the application on external I2C port, on bus 3 with device slave address is 8
$ bms_smbus start -X -b 3 -a 8

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("bms_smbus", "driver");

    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x08);

    PRINT_MODULE_USAGE_COMMAND_DESCR("man_info", "Prints manufacturer info.");
    PRINT_MODULE_USAGE_COMMAND_DESCR("suspend", "Suspends the driver from rescheduling the cycle.");
    PRINT_MODULE_USAGE_COMMAND_DESCR("resume", "Resumes the driver from suspension.");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *BMS_SMBUS::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
    SMBus *interface = new SMBus(DRV_BAT_DEVTYPE_SMBUS, config.bus, config.i2c_address);
    if (interface == nullptr) {
        PX4_ERR("alloc failed");
        return nullptr;
    }
    BMS_SMBUS *instance = new BMS_SMBUS(config, interface);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
        return nullptr;
    }

    int ret = instance->get_startup_info();

    if (ret != PX4_OK) {
        delete instance;
        return nullptr;
    }

    instance->ScheduleOnInterval(BMS_SMBUS_MEASUREMENT_INTERVAL_US);

    return instance;
}

void
BMS_SMBUS::custom_method(const BusCLIArguments &cli)
{
    switch(cli.custom1) {
        case 1: {
            PX4_INFO("The manufacturer name: %s", _manufacturer_name);
            PX4_INFO("The manufacturer date: %" PRId16, _manufacture_date);
            PX4_INFO("The serial number: %d" PRId16, _serial_number);
        }
            break;
        case 2:
            suspend();
            break;
        case 3:
            resume();
            break;
        case 4:
            getInfo();
            break;
    }
}

extern "C" __EXPORT int bms_smbus_main(int argc, char *argv[])
{
    using ThisDriver = BMS_SMBUS;
    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = 100000;
    cli.i2c_address = BMS_SMBUS_ADDR;

    const char *verb = cli.parseDefaultArguments(argc, argv);
    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_DEVTYPE_SMBUS);

    if (!strcmp(verb, "start")) {
        return ThisDriver::module_start(cli, iterator);
    }

    if (!strcmp(verb, "stop")) {
        return ThisDriver::module_stop(iterator);
    }

    if (!strcmp(verb, "status")) {
        return ThisDriver::module_status(iterator);
    }

    if (!strcmp(verb, "man_info")) {
        cli.custom1 = 1;
        return ThisDriver::module_custom_method(cli, iterator, false);
    }
    if (!strcmp(verb, "suspend")) {
        cli.custom1 = 2;
        return ThisDriver::module_custom_method(cli, iterator);
    }
    if (!strcmp(verb, "resume")) {
        cli.custom1 = 3;
        return ThisDriver::module_custom_method(cli, iterator);
    }
    if (!strcmp(verb, "get_all")) {
        cli.custom1 = 4;
        return ThisDriver::module_custom_method(cli, iterator, false);
    }

    ThisDriver::print_usage();
    return -1;
}
