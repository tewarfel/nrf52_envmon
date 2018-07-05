/* mbed Microcontroller Library
 * Copyright (c) 2006-2014 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/BatteryService.h"
#include "ble/services/DeviceInformationService.h"
#include "ble/services/EnvironmentalService.h"
#include "nrf_soc.h"

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME) * 1000) / (RESOLUTION))

static const uint16_t UNIT_1_25_MS  = 1250;
static const uint16_t UNIT_10_MS = 10000;


// turn LED off
DigitalOut led1(LED1, 0);

const static char     DEVICE_NAME[] = "EnvTest4a";
static const uint16_t uuid16_list[] = {GattService::UUID_BATTERY_SERVICE,
                                       GattService::UUID_ENVIRONMENTAL_SERVICE
                                     //  ,GattService::UUID_DEVICE_INFORMATION_SERVICE
};

static uint8_t batteryLevel = 50;
static BatteryService* batteryServicePtr;

static EnvironmentalService::HumidityType_t humidityLevel = 50;
static float temperature = 38;

static EnvironmentalService* environmentalServicePtr;

//static DeviceInformationService* deviceInformationServicePtr;

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    printf("\r\nDisconnected\r\n");
    BLE::Instance().gap().startAdvertising();
}

void updateSensorValues() {
    batteryLevel++;
    if (batteryLevel > 100) {
        batteryLevel = 05;
    }
    batteryServicePtr->updateBatteryLevel(batteryLevel);
  
    humidityLevel++;
    if (humidityLevel > 99) {
      humidityLevel = 05;
    }
    environmentalServicePtr->updateHumidity(humidityLevel);

    if (1) {
      int32_t inttemp;
      sd_temp_get(&inttemp);
      temperature = (float)inttemp * 0.25f;
    }
    environmentalServicePtr->updateTemperature(temperature);
}

void blinkCallback(void)
{
    //led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */

    BLE &ble = BLE::Instance();
    if (ble.gap().getState().connected) {
        eventQueue.call(updateSensorValues);
    }
}

/**< Minimum connection interval (379 ms) */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(379, UNIT_1_25_MS)

/**< Maximum connection interval (399 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(399, UNIT_1_25_MS)

/**< Slave latency. */
#define SLAVE_LATENCY                   4                         

/**< Connection supervisory timeout (6 seconds). */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)
 
void onConnectionCallback(const Gap::ConnectionCallbackParams_t *p_conn_param)
{
    Gap::ConnectionParams_t gap_conn_params;
    gap_conn_params.minConnectionInterval = MIN_CONN_INTERVAL;
    gap_conn_params.maxConnectionInterval = MAX_CONN_INTERVAL;
    gap_conn_params.slaveLatency = SLAVE_LATENCY;
    gap_conn_params.connectionSupervisionTimeout = CONN_SUP_TIMEOUT;


    BLE::Instance().gap().updateConnectionParams(p_conn_param->handle, &gap_conn_params);
    printf("\r\nConnected\r\n");
}
 

/**
 * This function is called when the ble initialization process has failled
 */
void onBleInitError(BLE &ble, ble_error_t error)
{
    /* Initialization error handling should go here */
}

void printMacAddress()
{
    /* Print out device MAC address to the console*/
    Gap::AddressType_t addr_type;
    Gap::Address_t address;
    BLE::Instance().gap().getAddress(&addr_type, address);
    printf("DEVICE MAC ADDRESS: ");
    for (int i = 5; i >= 1; i--){
        printf("%02x:", address[i]);
    }
    printf("%02x\r\n", address[0]);
    printf("\r\n");
}

/**
 * Callback triggered when the ble initialization process has finished
 */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        /* In case of error, forward the error handling to onBleInitError */
        onBleInitError(ble, error);
        return;
    }

    /* Ensure that it is the default instance of BLE */
    if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);
    ble.gap().onConnection(onConnectionCallback);

    /* Setup primary service */
    batteryServicePtr = new BatteryService(ble, batteryLevel);

    environmentalServicePtr = new EnvironmentalService(ble);

    //deviceInformationServicePtr = new DeviceInformationService(ble);
  
    /* Setup advertising */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *) uuid16_list, sizeof(uuid16_list));
  
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *) DEVICE_NAME, sizeof(DEVICE_NAME));
  
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(750); /* 1000ms */

    // set max output power
    ble.gap().setTxPower(4);

    ble.gap().startAdvertising();

    printMacAddress();
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

int main()
{
    eventQueue.call_every(1000, blinkCallback);

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(scheduleBleEventsProcessing);
    ble.init(bleInitComplete);

    eventQueue.dispatch_forever();

    return 0;
}
