
#include <stdio.h>
#include <stdlib.h>
#include "espcan.h"
#include "esptimer.h"
#include "../inc/timer.h"
#include "../inc/data.h"
#include "../inc/sysdep.h"
#include "config.h"
// Canfestivals includes
#include "../inc/can.h"
#include "../inc/esp32xxx/canfestival.h"
#include "../inc/esp32xxx/applicfg.h"
#include "../dictionary/esp32xxx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#define TAG "espcan"

#define RX_TASK_PRIO 8

#if defined(CONFIG_IDF_TARGET_ESP32)
#define CAN_POWER_NEED_CONTROL
#define TX_GPIO_NUM 27
#define RX_GPIO_NUM 26
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#define TX_GPIO_NUM 16
#define RX_GPIO_NUM 15
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define TX_GPIO_NUM 16
#define RX_GPIO_NUM 15
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#define TX_GPIO_NUM 19
#define RX_GPIO_NUM 18
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#define TX_GPIO_NUM 19
#define RX_GPIO_NUM 18
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#define TX_GPIO_NUM 19
#define RX_GPIO_NUM 18
#endif

CO_Data *pCoData;

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

// sdo读canopen从站数据
UNS8 ReadSDO(UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, void *data, UNS32 *size)
{
    UNS32 abortCode = 0;
    UNS8 res = SDO_UPLOAD_IN_PROGRESS;
    // Read SDO
    UNS8 err = readNetworkDict(pCoData, nodeId, index, subIndex, dataType, 0);
    if (err)
        return 0xFF;
    for (;;)
    {
        res = getReadResultNetworkDict(pCoData, nodeId, data, size, &abortCode);

        ESP_LOGI(TAG, "sendsdo res = %x", res);
        if (res != SDO_UPLOAD_IN_PROGRESS)
            break;
        vTaskDelay(pdMS_TO_TICKS(2));
        continue;
    }
    closeSDOtransfer(pCoData, nodeId, SDO_CLIENT);
    if (res == SDO_FINISHED)
        return 0;
    return 0xFF;
}

UNS8 WriteSDO(UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS32 count, UNS8 dataType, void *data, UNS8 useBlockMode)
{
    UNS32 abortCode = 0;
    UNS8 res = SDO_DOWNLOAD_IN_PROGRESS;
    // Write SDO
    UNS8 err = writeNetworkDict(pCoData, nodeId, index, subIndex, count, dataType, data, useBlockMode);
    if (err)
        return 0xFF;
    for (;;)
    {
        res = getWriteResultNetworkDict(pCoData, nodeId, &abortCode);
        ESP_LOGI(TAG, "write res = %x", res);
        if (res != SDO_DOWNLOAD_IN_PROGRESS)
            break;
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
    }
    closeSDOtransfer(pCoData, nodeId, SDO_CLIENT);

    if (res == SDO_FINISHED)
        return 0;
    return 0xFF;
}

void CB_heartbeatError(CO_Data *d, UNS8 heartbeatID)
{
    ESP_LOGI(TAG, "CB_heartbeatError %d", heartbeatID);
}

void CB_initialisation(CO_Data *d)
{
    // UNS32 TPDO1_COBID = 0x201;
    // UNS32 TPDO2_COBID = 0x204;
    // UNS32 TPDO3_COBID = 0x207;
    // UNS32 size = sizeof(UNS32);
    //	UNS8 unsize = sizeof(unsize);
    //	UNS8  TpdoTransmissionType = 0;

    ESP_LOGI(TAG, "Master_initialisation");
    //	writeLocalDict( d, /*CO_Data* d*/
    //			0x1400, /*UNS16 index*/
    //			0x01, /*UNS8 subind*/
    //			&RPDO1_COBID, /*void * pSourceData,*/
    //			&size, /* UNS8 * pExpectedSize*/
    //			RW);  /* UNS8 checkAccess */
    //	TpdoTransmissionType = TRANS_EVENT_SPECIFIC;
    //	writeLocalDict( d, /*CO_Data* d*/
    //			0x1400, /*UNS16 index*/
    //			0x02, /*UNS8 subind*/
    //			&TpdoTransmissionType, /*void * pSourceData,*/
    //			(UNS32*)&unsize, /* UNS8 * pExpectedSize*/
    //			RW);  /* UNS8 checkAccess */
    //
    //	writeLocalDict( d, /*CO_Data* d*/
    //			0x1401, /*UNS16 index*/
    //			0x01, /*UNS8 subind*/
    //			&RPDO2_COBID, /*void * pSourceData,*/
    //			&size, /* UNS8 * pExpectedSize*/
    //			RW);  /* UNS8 checkAccess */
    //	TpdoTransmissionType = TRANS_EVENT_SPECIFIC;
    //	writeLocalDict( d, /*CO_Data* d*/
    //			0x1401, /*UNS16 index*/
    //			0x02, /*UNS8 subind*/
    //			&TpdoTransmissionType, /*void * pSourceData,*/
    //			(UNS32*)&unsize, /* UNS8 * pExpectedSize*/
    //			RW);  /* UNS8 checkAccess */

    // writeLocalDict(d,			 /*CO_Data* d*/
    // 			   0x1800,		 /*UNS16 index*/
    // 			   0x01,		 /*UNS8 subind*/
    // 			   &TPDO1_COBID, /*void * pSourceData,*/
    // 			   &size,		 /* UNS8 * pExpectedSize*/
    // 			   RW);			 /* UNS8 checkAccess */

    // writeLocalDict(d,			 /*CO_Data* d*/
    // 			   0x1801,		 /*UNS16 index*/
    // 			   0x01,		 /*UNS8 subind*/
    // 			   &TPDO2_COBID, /*void * pSourceData,*/
    // 			   &size,		 /* UNS8 * pExpectedSize*/
    // 			   RW);			 /* UNS8 checkAccess */
    // writeLocalDict(d,			 /*CO_Data* d*/
    // 			   0x1802,		 /*UNS16 index*/
    // 			   0x01,		 /*UNS8 subind*/
    // 			   &TPDO3_COBID, /*void * pSourceData,*/
    // 			   &size,		 /* UNS8 * pExpectedSize*/
    // 			   RW);			 /* UNS8 checkAccess */
}

void CB_preOperational(CO_Data *d)
{
    //	ESP_LOGI(TAG, "CB_preOperational");
}

void CB_operational(CO_Data *d)
{
    //	ESP_LOGI(TAG, "CB_operational");
}

void CB_stopped(CO_Data *d)
{
    //	ESP_LOGI(TAG, "CB_stopped");
}

void CB_post_sync(CO_Data *d)
{
    //    ESP_LOGI(TAG, "CB_post_sync");
}

void CB_post_TPDO(CO_Data *d)
{
    //	ESP_LOGI(TAG, "CB_post_TPDO");
}

void CB_storeODSubIndex(CO_Data *d, UNS16 wIndex, UNS8 bSubindex)
{
    /*TODO :
     * - call getODEntry for index and subindex,
     * - save content to file, database, flash, nvram, ...
     *
     * To ease flash organisation, index of variable to store
     * can be established by scanning d->objdict[d->ObjdictSize]
     * for variables to store.
     *
     * */
    ESP_LOGI(TAG, "CB_storeODSubIndex : %4.4x %2.2x", wIndex, bSubindex);
}

void CB_post_emcy(CO_Data *d, UNS8 nodeID, UNS16 errCode, UNS8 errReg)
{
    ESP_LOGI(TAG, "Slave received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x", nodeID, errCode, errReg);
}

/**
 * @brief  Master_post_SlaveBootup.
 * @param  d:CANOpen object dictionary.nodeId:Node id
 * @retval None
 */
void Master_post_SlaveBootup(CO_Data *d, UNS8 nodeid)
{
    masterSendNMTstateChange(d, nodeid, NMT_Start_Node);
}
/**
 * @brief  Master_post_SlaveBootup.
 * @param  d:CANOpen object dictionary.nodeId:Node id newNodeState:Slave change state
 * @retval None
 */
void Master_post_SlaveStateChange(CO_Data *d, UNS8 nodeId, e_nodeState newNodeState)
{
    static UNS8 NodeIdNum = 0;
    if (newNodeState == Operational)
    {
        NodeIdNum++;
        if (NodeIdNum == MAX_CAN_BUS_ID)
        {
            NodeIdNum = 0;
            setState(d, Operational);
        }
    }
}

void canProcBeforeDispatch(Message *m)
{
    UNS16 cob_id = m->cob_id;
    switch (cob_id >> 7)
    {
    case SYNC: /* can be a SYNC or a EMCY message */
        break;
    /* case TIME_STAMP: */
    case PDO1tx:
    case PDO1rx:
    case PDO2tx:
    case PDO2rx:
    case PDO3tx:
    case PDO3rx:
    case PDO4tx:
    case PDO4rx:
        break;
    case SDOtx:
    case SDOrx:
        break;
    case NODE_GUARD:
        break;
    case NMT:
        break;
    }
}

void canProcAfterDispatch(Message *m)
{
    uint16_t cob_id = m->cob_id;
    switch (cob_id >> 7)
    {
    case SYNC: /* can be a SYNC or a EMCY message */
        break;
    /* case TIME_STAMP: */
    case PDO1tx:
    case PDO1rx:
    case PDO2tx:
    case PDO2rx:
    case PDO3tx:
    case PDO3rx:
    case PDO4tx:
    case PDO4rx:
        //			IOWriteDout(Write_Outputs_16_Bit[0]);
        break;
    case SDOtx:
    case SDOrx:
        break;
    case NODE_GUARD:
        break;
    case NMT:
        break;
    }
}

uint8_t isCANOpenConnected(void)
{
    return (getState(pCoData) == Operational);
}

void SetCallBack(CO_Data *d)
{
    d->heartbeatError = CB_heartbeatError;
    d->initialisation = CB_initialisation;
    d->preOperational = CB_preOperational;
    d->operational = CB_operational;
    d->stopped = CB_stopped;
    d->post_sync = CB_post_sync;
    d->post_TPDO = CB_post_TPDO;
    d->post_emcy = CB_post_emcy;
    d->storeODSubIndex = CB_storeODSubIndex;
}

static void canopeninit(uint8_t Setnodeid)
{
    pCoData = &esp32xxx_Data;
    initTimer(); // Start timer for the CANopen stack
    SetCallBack(pCoData);
    setNodeId(pCoData, Setnodeid);
    setState(pCoData, Initialisation); // Init the state
    setState(pCoData, Operational);
}

void ChangeNodeID(uint8_t newid)
{
    setState(pCoData, Stopped);
    setNodeId(pCoData, newid);
    ///	setState(pCoData, Pre_operational);
    setState(pCoData, Initialisation); // Init the state
    setState(pCoData, Operational);
}

static void twai_receive_task(void *arg)
{
    while (1)
    {
        twai_message_t rx_msg;
        Message rxm = {0};
        input_data2++;
        twai_receive(&rx_msg, portMAX_DELAY);
        rxm.cob_id = rx_msg.identifier;
        rxm.rtr = rx_msg.rtr;
        rxm.len = rx_msg.data_length_code;
        for (int i = 0; i < rxm.len; i++)
            rxm.data[i] = rx_msg.data[i];
        canDispatch(pCoData, &rxm);
    }
}

void espcan_init(void)
{
#ifdef CAN_POWER_NEED_CONTROL
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_16) | (1ULL << GPIO_NUM_23);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_NUM_16, 1);
    gpio_set_level(GPIO_NUM_23, 0);
#endif

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "Driver installed");

    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "Driver started");

    canopeninit(CONFIG_NodeID);

    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
}

// The driver send a CAN message passed from the CANopen stack
// 0-succcess
// 1-failed
unsigned char canSend(CAN_PORT notused, Message *m)
{
    unsigned char ret = 0;
    twai_message_t msg = {0};
    msg.identifier = m->cob_id;
    msg.rtr = m->rtr;
    msg.data_length_code = m->len;
    memcpy(&msg.data, &m->data, msg.data_length_code);
    esp_err_t err = twai_transmit(&msg, portMAX_DELAY);
    if (err != ESP_OK)
        ret = 1;
    return ret;
}

// The driver pass a received CAN message to the stack
/*
unsigned char canReceive(Message *m)
{
}
*/
unsigned char canChangeBaudRate_driver(CAN_HANDLE fd, char *baud)
{
    return 0;
}

// void CANOpen_MSG_RCV(_CANMSG *msg)
// {
//     int i;
//     rxm.cob_id = msg->id;
//     rxm.rtr = msg->rtr;
//     rxm.len = msg->len;
//     for (i = 0; i < rxm.len; i++)
//         rxm.data[i] = msg->buffer[i];
//     canDispatch(pCoData, &rxm);
//     //	canProcAfterDispatch(&rxm);
// }