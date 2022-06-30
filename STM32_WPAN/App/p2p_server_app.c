/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    p2p_server_app.c
 * @author  MCD Application Team
 * @brief   peer to peer Server Application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_ble.h"
#include "stm32wbxx_ll_lpuart.h"
#include "stm_list.h"
#include "stm32_lpm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef struct{
    uint8_t             Device_Led_Selection;
    uint8_t             Led1;
 }P2P_LedCharValue_t;

 typedef struct{
    uint8_t             Device_Button_Selection;
    uint8_t             ButtonStatus;
 }P2P_ButtonCharValue_t;

typedef struct
{
  uint8_t               Notification_Status; /* used to chek if P2P Server is enabled to Notify */
  P2P_LedCharValue_t    LedControl;
  P2P_ButtonCharValue_t ButtonControl;
  uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;

// Ref. UM1052 rev9, Table 40: FRAME_START byte
typedef PACKED_STRUCT // use Bit Fields
{
  uint8_t frame_id:5; // The least significant 5 bits indicate the frame identifier.
  uint8_t motor:3; // The most significant 3 bits indicate the motor selection.
}frame_start_t; // Frame_start: this byte defines the type of starting frame.

typedef PACKED_STRUCT{
  frame_start_t frame_start;
  
  uint8_t payload_length; // Payload_Length: the total number of bytes that compose the frame payload
  
  uint8_t payload_id; // Payload_ID: first byte of the payload that contains the identifier of payload.
  // Not necessary if not required by this type of frame. 
}gsf_header_t; // Generic starting frame header

// Ref. UM1052 rev9, Table 39: Generic starting frame
typedef PACKED_STRUCT // use Bit Fields
{
  tListNode node;
  
  gsf_header_t header;
  uint8_t payload[BLE_MAX_LENGTH - sizeof(gsf_header_t)]; // payload with CRC
}gsf_t; // Generic starting frame type

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_QUEUE_SIZE 16
#define MOTOR_RESET_TIMEOUT            (1*1000*1000/CFG_TS_TICK_VAL) /**< 1s */
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/**
 * START of Section BLE_APP_CONTEXT
 */

PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_Server_App_Context_t P2P_Server_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */
static tListNode m_tx_list;

static gsf_t * mp_generic_starting_frame;
static gsf_t m_temp[TX_QUEUE_SIZE];
static uint8_t m_idx;

static uint8_t m_uart_rx[BLE_MAX_LENGTH] = {0,};
static uint8_t m_uart_rx_waiting_payload = 0;

static uint8_t m_motor_reset_ctrl_id;
/* USER CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef hlpuart1;
/* USER CODE END EV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void P2PS_Send_Notification(void);
static void P2PS_APP_LED_BUTTON_context_Init(void);
static void P2PS_APP_frame_received_task(void);
static void P2PS_APP_ack_received_task(void);
static void P2PS_DEBUG_array2string(uint8_t *p_str, uint8_t *p_arr, uint8_t size);
static void motor_gpio_callback(void);
static void motor_uart_init(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */
  const uint8_t dead[] = {0xDE, 0xAD, 0xBE, 0xEF};
  const size_t sz = sizeof(dead);
/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
    case P2PS_STM_BOOT_REQUEST_EVT:
      APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
      APP_DBG_MSG(" \n\r");

      *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
      NVIC_SystemReset();
      break;
#endif
/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 1;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n"); 
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 0;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
      if(memcmp(dead, pNotification->DataTransfered.pPayload, sz) == 0){
        switch(pNotification->DataTransfered.pPayload[sz]){
          case 'B': // [STM] reset Bluetooth
            NVIC_SystemReset();
//            HAL_UART_MspDeInit(&hlpuart1);
//            
//            APP_BLE_Init( );
//            UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
//            MX_LPUART1_UART_Init();
//            BSP_LED_Off(LED_RED);
            break;
            
          case 'M': // [STM] reset Motor
            motor_uart_init();
            HW_TS_Start(m_motor_reset_ctrl_id, (uint32_t)MOTOR_RESET_TIMEOUT);
            BSP_LED_Off(LED_GREEN);
            break;
            
          default:
            break;
        }
      }
      else{
        m_temp[m_idx].node.prev = m_temp[m_idx].node.next = 0;
        memcpy(&m_temp[m_idx].header, pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
        LST_insert_tail(&m_tx_list, (tListNode *)&m_temp[m_idx]);
        m_idx = (m_idx + 1) % TX_QUEUE_SIZE;
        UTIL_SEQ_SetTask( 1<<CFG_TASK_FRAME_RECEIVED_ID, CFG_SCH_PRIO_0);
        BSP_LED_On(LED_RED);
      }
#if(P2P_SERVER1 != 0)
#endif
#if(P2P_SERVER2 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x02){ /* end device 2 selected */ 
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }   
      }
#endif      
#if(P2P_SERVER3 != 0)  
      if(pNotification->DataTransfered.pPayload[0] == 0x03){ /* end device 3 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER4 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x04){ /* end device 4 selected */ 
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }   
      }
#endif     
#if(P2P_SERVER5 != 0)  
      if(pNotification->DataTransfered.pPayload[0] == 0x05){ /* end device 5 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER6 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x06){ /* end device 6 selected */ 
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }   
      }
#endif 
/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */
      
/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
          
/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
       P2PS_APP_LED_BUTTON_context_Init();       
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
  //UTIL_SEQ_RegTask( 1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_FRAME_RECEIVED_ID, UTIL_SEQ_RFU, P2PS_APP_frame_received_task );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_ACK_RECEIVED_ID, UTIL_SEQ_RFU, P2PS_APP_ack_received_task );
  
  /**
   * Initialize LedButton Service
   */
  P2P_Server_App_Context.Notification_Status=0; 
  P2PS_APP_LED_BUTTON_context_Init();
  
  motor_uart_init();
  
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &m_motor_reset_ctrl_id, hw_ts_SingleShot, motor_gpio_callback);
  
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_TX_WRITE_RSP_ID);
/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void P2PS_APP_LED_BUTTON_context_Init(void){
  
  BSP_LED_Off(LED_RED);
  
  #if(P2P_SERVER1 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x01; /* Device1 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x01;/* Device1 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER2 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x02; /* Device2 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x02;/* Device2 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif  
#if(P2P_SERVER3 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x03; /* Device3 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x03; /* Device3 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER4 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif  
 #if(P2P_SERVER5 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER6 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x06; /* device6 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x06; /* Device6 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif  
}

void P2PS_APP_SW1_Button_Action(void)
{
  //UTIL_SEQ_SetTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}

void clear_lpuart_rx(void){
  while (LL_LPUART_IsActiveFlag_RXNE(hlpuart1.Instance))
  {
    /* Read Received character. RXNE flag is cleared by reading of RDR register */
    (void) LL_LPUART_ReceiveData8(hlpuart1.Instance);
  }
  
  /* Clear OVERRUN flag */
  LL_LPUART_ClearFlag_ORE(hlpuart1.Instance);
  
  /* Make sure that no LPUART transfer is on-going */
  while (LL_LPUART_IsActiveFlag_BUSY(hlpuart1.Instance) == 1);
  
  /* Make sure that LPUART is ready to receive */
  while (LL_LPUART_IsActiveFlag_REACK(hlpuart1.Instance) == 0);
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void P2PS_Send_Notification(void)
{
 
  if(P2P_Server_App_Context.ButtonControl.ButtonStatus == 0x00){
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x01;
  } else {
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
  }
  
   if(P2P_Server_App_Context.Notification_Status){ 
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : INFORM CLIENT BUTTON 1 PUSHED \n ");
    APP_DBG_MSG(" \n\r");
    P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.ButtonControl);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n "); 
   }

  return;
}

static void P2PS_APP_frame_received_task(void)
{
  HAL_StatusTypeDef hal_status;
  uint8_t str[BLE_MAX_LENGTH*2 + 1] = {0,};
  uint8_t *p_data;
  uint16_t size;
  
  UTIL_SEQ_WaitEvt(1 << CFG_IDLEEVT_TX_WRITE_RSP_ID);
  
  LST_remove_head(&m_tx_list, (tListNode **)&mp_generic_starting_frame);
  
  p_data = (uint8_t *)&mp_generic_starting_frame->header;
  size = sizeof(gsf_header_t) + mp_generic_starting_frame->header.payload_length;
  
  clear_lpuart_rx();
  hal_status = HAL_UART_Receive_DMA(&hlpuart1, m_uart_rx, 2); // Read Acknowledgment frame, read header
  m_uart_rx_waiting_payload = 0;
  
  if(hal_status == HAL_OK){
    // [STM] The uart interrupt shall be disable when the HAL is called to send data
    // This is because in the Rx uart handler, the HAL is called to receive new data
    BACKUP_PRIMASK();
    DISABLE_IRQ();
    hal_status |= HAL_UART_Transmit_DMA(&hlpuart1, p_data, size);
    RESTORE_PRIMASK();
  }
  
  if(hal_status != HAL_OK){
    APP_DBG_MSG("LPUART HAL error %d\n", hal_status);
    while(1){
      HAL_Delay(1000);
      BSP_LED_Toggle(LED_RED);
    }
  }
  
  // debug
  P2PS_DEBUG_array2string(str, p_data, size);
  APP_DBG_MSG("BLE Recieved%s\n", str);
}

static void P2PS_APP_ack_received_task(void)
{
  uint16_t sz = 2 + m_uart_rx[1] + 1;
  BSP_LED_Off(LED_RED);
  (void)P2PS_STM_App_Notify(m_uart_rx, sz);
  
  // debug
  uint8_t str[BLE_MAX_LENGTH*2 + 1] = {0,};
  P2PS_DEBUG_array2string(str, m_uart_rx, sz);
  APP_DBG_MSG("RX Recieved%s\n", str);
  
  UTIL_SEQ_SetEvt(1 << CFG_IDLEEVT_TX_WRITE_RSP_ID);
}

static void P2PS_DEBUG_array2string(uint8_t *p_str, uint8_t *p_arr, uint8_t size)
{
  for(uint8_t k=0; k < size; k++){
    sprintf((char *)p_str, "%s %02X", p_str, p_arr[k]);
  }
}

static void motor_gpio_callback(void){
  BSP_LED_On(LED_GREEN);
}

static void motor_uart_init(void){
  mp_generic_starting_frame = 0;
  m_idx = 0;
  memset(m_temp, 0x00, sizeof(m_temp));
  
  LST_init_head (&m_tx_list);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == LPUART1)
  {
    if(m_uart_rx_waiting_payload == 0){
      HAL_UART_Receive_DMA(&hlpuart1, &m_uart_rx[2], m_uart_rx[1] + 1); // Read Acknowledgment frame, wait for payload with CRC
      m_uart_rx_waiting_payload = 1;
    }
    else{
      UTIL_SEQ_SetTask( 1<<CFG_TASK_ACK_RECEIVED_ID, CFG_SCH_PRIO_0);
    }
  }
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
