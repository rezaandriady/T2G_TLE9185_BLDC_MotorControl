
/***************************************************************************//**
* \file main_cm0.c
*
* \version 1.0
*
* \brief Main example file for CM0plus
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


/*****************************************************************************
* Include files
*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include "TLE9185-reg.h"

/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/
#define TLE9xxx_CMD_WRITE          	0x80
#define TLE9xxx_CMD_READ          	0x00
#define TLE9xxx_CMD_CLEAR          	0x80
#define ACTIVE_MOSFET                   0
#define FW_MOSFET                       1
#define AGC_ACTIVE                      1

// Clock
#define SOURCE_CLOCK_FRQ                80000000ul
#define CORE_CLOCK_FRQ                  80000000ul

// Schedule Timer
#define Timer_DIV_NUM                   1ul

#define Timer_WD_TYPE                   TCPWM0_GRP0_CNT0
#define Timer_WD_PCLK                   PCLK_TCPWM0_CLOCKS0
#define Timer_WD_IRQN                   tcpwm_0_interrupts_0_IRQn

#define Timer_100ms_TYPE                TCPWM0_GRP0_CNT1
#define Timer_100ms_PCLK                PCLK_TCPWM0_CLOCKS1
#define Timer_100ms_IRQN                tcpwm_0_interrupts_1_IRQn

#define Timer_10ms_TYPE                TCPWM0_GRP0_CNT2
#define Timer_10ms_PCLK                PCLK_TCPWM0_CLOCKS2
#define Timer_10ms_IRQN                tcpwm_0_interrupts_2_IRQn

// PWM
#define PWM_DIV_NUM                     0u
#define PWM_PERIOD                      (0x140-1)   // 2MHz / 80 (0x50) = 25kH

#define PWM_A_TYPE                      TCPWM0_GRP0_CNT27
#define PWM_A_PCLK                      PCLK_TCPWM0_CLOCKS27

#define PWM_B_TYPE                      TCPWM0_GRP1_CNT11
#define PWM_B_PCLK                      PCLK_TCPWM0_CLOCKS267

#define PWM_C_TYPE                      TCPWM0_GRP0_CNT31
#define PWM_C_PCLK                      PCLK_TCPWM0_CLOCKS31

// SPI
#define SCB_SPI_BAUDRATE                125000ul /* Please set baudrate value of SPI you want */
#define SCB_SPI_OVERSAMPLING            16ul     /* Please set oversampling of SPI you want */
#define SCB_SPI_CLOCK_FREQ              (SCB_SPI_BAUDRATE * SCB_SPI_OVERSAMPLING)

#define CY_SPI_SCB_TYPE                 SCB6
#define CY_SPI_SCB_PCLK                 PCLK_SCB6_CLOCK
#define CY_SPI_SCB_IRQN                 scb_6_interrupt_IRQn
#define DIVIDER_NO_1                    1u

#define BB_POTI_ANALOG_MACRO        CY_ADC_POT_MACRO
#define BB_POTI_ANALOG_PCLK         CY_ADC_POT_PCLK
#define BB_POTI_ANALOG_INPUT_NO    ((cy_en_adc_pin_address_t)CY_ADC_POT_IN_NO)
#define ADC_OPERATION_FREQUENCY_MAX_IN_HZ       (26670000ul)
#define ANALOG_IN_SAMPLING_TIME_MIN_IN_NS       (412ull)
#define DIV_ROUND_UP(a,b)                       (((a) + (b)/2) / (b))
#define ADC_LOGICAL_CHANNEL                     0

#define HALL_CHANGE_BUFFER_SIZE                 8

/*****************************************************************************
* Global variable definitions (declared in header file with 'extern')
*****************************************************************************/

/*****************************************************************************
* Local type definitions ('typedef')
*****************************************************************************/
typedef struct
{
  uint8_t MODE;
  uint8_t VCC1_OV_MOD;
  bool RSTN_HYS;
  bool I_PEAK_TH;
  uint8_t VCC1_RT;
} M_S_CTRL_cfg_t;

typedef struct
{
  bool WD_STM_EN_1;
  bool SOFT_RESET_RO;
  bool RSTN_DEL;
  bool SH_DISABLE;
  bool VS_OV_SEL;
  bool TSD2_DEL;
} HW_CTRL_cfg_t;

typedef struct
{
  bool SUPPLY_STAT;
  bool TEMP_STAT;
  bool BD_STAT;
  bool SPI_CRC_FAIL;
  bool WD_SDM;
  bool WD_SDM_DISABLE;
  bool INTN_CYC_EN;
} INT_MASK_cfg_t;

typedef struct
{
  uint8_t ICHG;
  uint8_t IDCHG;
} HB_ICHG_cfg_t;

typedef struct
{
  uint8_t TCCP;
  uint8_t TBLANK;
} CCP_BLK_cfg_t;

typedef struct
{
  uint8_t PCHGINIT;
  uint8_t PDCHGINIT;
} PCHG_INIT_cfg_t;

typedef struct
{
  uint8_t TDON;
} TDON_HB_CTRL_cfg_t;

typedef struct
{
  uint8_t TDOFF;
} TDOFF_HB_CTRL_cfg_t;

typedef struct
{
  uint8_t TPCHG1;
  uint8_t TPCHG2;
  uint8_t TPCHG3;
} TPRECHG_cfg_t;

typedef struct
{
  uint8_t ICHGMAX1;
  uint8_t ICHGMAX2;
  uint8_t ICHGMAX3;
  bool HB1IDIAG;
  bool HB2IDIAG;
  bool HB3IDIAG;
} HB_ICHG_MAX_cfg_t;

typedef struct
{
  uint8_t ICHGST1;
  uint8_t ICHGST2;
  uint8_t ICHGST3;
} ST_ICHG_cfg_t;

typedef struct
{
  bool FMODE;
  bool IHOLD;
  bool EN_GEN_CHECK;
  bool AGCFILT;
  bool POCHGDIS;
  bool CPEN;
  bool AGC;
  bool IPCHGADT;
  bool BDOV_REC;
  bool CPSTGA;
  bool FET_LVL;
  bool CPUVTH;
  bool BDFREQ;
} GENCTRL_cfg_t;

typedef struct
{
  uint8_t HS1VDSTH;
  uint8_t HS2VDSTH;
  uint8_t HS3VDSTH;
  bool DEEP_ADAP;
} HS_VDS_cfg_t;

typedef struct
{
  uint8_t LS1VDSTH;
  uint8_t LS2VDSTH;
  uint8_t LS3VDSTH;
  bool TFVDS;
} LS_VDS_cfg_t;

typedef struct
{
  bool OCEN;
  uint8_t CSAG;
  uint8_t OCTH;
  bool CSA_OFF;
  uint8_t OCFILT;
  bool CSD;
  bool CSO_CAP;
  bool PWM_NB;
} CSA_cfg_t;

typedef struct
{
  uint8_t WD_TIMER;
  bool WD_CFG;
  bool WD_STM_EN_0;
  bool CHECKSUM;
} WD_CTRL_cfg_t;

typedef struct
{
  bool PWM_EN;
  bool AFW;
  uint8_t HBmode;
} HBMODE_cfg_t;

typedef struct {
  float dt;
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float prevErr;
} PID_t;

/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/
// Default WD_CTRL config, will be used during TLE9185 transition from Init to Normal Mode
WD_CTRL_cfg_t WD_CTRL_cfg =
{
  .WD_TIMER = 7ul,
  .WD_CFG = 0ul,
  .WD_STM_EN_0 = 1ul,
  .CHECKSUM = 0ul,
};

INT_MASK_cfg_t INT_MASK_cfg =
{
  .SUPPLY_STAT = 0,
  .TEMP_STAT = 0,
  .BD_STAT = 0,
  .SPI_CRC_FAIL = 0,
  .WD_SDM = 0,
  .WD_SDM_DISABLE = 1,
  .INTN_CYC_EN = 0,
};

M_S_CTRL_cfg_t M_S_CTRL_cfg =
{
  .MODE = 0,
  .VCC1_OV_MOD = 0,
  .RSTN_HYS = 0,
  .I_PEAK_TH = 0,
  .VCC1_RT = 0,
};

HW_CTRL_cfg_t HW_CTRL_cfg = 
{
  .WD_STM_EN_1 = 0,
  .SOFT_RESET_RO = 0,
  .RSTN_DEL = 0,
  .SH_DISABLE = 0,
  .VS_OV_SEL = 1,
  .TSD2_DEL = 0,
};

HB_ICHG_cfg_t HB_ICHG_Active_cfg = 
{
  .ICHG = 7,
  .IDCHG = 7,
};

HB_ICHG_cfg_t HB_ICHG_FW_cfg =
{
  .ICHG = 43,
  .IDCHG = 0,
};

CCP_BLK_cfg_t CCP_BLK_Active_cfg = 
{
  .TCCP = 10,
  .TBLANK = 11,
};

CCP_BLK_cfg_t CCP_BLK_FW_cfg = 
{
  .TCCP = 1,
  .TBLANK = 1,
};

PCHG_INIT_cfg_t PCHG_INIT_cfg = 
{
  .PCHGINIT = 23,
  .PDCHGINIT = 44,
};

TDON_HB_CTRL_cfg_t TDON_HB_CTRL_cfg = 
{
  .TDON = 18,
};

TDOFF_HB_CTRL_cfg_t TDOFF_HB_CTRL_cfg = 
{
  .TDOFF = 20,
};

TPRECHG_cfg_t TPRECHG_PCHG_cfg = 
{
  .TPCHG1 = 1,
  .TPCHG2 = 1,
  .TPCHG3 = 1,
};

TPRECHG_cfg_t TPRECHG_PDCHG_cfg = 
{
  .TPCHG1 = 3,
  .TPCHG2 = 3,
  .TPCHG3 = 3,
};

HB_ICHG_MAX_cfg_t HB_ICHG_MAX_cfg = 
{
  .ICHGMAX1 = 3,
  .ICHGMAX2 = 3,
  .ICHGMAX3 = 3,
  .HB1IDIAG = 0,
  .HB2IDIAG = 0,
  .HB3IDIAG = 0,
};

ST_ICHG_cfg_t ST_ICHG_cfg = 
{
  .ICHGST1 = 8,
  .ICHGST2 = 8,
  .ICHGST3 = 8,
};

HS_VDS_cfg_t HS_VDS_cfg = 
{
  .HS1VDSTH = 7,
  .HS2VDSTH = 7,
  .HS3VDSTH = 7,
  .DEEP_ADAP = 1,
};

LS_VDS_cfg_t LS_VDS_cfg = 
{
  .LS1VDSTH = 7,
  .LS2VDSTH = 7,
  .LS3VDSTH = 7,
  .TFVDS = 0,
};

HBMODE_cfg_t Active_PWM =
{
  .PWM_EN = 1,
  .AFW = 1,
  .HBmode = 2,
};

HBMODE_cfg_t Active_GND =
{
  .PWM_EN = 0,
  .AFW = 0,
  .HBmode = 1,
};

HBMODE_cfg_t Floating =
{
  .PWM_EN = 0,
  .AFW = 0,
  .HBmode = 3, // suspect -> if set to 3, current will rise when lock //not proven
};

PID_t pid =
{
  .dt = 0.1f,
  .Kp = 1,
  .Ki = 0,
  .Kd = 0.1,
  .integral = 0.0f,
  .prevErr = 0.0f,
};

bool SPI_Pending = 0;
uint8_t SPI_Read_data[4];
uint8_t SPI_Send_data[4];

uint8_t const hall_state_table[] = {6, 4, 5, 2, 1, 3};
uint8_t const hall_state_table_fast[] = {5, 3, 4, 1, 6, 2};

bool    motor_dir = 1;
uint8_t bldc_state = 6;
uint32_t PWM_DC = 50;
uint32_t PWM_CC0 = 1;
uint8_t hall_state = 0;
uint8_t old_hall_state = 6;

uint8_t hall_buffer_idx = 8;
uint32_t hall_change_count = 0;
uint32_t hall_change_count_temp_buff[HALL_CHANGE_BUFFER_SIZE] = {0};
uint32_t hall_change_count_temp = 0;
uint32_t hall_RPM = 0;

uint16_t WD_CTRL_data;

uint16_t POT_ADC_RES;
cy_stc_adc_ch_status_t POT_ADC_STAT;

uint16_t CSO_ADC_RES;
cy_stc_adc_ch_status_t CSO_ADC_STAT;

uint32_t CSO_ADC_THRESH = 0xBFF;
uint8_t OCD_CNT;
uint8_t OCD_CNT_THRESH = 50;
bool OCD_ACT_RESET = 1;

uint32_t hall_count_target;

bool BLDC_reset = 0;
bool BLDC_rampup;
bool BLDC_running;

/*****************************************************************************
* Local function prototypes ('static')                                                                            
*****************************************************************************/
void SetPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum);

void TLE9185_Reset();
void TLE9185_Init();
void TLE9185_EnCRC();
void TLE9185_DisCRC();
void TLE9185_M_S_CTRL_Set(M_S_CTRL_cfg_t M_S_CTRL_cfg);
void TLE9185_HW_CTRL_Set(HW_CTRL_cfg_t HW_CTRL_cfg);
void TLE9185_HB_ICHG_Set(HB_ICHG_cfg_t HB_ICHG_cfg, bool ACTorFW, uint8_t HB);
void TLE9185_CCP_BLK_Set(CCP_BLK_cfg_t CCP_BLK_cfg, bool ACTorFW, uint8_t HB);
void TLE9185_PCHG_INIT_Set(PCHG_INIT_cfg_t PCHG_INIT_cfg, uint8_t init_bnk);
void TLE9185_TDON_HB_CTRL_Set(TDON_HB_CTRL_cfg_t TDON_HB_CTRL_cfg, uint8_t hb_tdon_bnk);
void TLE9185_TDOFF_HB_CTRL_Set(TDOFF_HB_CTRL_cfg_t TDOFF_HB_CTRL_cfg, uint8_t hb_tdoff_bnk);
void TLE9185_TPRECHG_Set(uint8_t t_pchgx, uint8_t t_pdchgx);
void TLE9185_HB_ICHG_MAX_Set(uint8_t hbxidiag, uint8_t ichgMaxx);
void TLE9185_ST_ICHG_Set(uint8_t ichgst1, uint8_t ichgst2, uint8_t ichgst3);
void TLE9185_GENCTRL_Set();		
void TLE9185_HS_VDS_Set(uint8_t vdsth, bool deep_adap, uint8_t tfvds);
void TLE9185_LS_VDS_Set(uint8_t vdsth, bool deep_adap, uint8_t tfvds);
void TLE9185_CSA_Set(uint8_t octh, uint8_t csag, bool ocen);
void TLE9185_WD_CTRL_Set(WD_CTRL_cfg_t WD_CTRL_cfg);
void TLE9185_INT_MASK_Set(INT_MASK_cfg_t INT_MASK_cfg);
void TLE9185_HBMODE_Set(HBMODE_cfg_t PWMA, HBMODE_cfg_t PWMB, HBMODE_cfg_t PWMC);

void Timer_Init();
void Timer_100ms_Handler();
void Timer_10ms_Handler();
void Timer_WD_Handler();

void Hall_Init();
void Hall_RPM_Count();

void PWM_Init();
void PWM_BLDC_Control();

void PID_Reset(PID_t *controller);
int PID_Update(PID_t *pid, int actual, int target);
int clamp(int val, int min, int max);

void SPI_Init();
void SPI_WriteReg(uint8_t addr, uint16_t data);
void SPI_ReadReg(uint8_t addr);
void SPI_ISR(void);

void SPI_HBMODE_Set(HBMODE_cfg_t stateA, HBMODE_cfg_t stateB, HBMODE_cfg_t stateC);

void ADC_Init(void);
void POT_ADC_ISR(void);
void CSO_ADC_ISR(void);

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/

/**
*****************************************************************************
** Main function configures the HW and tasks and starts the OS
**
** \return none
*****************************************************************************/
int main(void)
{
  SystemInit();
  
  __enable_irq(); /* Enable global interrupts. */
  
  SPI_Init();
  
  TLE9185_Init(AGC_ACTIVE);
  
  ADC_Init();
  Timer_Init();
  
  // PWM Initialization conducted after Init to disable SPI CRC (PWM1/CRC pin = HIGH_Z)
  PWM_Init();
  
  Hall_Init();
  
  for(;;)
  {
    Cy_SysTick_DelayInUs(10);
    PWM_BLDC_Control();
    
    if (BLDC_reset)
    {
      PID_Reset(&pid);

      Cy_GPIO_Write(GPIO_PRT21, 5, 1);

      hall_change_count = 0;
      hall_change_count_temp = 0;
      hall_RPM = 0;
      PWM_CC0 = 1;
      BLDC_rampup = 1;
      hall_state = (GPIO_PRT7->unIN.u32Register >> 3);

      TLE9185_Reset();
    }
  }
}

void TLE9185_Reset()
{
  TLE9185_Init(AGC_ACTIVE);
  BLDC_reset = 0;
};

void TLE9185_Init()
{ 
  // TLE9185 in Init Mode, SPI write to transition into Normal Mode
  // Recommended to write into WD_CTRL as the first SPI command in Init
  TLE9185_WD_CTRL_Set(WD_CTRL_cfg);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_DisCRC();
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_HW_CTRL_Set(HW_CTRL_cfg);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_M_S_CTRL_Set(M_S_CTRL_cfg);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_INT_MASK_Set(INT_MASK_cfg);
  Cy_SysTick_DelayInUs(2000);
  
  for(uint8_t i = 0; i <= 2; i++)
  {
    TLE9185_HB_ICHG_Set(HB_ICHG_Active_cfg, ACTIVE_MOSFET, i);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_HB_ICHG);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_DEV_STAT);
    Cy_SysTick_DelayInUs(2000);
    
    TLE9185_HB_ICHG_Set(HB_ICHG_FW_cfg, FW_MOSFET, i);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_HB_ICHG);
    Cy_SysTick_DelayInUs(2000);
    
    TLE9185_CCP_BLK_Set(CCP_BLK_Active_cfg, ACTIVE_MOSFET, i);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_CCP_BLK);
    Cy_SysTick_DelayInUs(2000);
    
    TLE9185_CCP_BLK_Set(CCP_BLK_FW_cfg, FW_MOSFET, i);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_CCP_BLK);
    Cy_SysTick_DelayInUs(2000);
    
    TLE9185_PCHG_INIT_Set(PCHG_INIT_cfg, i);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_HB_PCHG_INIT);
    Cy_SysTick_DelayInUs(2000);
    
    TLE9185_TDON_HB_CTRL_Set(TDON_HB_CTRL_cfg, i);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_TDON_HB_CTRL);
    Cy_SysTick_DelayInUs(2000);
    
    TLE9185_TDOFF_HB_CTRL_Set(TDOFF_HB_CTRL_cfg, i);
    Cy_SysTick_DelayInUs(2000);
    SPI_ReadReg(REG_ADDR_TDOFF_HB_CTRL);
    Cy_SysTick_DelayInUs(2000);
  }
  
  TLE9185_TPRECHG_Set(1, 3);
  Cy_SysTick_DelayInUs(2000);
  SPI_ReadReg(REG_ADDR_TPRECHG);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_HB_ICHG_MAX_Set(0, 3);
  Cy_SysTick_DelayInUs(2000);
  SPI_ReadReg(REG_ADDR_HB_ICHG_MAX);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_ST_ICHG_Set(8, 8, 8);
  Cy_SysTick_DelayInUs(2000);
  SPI_ReadReg(REG_ADDR_ST_ICHG);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_GENCTRL_Set();
  Cy_SysTick_DelayInUs(2000);
  SPI_ReadReg(REG_ADDR_GENCTRL);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_HS_VDS_Set(7, 1, 0);
  Cy_SysTick_DelayInUs(2000);
  SPI_ReadReg(REG_ADDR_HS_VDS);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_LS_VDS_Set(7, 1, 0);
  Cy_SysTick_DelayInUs(2000);
  SPI_ReadReg(REG_ADDR_LS_VDS);
  Cy_SysTick_DelayInUs(2000);
  
  TLE9185_CSA_Set(3, 1, 0);
  Cy_SysTick_DelayInUs(2000);
  SPI_ReadReg(REG_ADDR_CSA);
  Cy_SysTick_DelayInUs(2000);
  
  SPI_WriteReg(REG_ADDR_DEV_STAT, 0);
  Cy_SysTick_DelayInUs(2000);
  
  SPI_WriteReg(REG_ADDR_SUP_STAT, 0);
  Cy_SysTick_DelayInUs(2000);
};

void TLE9185_EnCRC()
{
  SPI_Send_data[0] = 0xE7;
  SPI_Send_data[1] = 0xAA;
  SPI_Send_data[2] = 0xAA;
  SPI_Send_data[3] = 0x0E;
  Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE, (void*)(SPI_Send_data), 4);
};

void TLE9185_DisCRC()
{  
  Cy_GPIO_Write(GPIO_PRT21, 5, 0);
  
  SPI_Send_data[0] = 0xE7;
  SPI_Send_data[1] = 0xAA;
  SPI_Send_data[2] = 0xAA;
  SPI_Send_data[3] = 0xC3;
  Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE, (void*)(SPI_Send_data), 4);
};

void TLE9185_HW_CTRL_Set(HW_CTRL_cfg_t HW_CTRL_cfg)
{
  uint16_t ToSend = ((HW_CTRL_cfg.TSD2_DEL << 12) | (HW_CTRL_cfg.VS_OV_SEL << 11) | (HW_CTRL_cfg.SH_DISABLE << 10) | (HW_CTRL_cfg.RSTN_DEL << 9) | (HW_CTRL_cfg.SOFT_RESET_RO << 6) | (HW_CTRL_cfg.WD_STM_EN_1 << 2));
  SPI_WriteReg(REG_ADDR_HW_CTRL, ToSend);
}

void TLE9185_M_S_CTRL_Set(M_S_CTRL_cfg_t M_S_CTRL_cfg)
{
  uint16_t ToSend = ((M_S_CTRL_cfg.MODE << 14) | (M_S_CTRL_cfg.VCC1_OV_MOD << 9) | (M_S_CTRL_cfg.RSTN_HYS << 7) | (M_S_CTRL_cfg.I_PEAK_TH << 5) | (M_S_CTRL_cfg.VCC1_RT << 0));
  SPI_WriteReg(REG_ADDR_M_S_CTRL, ToSend);
}

void TLE9185_INT_MASK_Set(INT_MASK_cfg_t INT_MASK_cfg)
{
  uint16_t ToSend = ((INT_MASK_cfg.INTN_CYC_EN << 8) | (INT_MASK_cfg.WD_SDM_DISABLE << 7) | (INT_MASK_cfg.WD_SDM << 6) | (INT_MASK_cfg.SPI_CRC_FAIL << 5) | (INT_MASK_cfg.BD_STAT << 4) | (INT_MASK_cfg.TEMP_STAT << 1) | (INT_MASK_cfg.SUPPLY_STAT << 0));
  SPI_WriteReg(REG_ADDR_INT_MASK, ToSend);
}

void TLE9185_HB_ICHG_Set(HB_ICHG_cfg_t HB_ICHG_cfg, bool ACTorFW, uint8_t HB)
{
  
  uint16_t ToSend  = ((HB_ICHG_cfg.IDCHG<<10) & 0xFC00) | ((HB_ICHG_cfg.ICHG<<4) & 0x03F0) | (((ACTorFW<<2)|HB)& 0x000F);
  SPI_WriteReg(REG_ADDR_HB_ICHG, ToSend);
};

void TLE9185_CCP_BLK_Set(CCP_BLK_cfg_t CCP_BLK_cfg, bool ACTorFW, uint8_t HB)
{
  uint16_t ToSend = ((CCP_BLK_cfg.TBLANK<<12)&0xF000) | ((CCP_BLK_cfg.TCCP<<8)&0xF00) | (((ACTorFW<<2)|HB)& 0x000F);
  SPI_WriteReg(REG_ADDR_CCP_BLK, ToSend);
};

void TLE9185_PCHG_INIT_Set(PCHG_INIT_cfg_t PCHG_INIT_cfg, uint8_t init_bnk)
{
  uint16_t ToSend  = ((PCHG_INIT_cfg.PDCHGINIT<<10) & 0xFC00) | ((PCHG_INIT_cfg.PCHGINIT<<4) & 0x03F0) | (init_bnk& 0x7);
  SPI_WriteReg(REG_ADDR_HB_PCHG_INIT, ToSend);
};

void TLE9185_TDON_HB_CTRL_Set(TDON_HB_CTRL_cfg_t TDON_HB_CTRL_cfg, uint8_t hb_tdon_bnk)
{
  uint16_t ToSend  = ((TDON_HB_CTRL_cfg.TDON<<8) & 0x3F00) | (hb_tdon_bnk & 0x7);
  SPI_WriteReg(REG_ADDR_TDON_HB_CTRL, ToSend);
};

void TLE9185_TDOFF_HB_CTRL_Set(TDOFF_HB_CTRL_cfg_t TDOFF_HB_CTRL_cfg, uint8_t hb_tdoff_bnk)
{
  uint16_t ToSend  = ((TDOFF_HB_CTRL_cfg.TDOFF<<8) & 0x3F00) | (hb_tdoff_bnk & 0x7);
  SPI_WriteReg(REG_ADDR_TDOFF_HB_CTRL, ToSend);
};

void TLE9185_TPRECHG_Set(uint8_t t_pchgx, uint8_t t_pdchgx)
{
  uint16_t ToSend = 0;
  ToSend = ((t_pchgx << 10) & 0x1C00)| ((t_pchgx << 7) & 0x380)| ((t_pchgx << 4) & 0x70);
  SPI_WriteReg(REG_ADDR_TPRECHG, ToSend);				// Set Precharge time
  
  ToSend = ((t_pdchgx << 10) & 0x1C00)| ((t_pdchgx << 7) & 0x380)| ((t_pdchgx << 4) & 0x70) | (0x001);
  SPI_WriteReg(REG_ADDR_TPRECHG, ToSend);	
};

void TLE9185_HB_ICHG_MAX_Set(uint8_t hbxidiag, uint8_t ichgMaxx)
{
  uint16_t ToSend = 0;
  
  for(uint8_t i=0; i < 3; i++)
  {
    ToSend = ToSend | ((hbxidiag<<(i+12)) |  (ichgMaxx<<(i*2)));
  }
  
  SPI_WriteReg(REG_ADDR_HB_ICHG_MAX, ToSend);
};

void TLE9185_ST_ICHG_Set(uint8_t ichgst1, uint8_t ichgst2, uint8_t ichgst3)
{
  uint16_t ToSend  = ((ichgst3<<8) & 0x0F00) | ((ichgst2<<4) & 0x00F0) | (ichgst1 & 0x000F);
  SPI_WriteReg(REG_ADDR_ST_ICHG, ToSend);
};

void TLE9185_GENCTRL_Set()
{
  uint16_t ToSend = 0;
  ToSend = (1<<15)|(0<<12)|(1<<11)|(1<<10)|(1<<9)|(0<<8)|(1<<6)|(1<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0);
  SPI_WriteReg(REG_ADDR_GENCTRL, ToSend);
};

void TLE9185_HS_VDS_Set(uint8_t vdsth, bool deep_adap, uint8_t tfvds)
{
  uint16_t ToSend = ((deep_adap<<12) & 0x1000) | ((vdsth<<9) & 0xE00) | ((vdsth<<6) & 0x1C0) | ((vdsth<<3) & 0x38) | (vdsth & 0x7);
  SPI_WriteReg(REG_ADDR_HS_VDS, ToSend);
};

void TLE9185_LS_VDS_Set(uint8_t vdsth, bool deep_adap, uint8_t tfvds)
{
  uint16_t ToSend = ((tfvds<<12) & 0x3000) | ((vdsth<<9) & 0xE00) | ((vdsth<<6) & 0x1C0) | ((vdsth<<3) & 0x38) | (vdsth & 0x7);
  SPI_WriteReg(REG_ADDR_LS_VDS, ToSend);
};

void TLE9185_CSA_Set(uint8_t octh, uint8_t csag, bool ocen)
{
  uint16_t ToSend = 0;
  ToSend = ((0<<10)&0x400)|((1<<9)&0x200)|((1<<8)&0x100)|((1<<6)&0xC0)|((0<<5)&0x20)|((3<<3)&0x18)|((2<<1)&0x6)|(0&0x0);
  SPI_WriteReg(REG_ADDR_CSA, ToSend);
};

void TLE9185_WD_CTRL_Set(WD_CTRL_cfg_t WD_CTRL)
{
  WD_CTRL_data = ((WD_CTRL.CHECKSUM << 15) | (WD_CTRL.WD_STM_EN_0 << 6) | (WD_CTRL.WD_CFG << 5) | (WD_CTRL.WD_TIMER));
  SPI_WriteReg(REG_ADDR_WD_CTRL, WD_CTRL_data);
};

void TLE9185_HBMODE_Set(HBMODE_cfg_t PWMA, HBMODE_cfg_t PWMB, HBMODE_cfg_t PWMC)
{
  uint16_t HBMODE_data;
  
  HBMODE_data = (PWMA.HBmode<<2) | (PWMA.AFW<<1) | (PWMA.PWM_EN<<0);
  HBMODE_data |= (PWMB.HBmode<<6) | (PWMB.AFW<<5) | (PWMB.PWM_EN<<4);
  HBMODE_data |= (PWMC.HBmode<<10) | (PWMC.AFW<<9) | (PWMC.PWM_EN)<<8;
  
  SPI_WriteReg(REG_ADDR_HBMODE, HBMODE_data);
}

void Timer_Init()
{
  cy_stc_tcpwm_counter_config_t timer_cfg =
  {
    .period             = 15625ul - 1ul,                        // 15,625 / 15625 = 1Hz
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_128,         // 2,000,000Hz / 128 = 15,625Hz
    .runMode            = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection     = CY_TCPWM_COUNTER_COUNT_UP,
    .debug_pause        = 0uL,
    .compareOrCapture   = CY_TCPWM_COUNTER_MODE_COMPARE,
    .compare0           = 0ul,
    .compare0_buff      = 0ul,
    .compare1           = 0ul,
    .compare1_buff      = 0ul,
    .enableCompare0Swap = false,
    .enableCompare1Swap = false,
    .interruptSources   = CY_TCPWM_INT_NONE,
    .capture0InputMode  = CY_TCPWM_INPUT_LEVEL,
    .capture0Input      = 0ul,
    .reloadInputMode    = CY_TCPWM_INPUT_LEVEL,
    .reloadInput        = 0ul,
    .startInputMode     = CY_TCPWM_INPUT_LEVEL,
    .startInput         = 0ul,
    .stopInputMode      = CY_TCPWM_INPUT_LEVEL,
    .stopInput          = 0ul,
    .capture1InputMode  = CY_TCPWM_INPUT_LEVEL,
    .capture1Input      = 0ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
    .trigger0EventCfg   = CY_TCPWM_COUNTER_OVERFLOW,
    .trigger1EventCfg   = CY_TCPWM_COUNTER_OVERFLOW,
  };
  
  cy_stc_sysint_irq_t timer_irq_cfg = 
  {
    .sysIntSrc  = Timer_WD_IRQN,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
  };
  
  uint32_t targetFreq = 2000000ul;
  uint32_t divNum = (80000000ul / targetFreq);
  CY_ASSERT((80000000ul % targetFreq) == 0ul);
  
  Cy_SysClk_PeriphAssignDivider(Timer_WD_PCLK, CY_SYSCLK_DIV_16_BIT, Timer_DIV_NUM);
  Cy_SysClk_PeriphAssignDivider(Timer_100ms_PCLK, CY_SYSCLK_DIV_16_BIT, Timer_DIV_NUM);
  Cy_SysClk_PeriphAssignDivider(Timer_10ms_PCLK, CY_SYSCLK_DIV_16_BIT, Timer_DIV_NUM);
  Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, Timer_DIV_NUM, (divNum-1ul));  
  Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, Timer_DIV_NUM);
  
  // Timer WD
  Cy_SysInt_InitIRQ(&timer_irq_cfg);
  Cy_SysInt_SetSystemIrqVector(timer_irq_cfg.sysIntSrc, Timer_WD_Handler);
  
  // Timer 100ms
  timer_irq_cfg.sysIntSrc = Timer_100ms_IRQN;
  Cy_SysInt_InitIRQ(&timer_irq_cfg);
  Cy_SysInt_SetSystemIrqVector(timer_irq_cfg.sysIntSrc, Timer_100ms_Handler);
  
  timer_irq_cfg.sysIntSrc = Timer_10ms_IRQN;
  Cy_SysInt_InitIRQ(&timer_irq_cfg);
  Cy_SysInt_SetSystemIrqVector(timer_irq_cfg.sysIntSrc, Timer_10ms_Handler);
  
  NVIC_SetPriority(timer_irq_cfg.intIdx, 3ul);
  NVIC_EnableIRQ(timer_irq_cfg.intIdx);
  
  // Timer WD
  Cy_Tcpwm_Counter_Init(Timer_WD_TYPE, &timer_cfg);
  
  // Timer 100ms
  timer_cfg.period = 3125;
  timer_cfg.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_64, 
  Cy_Tcpwm_Counter_Init(Timer_100ms_TYPE, &timer_cfg);
  
  // Timer 100ms
  timer_cfg.period = 625;
  timer_cfg.clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_32, 
  Cy_Tcpwm_Counter_Init(Timer_10ms_TYPE, &timer_cfg);
  
  Cy_Tcpwm_Counter_Enable(Timer_WD_TYPE);
  Cy_Tcpwm_Counter_Enable(Timer_100ms_TYPE);
  Cy_Tcpwm_Counter_Enable(Timer_10ms_TYPE);
  
  Cy_Tcpwm_Counter_SetTC_IntrMask(Timer_WD_TYPE); 
  Cy_Tcpwm_Counter_SetTC_IntrMask(Timer_100ms_TYPE);
  Cy_Tcpwm_Counter_SetTC_IntrMask(Timer_10ms_TYPE);
  
  Cy_Tcpwm_TriggerStart(Timer_WD_TYPE);
  Cy_Tcpwm_TriggerStart(Timer_100ms_TYPE);
  Cy_Tcpwm_TriggerStart(Timer_10ms_TYPE);
};

void Timer_10ms_Handler(void)
{
  if(Cy_Tcpwm_Counter_GetTC_IntrMasked(Timer_10ms_TYPE) == 1ul)
  {
    Cy_Adc_Channel_SoftwareTrigger(&PASS0_SAR1->CH[ADC_LOGICAL_CHANNEL]);
    Cy_Tcpwm_Counter_ClearTC_Intr(Timer_10ms_TYPE);
  }
}

void Timer_100ms_Handler(void)
{
  if(Cy_Tcpwm_Counter_GetTC_IntrMasked(Timer_100ms_TYPE) == 1ul)
  {
    hall_buffer_idx = (hall_buffer_idx + 1) % HALL_CHANGE_BUFFER_SIZE;
    hall_change_count_temp_buff[hall_buffer_idx] = hall_change_count_temp;
    hall_change_count_temp = 0;
    
    int sum = 0;
    for (int i = 0; i < HALL_CHANGE_BUFFER_SIZE; i++)
    {
      sum += hall_change_count_temp_buff[i];
    }
    hall_change_count = (uint32_t) (sum / HALL_CHANGE_BUFFER_SIZE);    
    
    if (!hall_change_count && hall_count_target)
    {
      BLDC_rampup = 1;
    } else if (hall_change_count)
    {
      BLDC_rampup = 0;
    }
    
    OCD_CNT = 0;
    
    // PID
    PWM_CC0 = clamp(PWM_CC0 + PID_Update(&pid, hall_change_count, hall_count_target), 1, 200);
    
    Cy_Tcpwm_Counter_ClearTC_Intr(Timer_100ms_TYPE);
  }
}

void Timer_WD_Handler(void)
{
  if(Cy_Tcpwm_Counter_GetTC_IntrMasked(Timer_WD_TYPE) == 1ul)
  {
    // Feed TLE9185 Trigger
    //SPI_ReadReg(REG_ADDR_WD_CTRL);
    
    // Count RPM
    Hall_RPM_Count();
    
    Cy_Adc_Channel_SoftwareTrigger(&BB_POTI_ANALOG_MACRO->CH[ADC_LOGICAL_CHANNEL]);
    
    // ADC
    if (POT_ADC_RES > 120)
    {
      hall_count_target = POT_ADC_RES / 120;
    } else
    {
      hall_count_target = 0;
    }
    
    Cy_Tcpwm_Counter_ClearTC_Intr(Timer_WD_TYPE);
  }
};

void Hall_Init()
{
  cy_stc_gpio_pin_config_t const hall_ch_pin_cfg =
  {
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ, //Check
    .hsiom     = 0,
    .intEdge   = CY_GPIO_INTR_DISABLE,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
  };
  
  Cy_GPIO_Pin_Init(GPIO_PRT7, 3, &hall_ch_pin_cfg); //HALL1
  Cy_GPIO_Pin_Init(GPIO_PRT7, 4, &hall_ch_pin_cfg); //HALL2
  Cy_GPIO_Pin_Init(GPIO_PRT7, 5, &hall_ch_pin_cfg); //HALL3
};

void Hall_RPM_Count()
{
  hall_RPM = (hall_change_count << 2);
};

void PWM_Init()
{
  cy_stc_gpio_pin_config_t pwm_ch_pin_cfg =
  {
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = 8,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
  };
  
  cy_stc_tcpwm_pwm_config_t const pwm_ch_cfg =
  {
    .pwmMode            = CY_TCPWM_PWM_MODE_PWM,
    .clockPrescaler     = CY_TCPWM_PRESCALER_DIVBY_1,
    .cc0MatchMode       = CY_TCPWM_PWM_TR_CTRL2_CLEAR,
    .overflowMode       = CY_TCPWM_PWM_TR_CTRL2_SET,
    .runMode            = CY_TCPWM_PWM_CONTINUOUS,
    .period             = PWM_PERIOD,
    .compare0           = PWM_CC0,
    .compare0_buff      = PWM_CC0,
    .enableCompare0Swap = true,
    .killMode           = CY_TCPWM_PWM_STOP_ON_KILL,
    .switchInputMode    = CY_TCPWM_INPUT_RISING_EDGE,
    .switchInput        = 5ul,
    .kill0InputMode     = CY_TCPWM_INPUT_RISING_EDGE,
    .kill1Input         = 6ul,
    .startInputMode     = CY_TCPWM_INPUT_RISING_EDGE,
    .startInput         = 7ul,
    .countInputMode     = CY_TCPWM_INPUT_LEVEL,
    .countInput         = 1ul,
  };
  
  Cy_GPIO_Pin_Init(GPIO_PRT19, 2, &pwm_ch_pin_cfg); //PWMA
  Cy_GPIO_Pin_Init(GPIO_PRT23, 3, &pwm_ch_pin_cfg); //PWMB
  Cy_GPIO_Pin_Init(GPIO_PRT22, 3, &pwm_ch_pin_cfg); //PWMC
  
  uint32_t targetFreq = 8000000ul;
  uint32_t div = (80000000ul/ targetFreq);
  
  Cy_SysClk_PeriphAssignDivider(PWM_A_PCLK, CY_SYSCLK_DIV_16_BIT, PWM_DIV_NUM);
  Cy_SysClk_PeriphAssignDivider(PWM_B_PCLK, CY_SYSCLK_DIV_16_BIT, PWM_DIV_NUM);
  Cy_SysClk_PeriphAssignDivider(PWM_C_PCLK, CY_SYSCLK_DIV_16_BIT, PWM_DIV_NUM);
  
  Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, PWM_DIV_NUM, (div-1ul));
  
  Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, PWM_DIV_NUM);
  
  Cy_Tcpwm_Pwm_Init(PWM_A_TYPE, &pwm_ch_cfg);
  Cy_Tcpwm_Pwm_Init(PWM_B_TYPE, &pwm_ch_cfg);
  Cy_Tcpwm_Pwm_Init(PWM_C_TYPE, &pwm_ch_cfg);
  
  Cy_Tcpwm_Pwm_Enable(PWM_A_TYPE);
  Cy_Tcpwm_Pwm_Enable(PWM_B_TYPE);
  Cy_Tcpwm_Pwm_Enable(PWM_C_TYPE);
};

void PWM_BLDC_Control()
{
  // Read Hall States
  hall_state = (GPIO_PRT7->unIN.u32Register >> 3);

  if (old_hall_state == hall_state && !BLDC_rampup)
  {
    return;
  }
    
  Cy_SysTick_DelayInUs(10);
  
  if (old_hall_state != hall_state)
  {
    hall_change_count_temp++;
  }
  
  PWM_A_TYPE->unTR_CMD.stcField.u1STOP = 1;
  PWM_B_TYPE->unTR_CMD.stcField.u1STOP = 1;
  PWM_C_TYPE->unTR_CMD.stcField.u1STOP = 1;
  
  old_hall_state = hall_state;

  // Next BLDC Sector
  {
    uint8_t hall_state_index = 0;
    for (uint8_t i = 0; i < 6; i++) 
    {
      if (hall_state_table[i] == hall_state) 
      {
        hall_state_index = i;
        break;
      }
    }
    
    if (motor_dir)
    {
      bldc_state = hall_state_table[(hall_state_index == 5)? 0 : hall_state_index+1];
    } else
    {
      bldc_state = hall_state_table[(hall_state_index == 0)? 5 : hall_state_index-1];
    }
  }
  
  switch (bldc_state)
  {
  case 1:
    TLE9185_HBMODE_Set(Active_PWM, Active_GND, Floating);
    PWM_B_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_C_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_A_TYPE->unCC0.stcField.u32CC = PWM_CC0;
    PWM_A_TYPE->unTR_CMD.stcField.u1START = 1;
    break;
    
  case 2:
    TLE9185_HBMODE_Set(Active_PWM, Floating, Active_GND);
    PWM_B_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_C_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_A_TYPE->unCC0.stcField.u32CC = PWM_CC0;
    PWM_A_TYPE->unTR_CMD.stcField.u1START = 1;
    break;
    
  case 3:
    TLE9185_HBMODE_Set(Floating, Active_PWM, Active_GND);
    PWM_A_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_C_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_B_TYPE->unCC0.stcField.u32CC = PWM_CC0;
    PWM_B_TYPE->unTR_CMD.stcField.u1START = 1;
    break;
    
  case 4:
    TLE9185_HBMODE_Set(Active_GND, Active_PWM, Floating);
    PWM_A_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_C_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_B_TYPE->unCC0.stcField.u32CC = PWM_CC0;
    PWM_B_TYPE->unTR_CMD.stcField.u1START = 1;
    break;
    
  case 5:
    TLE9185_HBMODE_Set(Active_GND, Floating, Active_PWM);
    PWM_A_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_B_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_C_TYPE->unCC0.stcField.u32CC = PWM_CC0;
    PWM_C_TYPE->unTR_CMD.stcField.u1START = 1;
    break;
    
  case 6:
    TLE9185_HBMODE_Set(Floating, Active_GND, Active_PWM);
    PWM_A_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_B_TYPE->unTR_CMD.stcField.u1STOP = 1;
    PWM_C_TYPE->unCC0.stcField.u32CC = PWM_CC0;
    PWM_C_TYPE->unTR_CMD.stcField.u1START = 1;
    break;
    
  default:
    break;
  }
  
//    Cy_SysTick_DelayInUs(2000);
    
//    SPI_WriteReg(REG_ADDR_DEV_STAT, 0);
//    Cy_SysTick_DelayInUs(2000);
//    SPI_WriteReg(REG_ADDR_SUP_STAT, 0);
//    Cy_SysTick_DelayInUs(2000);
}

void PID_Reset(PID_t *pid) {
  pid->integral = 0.0f;
  pid->prevErr = 0.0f;
}

int PID_Update(PID_t *pid, int val, int target) {
  float dt = pid->dt;  
  float err = (float)(target - val);
  float P = pid->Kp * err;
  
  pid->integral += err * dt;
  float I = pid->Ki * pid->integral;
  
  float derivative = (pid->prevErr - err) / dt;
  float D = pid->Kd * derivative;
  pid->prevErr = err;
  int pidval = (int)(P + I + D);
  
  return (int)(P + I + D);
}

int clamp(int val, int min, int max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

void SPI_Init()
{
  cy_stc_gpio_pin_config_t spi_pin_cfg =
  {
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = 19,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
  };
  
  cy_stc_gpio_pin_config_t const spi_cs_pin_cfg =
  {
    .outVal    = 1ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = 0,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
  };
  
  static const cy_stc_scb_spi_config_t SCB_SPI_cfg =
  {
    .spiMode                    = CY_SCB_SPI_MASTER,      /*** Specifies the mode of operation    ***/
    .subMode                    = CY_SCB_SPI_MOTOROLA,    /*** Specifies the sub mode of SPI operation    ***/
    .sclkMode                   = CY_SCB_SPI_CPHA1_CPOL0,
    .oversample                 = SCB_SPI_OVERSAMPLING,   /*** SPI_CLOCK divided by SCB_SPI_OVERSAMPLING should be baudrate  ***/
    .rxDataWidth                = 8ul,                   /*** The width of RX data (valid range 4-16). It must be the same as \ref txDataWidth except in National sub-mode. ***/
    .txDataWidth                = 8ul,                   /*** The width of TX data (valid range 4-16). It must be the same as \ref rxDataWidth except in National sub-mode. ***/
    .enableMsbFirst             = false,                   /*** Enables the hardware to shift out the data element MSB first, otherwise, LSB first ***/
    .enableFreeRunSclk          = false,                  /*** Enables the master to generate a continuous SCLK regardless of whether there is data to send  ***/
    .enableInputFilter          = false,                  /*** Enables a digital 3-tap median filter to be applied to the input of the RX FIFO to filter glitches on the line. ***/
    .enableMisoLateSample       = true,                   /*** Enables the master to sample MISO line one half clock later to allow better timings. ***/
    .enableTransferSeperation   = false,                   /*** Enables the master to transmit each data element separated by a de-assertion of the slave select line (only applicable for the master mode) ***/
    .ssPolarity0                = false,                  /*** SS0: active low ***/
    .ssPolarity1                = false,                  /*** SS1: active low ***/
    .ssPolarity2                = false,                  /*** SS2: active low ***/
    .ssPolarity3                = false,                  /*** SS3: active low ***/
    .enableWakeFromSleep        = false,                  /*** When set, the slave will wake the device when the slave select line becomes active. Note that not all SCBs support this mode. Consult the device datasheet to determine which SCBs support wake from deep sleep. ***/
    .rxFifoTriggerLevel         = 1ul,                    /*** Interrupt occurs, when there are more entries of 2 in the RX FIFO */
    .rxFifoIntEnableMask        = 0ul,                    /*** Bits set in this mask will allow events to cause an interrupt  */
    .txFifoTriggerLevel         = 0ul,                    /*** When there are fewer entries in the TX FIFO, then at this level the TX trigger output goes high. This output can be connected to a DMA channel through a trigger mux. Also, it controls the \ref CY_SCB_SPI_TX_TRIGGER interrupt source. */
    .txFifoIntEnableMask        = 0ul,                    /*** Bits set in this mask allow events to cause an interrupt  */
    .masterSlaveIntEnableMask   = 0ul,                    /*** Bits set in this mask allow events to cause an interrupt  */
    .enableSpiDoneInterrupt     = true,
    .enableSpiBusErrorInterrupt = false,
  };
  
  static cy_stc_sysint_irq_t spi_irq_cfg =
  {
    .sysIntSrc  = CY_SPI_SCB_IRQN,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
  };
  
  Cy_GPIO_Pin_Init(GPIO_PRT22, 2, &spi_pin_cfg); //SCL
  Cy_GPIO_Pin_Init(GPIO_PRT22, 1, &spi_pin_cfg); //MOSI
  spi_pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;
  Cy_GPIO_Pin_Init(GPIO_PRT22, 0, &spi_pin_cfg); //MISO
  Cy_GPIO_Pin_Init(GPIO_PRT21, 5, &spi_cs_pin_cfg); //CS
  
  Cy_SysClk_PeriphAssignDivider(CY_SPI_SCB_PCLK, CY_SYSCLK_DIV_24_5_BIT, DIVIDER_NO_1);
  SetPeripheFracDiv24_5(SCB_SPI_CLOCK_FREQ, SOURCE_CLOCK_FRQ, DIVIDER_NO_1);
  Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_24_5_BIT, 1u);
  
  Cy_SCB_SPI_DeInit(CY_SPI_SCB_TYPE);
  
  Cy_SysInt_InitIRQ(&spi_irq_cfg);
  Cy_SysInt_SetSystemIrqVector(spi_irq_cfg.sysIntSrc, SPI_ISR);
  NVIC_EnableIRQ(spi_irq_cfg.intIdx);
  
  Cy_SCB_SPI_Init(CY_SPI_SCB_TYPE, &SCB_SPI_cfg, NULL);
  CY_SPI_SCB_TYPE->unINTR_M_MASK.stcField.u1SPI_DONE = 0x1;

  Cy_SCB_SPI_SetActiveSlaveSelect(CY_SPI_SCB_TYPE, 0ul);
  Cy_SCB_SPI_Enable(CY_SPI_SCB_TYPE);

  Cy_GPIO_Write(GPIO_PRT21, 5, 1);
};

void SPI_WriteReg(uint8_t addr, uint16_t data)
{
  while (SPI_Pending);
  
  SPI_Pending = 1;
  Cy_GPIO_Write(GPIO_PRT21, 5, 0);
  
  SPI_Send_data[0] = (addr | TLE9xxx_CMD_WRITE);
  SPI_Send_data[1] = data & 0xFF;
  SPI_Send_data[2] = (data >> 8) & 0xFF;
  SPI_Send_data[3] = 0xA5;
  Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE, (void*)(SPI_Send_data), 4);
};

void SPI_ReadReg(uint8_t addr)
{
  while (SPI_Pending);
  
  SPI_Pending = 1;
  Cy_GPIO_Write(GPIO_PRT21, 5, 0);
  
  SPI_Send_data[0] = (addr | TLE9xxx_CMD_READ);
  SPI_Send_data[1] = 0xFF;
  SPI_Send_data[2] = 0xFF;
  SPI_Send_data[3] = 0xA5;
  Cy_SCB_SPI_WriteArray(CY_SPI_SCB_TYPE, (void*)(SPI_Send_data), 4);
}

void SPI_ISR(void)
{
  uint32_t SPI_IRQ_Status;
  
  SPI_IRQ_Status = Cy_SCB_SPI_IsTxComplete(CY_SPI_SCB_TYPE);
  if(SPI_IRQ_Status)
  {
    Cy_GPIO_Write(GPIO_PRT21, 5, 1);
    
    Cy_SCB_SPI_ReadArray(CY_SPI_SCB_TYPE, (void*)SPI_Read_data, 4);
    
    SPI_Pending = 0;
    Cy_SCB_SPI_ClearSlaveMasterStatus(CY_SPI_SCB_TYPE, CY_SCB_SPI_MASTER_DONE);
  }
};

void POT_ADC_ISR(void)
{
  cy_stc_adc_interrupt_source_t intrSource = { false };
  Cy_Adc_Channel_GetInterruptMaskedStatus(&BB_POTI_ANALOG_MACRO->CH[ADC_LOGICAL_CHANNEL], &intrSource);
  
  if(intrSource.grpDone)
  {
    Cy_Adc_Channel_GetResult(&BB_POTI_ANALOG_MACRO->CH[ADC_LOGICAL_CHANNEL], &POT_ADC_RES, &POT_ADC_STAT);
    Cy_Adc_Channel_ClearInterruptStatus(&BB_POTI_ANALOG_MACRO->CH[ADC_LOGICAL_CHANNEL], &intrSource);
  }
}

void CSO_ADC_ISR(void)
{
  cy_stc_adc_interrupt_source_t intrSource = { false };
  Cy_Adc_Channel_GetInterruptMaskedStatus(&PASS0_SAR1->CH[ADC_LOGICAL_CHANNEL], &intrSource);
  
  if(intrSource.chRange)
  {
    Cy_Adc_Channel_GetResult(&PASS0_SAR1->CH[ADC_LOGICAL_CHANNEL], &CSO_ADC_RES, &CSO_ADC_STAT);
    
    if (CSO_ADC_RES > CSO_ADC_THRESH)
    {
      OCD_CNT++;
    }

    if (OCD_CNT > OCD_CNT_THRESH) 
    {
      OCD_CNT = 0;
      if (OCD_ACT_RESET)
      {
        BLDC_reset = 1;
      }
    }
    
    Cy_Adc_Channel_ClearInterruptStatus(&PASS0_SAR1->CH[ADC_LOGICAL_CHANNEL], &intrSource);
  }
}

void ADC_Init(void)
{
  cy_stc_gpio_pin_config_t adcPinConfig =
  {
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_ANALOG,
    .hsiom     = CY_ADC_POT_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
  };
  Cy_GPIO_Pin_Init(CY_ADC_POT_PORT, CY_ADC_POT_PIN, &adcPinConfig);
  Cy_GPIO_Pin_Init(GPIO_PRT13, 2, &adcPinConfig);
  
  uint32_t actualAdcOperationFreq;
  {
    uint32_t periFreq = 80000000ul;
    uint32_t divNum = DIV_ROUND_UP(periFreq, ADC_OPERATION_FREQUENCY_MAX_IN_HZ);
    actualAdcOperationFreq = periFreq / divNum;
    Cy_SysClk_PeriphAssignDivider(BB_POTI_ANALOG_PCLK, CY_SYSCLK_DIV_16_BIT, 0ul);
    Cy_SysClk_PeriphAssignDivider(PCLK_PASS0_CLOCK_SAR1, CY_SYSCLK_DIV_16_BIT, 0ul);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0ul, (divNum - 1ul));
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 0ul);
  }
  
  uint32_t samplingCycle = (uint32_t)DIV_ROUND_UP((ANALOG_IN_SAMPLING_TIME_MIN_IN_NS * (uint64_t)actualAdcOperationFreq), 1000000000ull);
  
  cy_stc_adc_config_t adcConfig =
  {
    .preconditionTime    = 0u,
    .powerupTime         = 0u,
    .enableIdlePowerDown = false,
    .msbStretchMode      = CY_ADC_MSB_STRETCH_MODE_1CYCLE,
    .enableHalfLsbConv   = 0u,
    .sarMuxEnable        = true,
    .adcEnable           = true,
    .sarIpEnable         = true,
  };
  
  cy_stc_adc_channel_config_t adcChannelConfig =
  {
    .triggerSelection          = CY_ADC_TRIGGER_OFF,
    .channelPriority           = 0u,
    .preenptionType            = CY_ADC_PREEMPTION_FINISH_RESUME,
    .isGroupEnd                = true,
    .doneLevel                 = CY_ADC_DONE_LEVEL_PULSE,
    .pinAddress                = BB_POTI_ANALOG_INPUT_NO,
    .portAddress               = CY_ADC_PORT_ADDRESS_SARMUX0,
    .extMuxSelect              = 0u,
    .extMuxEnable              = true,
    .preconditionMode          = CY_ADC_PRECONDITION_MODE_OFF,
    .overlapDiagMode           = CY_ADC_OVERLAP_DIAG_MODE_OFF,
    .sampleTime                = samplingCycle,
    .calibrationValueSelect    = CY_ADC_CALIBRATION_VALUE_REGULAR,
    .postProcessingMode        = CY_ADC_POST_PROCESSING_MODE_NONE,
    .resultAlignment           = CY_ADC_RESULT_ALIGNMENT_RIGHT,
    .signExtention             = CY_ADC_SIGN_EXTENTION_UNSIGNED,
    .averageCount              = 0u,
    .rightShift                = 0u,
    .rangeDetectionMode        = CY_ADC_RANGE_DETECTION_MODE_INSIDE_RANGE,
    .rangeDetectionLoThreshold = 0x0000u,
    .rangeDetectionHiThreshold = 0x0FFFu,
    .mask.grpDone              = true,
    .mask.grpCancelled         = false,
    .mask.grpOverflow          = false,
    .mask.chRange              = false,
    .mask.chPulse              = false,
    .mask.chOverflow           = false,
  };
  
  Cy_Adc_Init(BB_POTI_ANALOG_MACRO, &adcConfig);
  Cy_Adc_Channel_Init(&BB_POTI_ANALOG_MACRO->CH[ADC_LOGICAL_CHANNEL], &adcChannelConfig);
  
  adcChannelConfig.pinAddress                   = ((cy_en_adc_pin_address_t)14);
  adcChannelConfig.postProcessingMode           = CY_ADC_POST_PROCESSING_MODE_RANGE,
  adcChannelConfig.rangeDetectionMode           = CY_ADC_RANGE_DETECTION_MODE_OUTSIDE_RANGE;
  adcChannelConfig.rangeDetectionHiThreshold    = 0x0BFF;
  adcChannelConfig.mask.grpDone                 = false;
  adcChannelConfig.mask.chRange                 = true;
  
  Cy_Adc_Init(PASS0_SAR1, &adcConfig);
  Cy_Adc_Channel_Init(&PASS0_SAR1->CH[ADC_LOGICAL_CHANNEL], &adcChannelConfig);
   
  {
    cy_stc_sysint_irq_t irq_cfg;
    irq_cfg = (cy_stc_sysint_irq_t){
      .sysIntSrc  = (cy_en_intr_t)((uint32_t)CY_ADC_POT_IRQN + ADC_LOGICAL_CHANNEL),
      .intIdx     = CPUIntIdx3_IRQn,
      .isEnabled  = true,
    };
    
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, POT_ADC_ISR);
    
    irq_cfg.sysIntSrc = (cy_en_intr_t)((uint32_t)pass_0_interrupts_sar_32_IRQn + ADC_LOGICAL_CHANNEL);
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, CSO_ADC_ISR);
    
    NVIC_SetPriority(irq_cfg.intIdx, 0ul);
    NVIC_EnableIRQ(irq_cfg.intIdx);
  }
  
  Cy_Adc_Channel_Enable(&BB_POTI_ANALOG_MACRO->CH[ADC_LOGICAL_CHANNEL]);
  Cy_Adc_Channel_Enable(&PASS0_SAR1->CH[ADC_LOGICAL_CHANNEL]);
  
  Cy_Adc_Channel_SoftwareTrigger(&BB_POTI_ANALOG_MACRO->CH[ADC_LOGICAL_CHANNEL]);
  Cy_Adc_Channel_SoftwareTrigger(&PASS0_SAR1->CH[ADC_LOGICAL_CHANNEL]);
}

void SetPeripheFracDiv24_5(uint64_t targetFreq, uint64_t sourceFreq, uint8_t divNum)
{
  uint64_t temp = ((uint64_t)sourceFreq << 5ull);
  uint32_t divSetting;
  
  divSetting = (uint32_t)(temp / targetFreq);
  Cy_SysClk_PeriphSetFracDivider(CY_SYSCLK_DIV_24_5_BIT, divNum, 
                                 (((divSetting >> 5ul) & 0x00000FFFul) - 1ul), 
                                 (divSetting & 0x0000001Ful));
};

/* [] END OF FILE */