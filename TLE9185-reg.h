enum CtrlRegisters
{
  /* SPI control registers, device control registers*/
  REG_ADDR_M_S_CTRL 			= (0x01),		/**< Mode and Supply Control*/
  REG_ADDR_HW_CTRL 			= (0x02),		/**< Hardware control*/
  REG_ADDR_WD_CTRL 			= (0x03),		/**< Watchdog control*/
  REG_ADDR_WK_CTRL 			= (0x05),		/**< Wake-up control*/
  REG_ADDR_TIMER_CTRL 		= (0x06),		/**< Timer 1 and Timer 2control and selection*/
  REG_ADDR_INT_MASK 			= (0x09),		/**< Interrupt mask control*/
  REG_ADDR_SYS_STAT_CTRL 		= (0x0B),		/**< System status contorl*/
  /*SPI control registers, contorl registers bridge driver*/
  REG_ADDR_GENCTRL 			= (0x10),		/**< General bridge control*/
  REG_ADDR_CSA 				= (0x11),		/**< Current sense amplifier*/
  REG_ADDR_LS_VDS 			= (0x12),		/**< Drain-Source monitoring threshold*/
  REG_ADDR_HS_VDS 			= (0x13),		/**< Drain-Source monitoring threshold*/
  REG_ADDR_CCP_BLK 			= (0x14),		/**< CCP and times selection*/
  REG_ADDR_HBMODE 			= (0x15),		/**< Half-Bridge MODE*/
  REG_ADDR_TPRECHG 			= (0x16),		/**< PWM pre-charge and pre-discharge time*/	
  REG_ADDR_ST_ICHG 			= (0x17),		/**< Static charge/discharge current*/
  REG_ADDR_HB_ICHG 			= (0x18),		/**< PWM charge/discharge current*/
  REG_ADDR_HB_ICHG_MAX		= (0x19),		/**< PWM max. pre-charge/pre-discharge current*/
  REG_ADDR_HB_PCHG_INIT		= (0x1A),		/**< PWM pre-charge/pre-discharge initialization*/
  REG_ADDR_TDON_HB_CTRL 		= (0x1B),		/**< PWM inputs TON configuration*/
  REG_ADDR_TDOFF_HB_CTRL 		= (0x1C),		/**< PWM inputs TOFF configuration*/
  REG_ADDR_BRAKE	 			= (0x1D),		/**< Brake control*/
};

//! \brief enum for the control registers in a TLE9xxx
enum StatusRegisters
{
  /* SPI status information registers, Device Status Registers */
  REG_ADDR_SUP_STAT			= (0x40),       /**< Supply Voltage Fail Status */
  REG_ADDR_THERM_STAT			= (0x41),       /**< Thermal Protection Status */
  REG_ADDR_DEV_STAT			= (0x42),       /**< Device Information Status */
  REG_ADDR_WK_STAT			= (0x44),       /**< Wake-up Source and Information Status */
  REG_ADDR_WK_LVL_STAT		= (0x45),       /**< WK Input Level */
  /* SPI status information registers, Status registers bridge driver*/
  REG_ADDR_GEN_STAT			= (0x50),       /**< GEN Status register */
  REG_ADDR_TDREG				= (0x51),       /**< Turn-on/off delay regulation register */
  REG_ADDR_DSOV				= (0x52),       /**< Drain-source overvoltage HBVOUT */
  REG_ADDR_EFF_TDON_OFF1		= (0x53),       /**< Effective MOSFET turn-on/off delay - PWM half- bridge 1 */
  REG_ADDR_EFF_TDON_OFF2		= (0x54),       /**< Effective MOSFET turn-on/off delay - PWM half- bridge 2 */
  REG_ADDR_EFF_TDON_OFF3		= (0x55),       /**< Effective MOSFET turn-on/off delay - PWM half- bridge 3 */
  REG_ADDR_TRISE_FALL1		= (0x57),       /**< MOSFET rise/fall time - PWM half-bridge 1 */
  REG_ADDR_TRISE_FALL2		= (0x58),       /**< MOSFET rise/fall time - PWM half-bridge 2 */
  REG_ADDR_TRISE_FALL3		= (0x59),       /**< MOSFET rise/fall time - PWM half-bridge 3 */
  /* SPI status information registers, Family and product information register */
  REG_ADDR_FAM_PROD_STAT		= (0x70)        /**< Family and Product Identification Register */
};

/*============================================================================================================================================*/
/*====================================== Bit field enums =====================================================================================*/
/*============================================================================================================================================*/
enum Int_Mask
{
  SUPPLY_STAT				= (0x01),		/**< Supply status interrupt generation */
  TEMP_STAT				= (0x02),		/**< Temperature Interrupt generation */
  BD_STAT					= (0x10),		/**< BUS interrupt generation */
  SPI_CRC_FAIL			= (0x20),		/**< SPI and CRC interrupt generation */
  WD_SDM					= (0x40),		/**< Watchdog failure in SW Dev mode */
  WD_SDM_DISABLE			= (0x80),		/**< Disable Watchdog in SW Dev mode */
  INTN_CYC_EN				= (0x100)		/**< Periodical INTN generation */
};