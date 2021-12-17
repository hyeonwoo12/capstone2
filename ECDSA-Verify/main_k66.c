/*
 * Kinetis K66 board support for the bootloader.
 *
 */

#define _CRT_SECURE_NO_WARNINGS

#include "kinetis.h"
#include "gpio/fsl_gpio.h"
#include "port/fsl_port.h"
#include "smc/smc.h"
#include "flash/fsl_flash.h"

#include "hw_config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "bl.h"
#include "ECDSA/ec-kcdsa.h"

#define BOOTLOADER_RESERVATION_SIZE	(24 * 1024)
#define FIRST_FLASH_SECTOR_TO_ERASE  (BOARD_FIRST_FLASH_SECTOR_TO_ERASE + (BOOTLOADER_RESERVATION_SIZE/FLASH_SECTOR_SIZE))

#define BOARD_RESETCLOCKRUN_CORE_CLOCK             20971520U  /*!< Core clock frequency: 20971520Hz */

#define MCG_IRCLK_DISABLE                                 0U  /*!< MCGIRCLK disabled */
#define MCG_PLL_DISABLE                                   0U  /*!< MCGPLLCLK disabled */
#define OSC_CAP0P                                         0U  /*!< Oscillator 0pF capacitor load */
#define OSC_ER_CLK_DISABLE                                0U  /*!< Disable external reference clock */
#define SIM_OSC32KSEL_OSC32KCLK_CLK                       0U  /*!< OSC32KSEL select: OSC32KCLK clock */
#define SIM_PLLFLLSEL_MCGFLLCLK_CLK                       0U  /*!< PLLFLL select: MCGFLLCLK clock */

// SIM_SDID
#define KINETIS_UNKNOWN	0
#define KINETIS_K66

#define PIN_MASK           0x0000000f
#define PIN_SHIFTS         0
#define FAM_MASK           0x00000070
#define FAM_SHIFTS         4
#define DIEID_MASK         0x00000f80
#define DIE_SHIFTS         7
#define REVID_MASK         0x0000f000
#define REVID_SHIFTS       12
#define RESID_MASK         0x000f0000
#define RESID_SHIFTS       16
#define SERIESID_MASK      0x00f00000
#define SERIESID_SHIFTS    20
#define SUBFAMID_MASK      0x0f000000
#define SUBFAMID_SHIFTS    24
#define FAMID_MASK         0xf0000000
#define FAMID_SHIFTS       28

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	(void *)BOARD_USART
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  	NULL
#endif

#define STK_CSR_CLKSOURCE_LSB   2
#define STK_CSR_CLKSOURCE_AHB_DIV8  (0 << STK_CSR_CLKSOURCE_LSB)
#define STK_CSR_CLKSOURCE_AHB   (1 << STK_CSR_CLKSOURCE_LSB)

flash_config_t s_flashDriver;                       //!< Flash driver instance.
static uint32_t s_flashRunCommand[kFLASH_ExecuteInRamFunctionMaxSizeInWords];
static uint32_t s_flashCacheClearCommand[kFLASH_ExecuteInRamFunctionMaxSizeInWords];
static flash_execute_in_ram_function_config_t s_flashExecuteInRamFunctionInfo = {
	.activeFunctionCount = 0,
	.flashRunCommand = s_flashRunCommand,
	.flashCacheClearCommand = s_flashCacheClearCommand,
};


/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= 120,
};

static void board_init(void);

#define BOOT_RTC_SIGNATURE          0xb007b007
#define POWER_DOWN_RTC_SIGNATURE    0xdeaddead // Written by app fw to not re-power on.
#define BOOT_RTC_REG                0

/* State of an inserted USB cable */
static bool usb_connected = false;

static uint32_t
board_get_rtc_signature()
{
	return RFVBAT->REG[BOOT_RTC_REG];
}

static void
board_set_rtc_signature(uint32_t sig)
{
	RFVBAT->REG[BOOT_RTC_REG] = sig;
}


static bool
board_test_force_pin()
{
	return false;
}
#if INTERFACE_USART == 1
static bool
board_test_usart_receiving_break()
{
#if !defined(SERIAL_BREAK_DETECT_DISABLED)
	/* (re)start the SysTick timer system */
	systick_interrupt_disable(); // Kill the interrupt if it is still active
	systick_counter_disable(); // Stop the timer
	systick_set_clocksource(SYSTIC_CLKSOURCE_AHB);

	/* Set the timer period to be half the bit rate
	 *
	 * Baud rate = 115200, therefore bit period = 8.68us
	 * Half the bit rate = 4.34us
	 * Set period to 4.34 microseconds (timer_period = timer_tick / timer_reset_frequency = 168MHz / (1/4.34us) = 729.12 ~= 729)
	 */
	systick_set_reload(729);  /* 4.3us tick, magic number */
	systick_counter_enable(); // Start the timer

	uint8_t cnt_consecutive_low = 0;
	uint8_t cnt = 0;

	/* Loop for 3 transmission byte cycles and count the low and high bits. Sampled at a rate to be able to count each bit twice.
	 *
	 * One transmission byte is 10 bits (8 bytes of data + 1 start bit + 1 stop bit)
	 * We sample at every half bit time, therefore 20 samples per transmission byte,
	 * therefore 60 samples for 3 transmission bytes
	 */
	while (cnt < 60) {
		// Only read pin when SysTick timer is true
		if (systick_get_countflag() == 1) {
			if (GPIO_ReadPinInput(BOARD_PORT_UART, BOARD_PIN_RX) == 0) {
				cnt_consecutive_low++;	// Increment the consecutive low counter

			} else {
				cnt_consecutive_low = 0; // Reset the consecutive low counter
			}

			cnt++;
		}

		// If 9 consecutive low bits were received break out of the loop
		if (cnt_consecutive_low >= 18) {
			break;
		}

	}

	systick_counter_disable(); // Stop the timer

	/*
	 * If a break is detected, return true, else false
	 *
	 * Break is detected if line was low for 9 consecutive bits.
	 */
	if (cnt_consecutive_low >= 18) {
		return true;
	}

#endif // !defined(SERIAL_BREAK_DETECT_DISABLED)

	return false;
}
#endif

uint32_t
board_get_devices(void)
{
	uint32_t devices = BOOT_DEVICES_SELECTION;

	if (usb_connected) {
		devices &= BOOT_DEVICES_FILTER_ONUSB;
	}

	return devices;
}

static void
board_init(void)
{
	exit_vlpr();

	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;

#if defined(BOARD_POWER_PIN_OUT)

	/* Configure the Power pins */

	/*  Sets the ports clocking */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_POWER_PORT));

	port_pin_config_t power_port_config = {
		.pullSelect           = kPORT_PullDown,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = kPORT_MuxAsGpio,
		.lockRegister         = kPORT_UnLockRegister,
	};

	/*  Sets the port configuration */

	PORT_SetPinConfig(KINETIS_PORT(BOARD_POWER_PORT), BOARD_POWER_PIN_OUT, &power_port_config);

	gpio_pin_config_t power_pin_config = {
		kGPIO_DigitalOutput,
		1,
	};

	/*  Sets the pin configuration */

	GPIO_PinInit(KINETIS_GPIO(BOARD_POWER_PORT), BOARD_POWER_PIN_OUT, &power_pin_config);

	BOARD_POWER_ON(KINETIS_GPIO(BOARD_POWER_PORT), BOARD_POWER_PIN_OUT);
#endif

#if INTERFACE_USB

	// Disable the MPU otherwise USB cannot access the bus

	MPU->CESR = 0;

	/* enable Port pin to sample VBUS */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_PORT_VBUS));

	port_pin_config_t vbus_port_config = {
		.pullSelect           = kPORT_PullDown,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = kPORT_MuxAsGpio,
		.lockRegister         = kPORT_UnLockRegister,
	};

	/*  Sets the port configuration */

	PORT_SetPinConfig(KINETIS_PORT(BOARD_PORT_VBUS), BOARD_PIN_VBUS, &vbus_port_config);

	gpio_pin_config_t vbus_pin_config = {
		kGPIO_DigitalInput,
		0,
	};

	/*  Sets the pin configuration */

	GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_VBUS), BOARD_PIN_VBUS, &vbus_pin_config);

#endif

	/* Regardless of UART booting ensure CTS on Radio is set High to not enter
	 * bootloader on sik
	 */

#if INTERFACE_USB == 0
	/* enable Clock to Port E pin if USB was not selected to set RTS */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_PORT_UART_RTS));
#endif

	port_pin_config_t uart_rts_port_config = {
		.pullSelect           = kPORT_PullDisable,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = kPORT_MuxAsGpio,
		.lockRegister         = kPORT_UnLockRegister,
	};

	/*  Sets the port configuration */

	PORT_SetPinConfig(KINETIS_PORT(BOARD_PORT_UART_RTS), BOARD_UART_RTS_PIN, &uart_rts_port_config);

	gpio_pin_config_t rts_pin_config = {
		kGPIO_DigitalOutput,
		1,
	};
	GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_UART_RTS), BOARD_UART_RTS_PIN, &rts_pin_config);

#if INTERFACE_USART

	/* configure USART clock */

	CLOCK_EnableClock(KINETIS_CLOCK_UART(BOARD_USART));

	/* configure USART pins */

	CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_PORT_UART));

	port_pin_config_t uart_port_config = {
		.pullSelect           = kPORT_PullDisable,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = BOARD_PORT_UART_AF,
		.lockRegister         = kPORT_UnLockRegister,
	};

	/*  Sets the port configuration */

	PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_UART), KINETIS_MASK(BOARD_PIN_TX) | KINETIS_MASK(BOARD_PIN_RX),
				   &uart_port_config);

#endif

	/* Initialize LEDs */

	port_pin_config_t led_port_config = {
		.pullSelect           = kPORT_PullDisable,
		.slewRate             = kPORT_FastSlewRate,
		.passiveFilterEnable  = kPORT_PassiveFilterDisable,
		.openDrainEnable      = kPORT_OpenDrainDisable,
		.driveStrength        = kPORT_LowDriveStrength,
		.mux                  = kPORT_MuxAsGpio,
		.lockRegister         = kPORT_UnLockRegister,
	};

	uint32_t leds = 0;
#if defined(BOARD_PIN_LED_ACTIVITY)
	leds |= KINETIS_MASK(BOARD_PIN_LED_ACTIVITY);
#endif

#if defined(BOARD_PIN_LED_BOOTLOADER)
	leds |= KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER);
#endif

	if (leds) {
		CLOCK_EnableClock(KINETIS_CLOCK_PORT(BOARD_PORT_LEDS));

		PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_LEDS), leds, &led_port_config);

		gpio_pin_config_t led_pin_config = {
			kGPIO_DigitalOutput,
			1,
		};

		/*  Sets the pin configuration */

#if defined(BOARD_PIN_LED_ACTIVITY)
		GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_LEDS), BOARD_PIN_LED_ACTIVITY, &led_pin_config);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
		GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_LEDS), BOARD_PIN_LED_BOOTLOADER, &led_pin_config);
#endif
		BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), leds);
	}
}

void
board_deinit(void)
{

	port_pin_config_t unconfigure_port_config = {
		kPORT_PullDisable,
		kPORT_FastSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_OpenDrainDisable,
		kPORT_LowDriveStrength,
		kPORT_PinDisabledOrAnalog,
		kPORT_UnLockRegister,
	};
	gpio_pin_config_t unconfigure_pin_config = {
		kGPIO_DigitalInput,
		0,
	};

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	GPIO_PinInit(KINETIS_GPIO(BOARD_POWER_PORT), BOARD_POWER_PIN_OUT, &unconfigure_pin_config);
	PORT_SetPinConfig(KINETIS_PORT(BOARD_POWER_PORT), BOARD_POWER_PIN, &unconfigure_port_config);

#endif

#if INTERFACE_USB
	PORT_SetPinConfig(KINETIS_PORT(BOARD_PORT_VBUS), BOARD_PIN_VBUS, &unconfigure_port_config);
#endif

#if INTERFACE_USART
	PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_UART), KINETIS_MASK(BOARD_PIN_TX) | KINETIS_MASK(BOARD_PIN_RX),
				   &unconfigure_port_config);
#endif

	/* deinitialise RTS */
	GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_UART_RTS), BOARD_UART_RTS_PIN, &unconfigure_pin_config);
	PORT_SetPinConfig(KINETIS_PORT(BOARD_PORT_UART_RTS), BOARD_UART_RTS_PIN, &unconfigure_port_config);

	/* deinitialise LEDs */
	uint32_t leds = 0;
#if defined(BOARD_PIN_LED_ACTIVITY)
	leds |= KINETIS_MASK(BOARD_PIN_LED_ACTIVITY);
#endif

#if defined(BOARD_PIN_LED_BOOTLOADER)
	leds |= KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER);
#endif

	if (leds) {
		GPIO_ClearPinsOutput(KINETIS_GPIO(BOARD_PORT_LEDS), leds);

#if defined(BOARD_PIN_LED_ACTIVITY)
		GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_LEDS), BOARD_PIN_LED_ACTIVITY, &unconfigure_pin_config);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
		GPIO_PinInit(KINETIS_GPIO(BOARD_PORT_LEDS), BOARD_PIN_LED_BOOTLOADER, &unconfigure_pin_config);
#endif

		PORT_SetMultiplePinsConfig(KINETIS_PORT(BOARD_PORT_LEDS), leds, &unconfigure_port_config);
	}

	/*  Incase of any over lap un-configure clocks last -*/

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	CLOCK_DisableClock(KINETIS_CLOCK_PORT(BOARD_POWER_PORT));
#endif

#if INTERFACE_USB
	CLOCK_DisableClock(KINETIS_CLOCK_PORT(BOARD_PORT_VBUS));
#endif

#if INTERFACE_USB == 0
	CLOCK_DisableClock(KINETIS_CLOCK_PORT(BOARD_PORT_UART_RTS));
#endif

#if INTERFACE_USART
	CLOCK_DisableClock(KINETIS_CLOCK_PORT(BOARD_PORT_UART));
	CLOCK_DisableClock(KINETIS_CLOCK_UART(BOARD_UART));
#endif

	if (leds) {
		CLOCK_DisableClock(KINETIS_CLOCK_PORT(BOARD_PORT_LEDS));
	}
}


static void CLOCK_CONFIG_FllStableDelay(void)
{
	uint32_t i = 30000U;

	while (i--) {
		__NOP();
	}
}

inline void arch_systic_init(void)
{
	/* (re)start the timer system */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(board_info.systick_mhz * 1000);  /* 1ms tick, magic number */
	systick_interrupt_enable();
	systick_counter_enable();
}

inline void arch_systic_deinit(void)
{
	/* kill the systick interrupt */
	systick_interrupt_disable();
	systick_counter_disable();
}

void
clock_deinit(void)
{

	const mcg_config_t mcgConfig_BOARD_BootClockHSRUN = {
		.mcgMode = kMCG_ModePBE,                  /* PEE - PLL Engaged External */
		.irclkEnableMode = kMCG_IrclkEnable,      /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
		.ircs = kMCG_IrcSlow,                     /* Slow internal reference clock selected */
		.fcrdiv = 0x1U,                           /* Fast IRC divider: divided by 2 */
		.frdiv = 0x0U,                            /* FLL reference clock divider: divided by 32 */
		.drs = kMCG_DrsLow,                       /* Low frequency range */
		.dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
		.oscsel = kMCG_OscselOsc,                 /* Selects System Oscillator (OSCCLK) */
		.pll0Config =
		{
			.enableMode = MCG_PLL_DISABLE,    /* MCGPLLCLK disabled */
			.prdiv = 0x0U,                    /* PLL Reference divider: divided by 1 */
			.vdiv = 0xeU,                     /* VCO divider: multiplied by 30 */
		},
		.pllcs = kMCG_PllClkSelPll0,              /* PLL0 output clock is selected */
	};
	const osc_config_t oscConfig_BOARD_BootClockHSRUN = {
		.freq = 12000000U,                        /* Oscillator frequency: 12000000Hz */
		.capLoad = (OSC_CAP0P),                   /* Oscillator capacity load: 0pF */
		.workMode = kOSC_ModeOscLowPower,         /* Oscillator low power */
		.oscerConfig =
		{
			.enableMode = kOSC_ErClkEnable,   /* Enable external reference clock, disable external reference clock in STOP mode */
			.erclkDiv = 0,                    /* Divider for OSCERCLK: divided by 1 */
		}
	};

	/*******************************************************************************
	 * Code for BOARD_BootClockHSRUN configuration
	 ******************************************************************************/

	/* Set the system clock dividers in SIM to safe value. */
	CLOCK_SetSimSafeDivs();
	/* Initializes OSC0 according to board configuration. */
	CLOCK_InitOsc0(&oscConfig_BOARD_BootClockHSRUN);
	CLOCK_SetXtal0Freq(oscConfig_BOARD_BootClockHSRUN.freq);
	/* Configure the Internal Reference clock (MCGIRCLK). */
	CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockHSRUN.irclkEnableMode,
				      mcgConfig_BOARD_BootClockHSRUN.ircs,
				      mcgConfig_BOARD_BootClockHSRUN.fcrdiv);
	/* Set MCG to PEE mode. */
	CLOCK_BootToBlpeMode(mcgConfig_BOARD_BootClockHSRUN.oscsel);

	const mcg_config_t mcgConfig_ClocksFunc_1 = {
		.mcgMode = kMCG_ModeFEI,                  /* FEI - FLL Engaged Internal */
		.irclkEnableMode = MCG_IRCLK_DISABLE,     /* MCGIRCLK disabled */
		.ircs = kMCG_IrcSlow,                     /* Slow internal reference clock selected */
		.fcrdiv = 0x1U,                           /* Fast IRC divider: divided by 2 */
		.frdiv = 0x0U,                            /* FLL reference clock divider: divided by 1 */
		.drs = kMCG_DrsLow,                       /* Low frequency range */
		.dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
		.oscsel = kMCG_OscselOsc,                 /* Selects System Oscillator (OSCCLK) */
		.pll0Config =
		{
			.enableMode = MCG_PLL_DISABLE,    /* MCGPLLCLK disabled */
			.prdiv = 0x0U,                    /* PLL Reference divider: divided by 1 */
			.vdiv = 0x0U,                     /* VCO divider: multiplied by 16 */
		},
		.pllcs = kMCG_PllClkSelPll0,              /* PLL0 output clock is selected */
	};
	const sim_clock_config_t simConfig_ClocksFunc_1 = {
		.pllFllSel = SIM_PLLFLLSEL_MCGFLLCLK_CLK, /* PLLFLL select: MCGFLLCLK clock */
		.pllFllDiv = 0,                           /* PLLFLLSEL clock divider divisor: divided by 1 */
		.pllFllFrac = 0,                          /* PLLFLLSEL clock divider fraction: multiplied by 1 */
		.er32kSrc = SIM_OSC32KSEL_OSC32KCLK_CLK,  /* OSC32KSEL select: OSC32KCLK clock */
		.clkdiv1 = 0x110000U,                     /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /1, OUTDIV3: /2, OUTDIV4: /2 */
	};

	/* Set MCG to FEI mode. */
#if FSL_CLOCK_DRIVER_VERSION >= MAKE_VERSION(2, 2, 0)
	CLOCK_SetFbeMode(0, mcgConfig_ClocksFunc_1.dmx32,
			 mcgConfig_ClocksFunc_1.drs,
			 CLOCK_CONFIG_FllStableDelay);

	CLOCK_BootToFeiMode(mcgConfig_ClocksFunc_1.dmx32,
			    mcgConfig_ClocksFunc_1.drs,
			    CLOCK_CONFIG_FllStableDelay);
#else
	CLOCK_BootToFeiMode(mcgConfig_ClocksFunc_1.drs,
			    CLOCK_CONFIG_FllStableDelay);
#endif
	/* Set the clock configuration in SIM module. */
	CLOCK_SetSimConfig(&simConfig_ClocksFunc_1);
	/* Set SystemCoreClock variable. */
	SystemCoreClock = BOARD_RESETCLOCKRUN_CORE_CLOCK;
}

inline void arch_flash_lock(void)
{
}

inline void arch_flash_unlock(void)
{
}

inline void arch_setvtor(uint32_t address)
{
	SCB->VTOR = address;
}

uint32_t flash_func_sector_size(unsigned sector)
{
	return (sector < BOARD_FLASH_SECTORS) ? FLASH_SECTOR_SIZE : 0;
}

void flash_erase_sector(unsigned sector)
{
	__disable_irq();
	FLASH_Erase(&s_flashDriver, (sector * FLASH_SECTOR_SIZE), FLASH_SECTOR_SIZE, kFLASH_ApiEraseKey);
	__enable_irq();

}

static bool flash_verify_erase(unsigned sector)
{
	/* Calculate the physical address of the sector
	 */
	volatile uint32_t address = APP_LOAD_ADDRESS + (sector - FIRST_FLASH_SECTOR_TO_ERASE) * FLASH_SECTOR_SIZE;
	__disable_irq();
	volatile status_t status = FLASH_VerifyErase(&s_flashDriver, address, FLASH_SECTOR_SIZE, kFLASH_MarginValueNormal);
	__enable_irq();
	return status == kStatus_FLASH_Success;
}

void flash_func_erase_sector(unsigned sector)
{
	if (sector >= BOARD_FLASH_SECTORS || sector < FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	if (!flash_verify_erase(sector)) {

		/* erase the sector if it failed the blank check */

		flash_erase_sector(sector);
	}
}

static uint32_t words[2] = {0, 0};
static uint32_t second_word = 0xffffffff;
static uint32_t pending = 0;

void
flash_func_write_word(uint32_t address, uint32_t word)
{
	address += APP_LOAD_ADDRESS;

	uint32_t loc = (address & 4) >> 2;
	pending = loc == 0;

	// Cache words
	words[loc] = word;

	/* Program the 64bits. */

	// Time to write first and second word

	if (address == APP_LOAD_ADDRESS && second_word != 0xffffffff) {
		words[1] = second_word;
		loc = 1;
		pending = 0;
		address +=  sizeof(words[1]);
	}

	if (loc == 1) {
		if (address == APP_LOAD_ADDRESS + sizeof(word) && second_word == 0xffffffff) {
			second_word = words[1];
			return;
		}

		__disable_irq();
		FLASH_Program(&s_flashDriver, address - sizeof(words[1]), &words[0], sizeof(words));
		__enable_irq();

	}

}

uint32_t flash_func_read_word(uint32_t address)
{
	if (address & 3) {
		return 0;
	}

	if (address == sizeof(uint32_t) && second_word != 0xffffffff) { return second_word; }

	return (pending) ? words[0] : *(uint32_t *)(address + APP_LOAD_ADDRESS);
}

uint32_t
flash_func_read_otp(uint32_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	return SIM->SDID;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	const char dig[] = "0123456789ABCDEF";
	const char none[] = "MK66FN2M0VMD18,0";
	int i;

	for (i = 0; none[i] && i < max - 1; i++) {
		revstr[i] = none[i];
	}

	uint32_t id = (get_mcu_id() & REVID_MASK) >> REVID_SHIFTS;
	revstr[i - 1] = dig[id];
	return i;
}


int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uint32_t address)
{
	address /= sizeof(address);
	const volatile uint32_t *p = &SIM->UIDH;
	return p[address];
}

void
led_on(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		BOARD_LED_ON(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

void
led_off(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		BOARD_LED_OFF(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		BOARD_LED_OFF(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

void
led_toggle(unsigned led)
{
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		GPIO_TogglePinsOutput(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_ACTIVITY));
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		GPIO_TogglePinsOutput(KINETIS_GPIO(BOARD_PORT_LEDS), KINETIS_MASK(BOARD_PIN_LED_BOOTLOADER));
#endif
		break;
	}
}

int
main(void)
{
	// The Kinetis start up code has initialized Clocks and MPU, FPU

	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

#if defined(BOARD_POWER_PIN_OUT)

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif

	// Initialize the RAM based function in the Kinetis Flash Lib (this could be removed with run_from_ram tags
	// and some linker magic.

	s_flashDriver.flashExecuteInRamFunctionInfo = &s_flashExecuteInRamFunctionInfo.activeFunctionCount;
	FLASH_PrepareExecuteInRamFunctions(&s_flashDriver);
	FLASH_Init(&s_flashDriver);

	/* do board-specific initialization */
	board_init();

	/*
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting.
	 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {

		/*
		 * Don't even try to boot before dropping to the bootloader.
		 */
		try_boot = false;

		/*
		 * Don't drop out of the bootloader until something has been uploaded.
		 */
		timeout = 0;

		/*
		 * Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.
		 */
		board_set_rtc_signature(0);
	}

	/*
	 * Check if the force-bootloader pins are strapped; if strapped,
	 * don't try booting.
	 */
	if (board_test_force_pin()) {
		try_boot = false;
	}

#if INTERFACE_USB

	/*
	 * Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
#if defined(BOARD_USB_VBUS_SENSE_DISABLED)
	try_boot = false;
#else

	if (GPIO_ReadPinInput(KINETIS_GPIO(BOARD_PORT_VBUS), BOARD_PIN_VBUS) != 0) {
		usb_connected = true;
		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}

#endif
#endif

#if INTERFACE_USART

	/*
	 * Check for if the USART port RX line is receiving a break command, or is being held low. If yes,
	 * don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
	if (board_test_usart_receiving_break()) {
		try_boot = false;
	}

#endif

	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* try to boot immediately */
		jump_to_app();

		// If it failed to boot, reset the boot signature and stay in bootloader
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		timeout = 0;
	}

	/* start the interface */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif

	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		if (board_test_force_pin()) {
			continue;
		}

#if INTERFACE_USART

		/* if the USART port RX line is still receiving a break, just loop back */
		if (board_test_usart_receiving_break()) {
			continue;
		}

#endif

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;

		


		
	}
}

void _start()
{
	main();
	// public key 생성시 필요한 객체
	MPZ example;
	ECC_PARAMS curve;
	ECC_POINT public_point2;

	// 00000000: ee9e ff1f 1164 0000 ad7d 0000 ad7d 0000
	// 001c5030: 70bd 019a 0132 f5e7 00f0 0140 4480 0440

	// Px4 image 크기 : 1c5030 = 1855536

	//unsigned char msg[1855536];
	//int msglen = 1855536;
	




	
	// example demo 
	/*
	unsigned char msg1[] =
	{
		0xee, 0x9e, 0xff, 0x1f, 0x11, 0x64, 0x00, 0x00, 0xad, 0x7d, 0x00, 0x00, 0xad, 0x7d, 0x00,0x00,
		0x70, 0xbd, 0x01, 0x9a, 0x01, 0x32, 0xf5, 0xe7, 0x00, 0xf0, 0x01, 0x40, 0x44, 0x80, 0x04,0x40

	};
	int msg1Len = 32;
	*/

	// 00000000: ee9e ff1f 1164 0000 ad7d 0000 ad7d 0000
	// 001c5030: 70bd 019a 0132 f5e7 00f0 0140 4480 0440

	// Px4 image 크기 : 1c5030 = 1855536

	//unsigned char msg[1855536];
	//int msglen = 1855536;

	unsigned char msg[15000];
	int msgLen = 5000;
	
	//unsigned char* px4addr1 = 0x6000;
	for (int i = 0; i < msgLen; i++) {
		msg[i] = *(unsigned char*)(0x6000+ i);
	}

	//unsigned char* px4addr2 = 0xdbba0;
	for (int i = msgLen; i < msgLen + msgLen; i++) {
		msg[i] = *(unsigned char*)(0xdbba0 + i);
	}

	//unsigned char* px4addr3 = 0x1b7740;
	for (int i = msgLen + msgLen; i < msgLen + msgLen + msgLen; i++) {
		msg[i] = *(unsigned char*)(0x1b7740 + i);
	}


	// 키생성을 위한 사용자 입력
	unsigned char user_provided_random_input[] =
	{
		0x73, 0x61, 0x6C, 0x64, 0x6A, 0x66, 0x61, 0x77, 0x70, 0x33, 0x39, 0x39, 0x75, 0x33, 0x37, 0x34,
		0x72, 0x30, 0x39, 0x38, 0x75, 0x39, 0x38, 0x5E, 0x25, 0x5E, 0x25, 0x68, 0x6B, 0x72, 0x67, 0x6E,
		0x3B, 0x6C, 0x77, 0x6B, 0x72, 0x70, 0x34, 0x37, 0x74, 0x39, 0x33, 0x63, 0x25, 0x24, 0x38, 0x39,
		0x34, 0x33, 0x39, 0x38, 0x35, 0x39, 0x6B, 0x6A, 0x64, 0x6D, 0x6E, 0x76, 0x63, 0x6D, 0x20, 0x63,
		0x76, 0x6B, 0x20, 0x6F, 0x34, 0x75, 0x30, 0x39, 0x72, 0x20, 0x34, 0x6A, 0x20, 0x6F, 0x6A, 0x32,
		0x6F, 0x75, 0x74, 0x32, 0x30, 0x39, 0x78, 0x66, 0x71, 0x77, 0x3B, 0x6C, 0x2A, 0x26, 0x21, 0x5E,
		0x23, 0x40, 0x55, 0x23, 0x2A, 0x23, 0x24, 0x29, 0x28, 0x23, 0x20, 0x7A, 0x20, 0x78, 0x6F, 0x39,
		0x35, 0x37, 0x74, 0x63, 0x2D, 0x39, 0x35, 0x20, 0x35, 0x20, 0x76, 0x35, 0x6F, 0x69, 0x75, 0x76,
		0x39, 0x38, 0x37, 0x36, 0x20, 0x36, 0x20, 0x76, 0x6A, 0x20, 0x6F, 0x35, 0x69, 0x75, 0x76, 0x2D,
		0x30, 0x35, 0x33, 0x2C, 0x6D, 0x63, 0x76, 0x6C, 0x72, 0x6B, 0x66, 0x77, 0x6F, 0x72, 0x65, 0x74
	};
	int user_provided_random_input_Len = 160;
	//unsigned char sig[72];

	
	ECC_init_params(ECC_PRIME_FIELD, &curve, SECP256r);
	
	ECC_set_params(SECP256r, &curve);

	Private_Key_generator(&example, curve, SECP256r, SHA256, user_provided_random_input, user_provided_random_input_Len);


	//printf("%02X ", sk.len);
	//printf("%02X ", sk.sig);

	example.len = 8;
	example.sig = 1;

	example.dat[0] = 1096416938;
	example.dat[1] = 1447096863;
	example.dat[2] = -1856771272;
	example.dat[3] = 1231516280;
	example.dat[4] = -303423242;
	example.dat[5] = -1535317680;
	example.dat[6] = 223322781;
	example.dat[7] = 1308051087;
	
	//1096416938 1447096863 - 1856771272 1231516280 - 303423242 - 1535317680 223322781 1308051087



	Public_Key_generator(&public_point2, example.dat, curve);

	/*

	AA D8 CF 9C 3C 81 A0 51 86 F5 9E 7E 09 EF B1 0D 8E 59 2D B0 07 9B E3 29 E7 26 82 9B
	5A 9B 10 56 AB 68 BF 38 CE 7A 6F B6 C5 D0 21 C2 6A D0 22 CB AB 7B 99 57 43 46 0D FF
	20 26 08 AE 07 3B 99 17 CC CC CC CC CC CC CC CC
	*/

	unsigned char digitalsig[] = {
	0xAA, 0xD8, 0xCF, 0x9C, 0x3C, 0x81, 0xA0, 0x51, 0x86, 0xF5, 0x9E, 0x7E, 0x09, 0xEF, 0xB1, 0x0D, 0x8E, 0x59, 0x2D, 0xB0, 0x07, 0x9B, 0xE3, 0x29, 0xE7, 0x26, 0x82, 0x9B,
	0x5A, 0x9B, 0x10, 0x56, 0xAB, 0x68, 0xBF, 0x38, 0xCE, 0x7A, 0x6F, 0xB6, 0xC5, 0xD0, 0x21, 0xC2, 0x6A, 0xD0, 0x22, 0xCB, 0xAB, 0x7B, 0x99, 0x57, 0x43, 0x46, 0x0D, 0xFF,
	0x20, 0x26, 0x08, 0xAE, 0x07, 0x3B, 0x99, 0x17, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC
	};




	/*
	printf("R = ");
	for (int i = 0; i < 28; i++) {
		printf("%02X ", sig[i]);
	}
	printf("\n");
	printf("S = ");
	for (int i = 28; i < 56; i++) {
		printf("%02X ", sig[i]);
	}
	printf("\n");
	for (int i = 0; i < 72; i++) {
		printf("%02X ", sig[i]);
	}
	printf("\n");
	*/
	
	

	int ret = EC_KCDSA_verify(public_point2, digitalsig, curve, SECP256r, SHA256, msg, msgLen);
	// Secure boot 정상시
	if (ret == 0) {
		*(unsigned char*)0x1ddddd = 0;
	}
	// Secure boot 비정상시
	else {
		*(unsigned char*)0x1ddddd = 1;
	}



}


void SysTick_Handler()
{
	sys_tick_handler();
}

int DbgConsole_Printf(char *fmt_s, ...)
{
	return 0;
}
