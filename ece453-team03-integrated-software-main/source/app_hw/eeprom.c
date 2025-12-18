/**
 * @file eeprom.c
 * @author Joe Krachey (jkrachey@wisc.edu)
 * @brief 
 * @version 0.1
 * @date 2023-10-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "eeprom.h"
#include "cyhal_gpio.h"
#include "cyhal_hw_types.h"
#include <stdbool.h>

static cyhal_spi_t *eeprom_spi_obj;
static cyhal_gpio_t eeprom_cs_pin;

/**
 * @brief 
 *  Initializes the SPI handle and CS pin for the EEPROM
 * @param spi_obj 
 * @param cs_pin 
 */
void eeprom_init(cyhal_spi_t *spi_obj, cyhal_gpio_t cs_pin)
{
	if(spi_obj == NULL || cs_pin == NC)
	{
		CY_ASSERT(0);
	}

	eeprom_spi_obj = spi_obj;
	eeprom_cs_pin = cs_pin;
	cyhal_gpio_init(eeprom_cs_pin, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
}

/** Determine if the EEPROM is busy writing the last
 *  transaction to non-volatile storage
 *
 * @param
 *
 */
void eeprom_wait_for_write(void)
{
	uint8_t transmit_data[2] = {EEPROM_CMD_RDSR, 0xFF};
	uint8_t receive_data[2] = {0x00, 0x00};
	cy_rslt_t rslt;

	// Check to see if the eeprom is still updating
	// the data from the last write
	do
	{
		// Set the CS Low
		cyhal_gpio_write(eeprom_cs_pin, 0);

		// Starts a data transfer
		rslt = cyhal_spi_transfer(
			eeprom_spi_obj,
			transmit_data,
			2u,
			receive_data,
			2u,
			0xFF);

		CY_ASSERT(rslt == CY_RSLT_SUCCESS); /* Halt MCU if SPI transaction fails*/

		// Set the CS High
		cyhal_gpio_write(eeprom_cs_pin, 1);

		// If the address was not ACKed, try again.
	} while ((receive_data[1] & 0x01) != 0);
}

/** Enables Writes to the EEPROM
 *
 * @param
 *
 */
void eeprom_write_enable(void)
{
	uint8_t transmit_data[1] = {EEPROM_CMD_WREN};
	uint8_t receive_data[1] = {0x00};
	cy_rslt_t rslt;

	// Set the CS Low
	cyhal_gpio_write(eeprom_cs_pin, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
		eeprom_spi_obj,
		transmit_data,
		1u,
		receive_data,
		1u,
		0xFF);

	CY_ASSERT(rslt == CY_RSLT_SUCCESS); /* Halt MCU if SPI transaction fails*/

	// Set the CS High
	cyhal_gpio_write(eeprom_cs_pin, 1);
}

/** Disable Writes to the EEPROM
 *
 * @param
 *
 */
void eeprom_write_disable(void)
{
	uint8_t transmit_data[1] = {EEPROM_CMD_WRDI};
	uint8_t receive_data[1] = {0x00};
	cy_rslt_t rslt;

	// Set the CS Low
	cyhal_gpio_write(eeprom_cs_pin, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
		eeprom_spi_obj,
		transmit_data,
		1u,
		receive_data,
		1u,
		0xFF);

	CY_ASSERT(rslt == CY_RSLT_SUCCESS); /* Halt MCU if SPI transaction fails*/

	// Set the CS High
	cyhal_gpio_write(eeprom_cs_pin, 1);
}

/** Writes a single byte to the specified address
 *
 * @param address -- 16 bit address in the EEPROM
 * @param data    -- value to write into memory
 *
 */
void eeprom_write_byte(uint16_t address, uint8_t data)
{
	uint8_t transmit_data[4];
	uint8_t receive_data[4];
	cy_rslt_t rslt;

	// Wait for any outstanding writes to complete
	eeprom_wait_for_write();

	// Enable writes to the eeprom
	eeprom_write_enable();

	transmit_data[0] = EEPROM_CMD_WRITE;
	transmit_data[1] = (uint8_t)(address >> 8);
	transmit_data[2] = (uint8_t)address;
	transmit_data[3] = data;

	// Set the CS Low
	cyhal_gpio_write(eeprom_cs_pin, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
		eeprom_spi_obj,
		transmit_data,
		4u,
		receive_data,
		4u,
		0xFF);

	CY_ASSERT(rslt == CY_RSLT_SUCCESS); /* Halt MCU if SPI transaction fails*/

	// Set the CS High
	cyhal_gpio_write(eeprom_cs_pin, 1);

	// Disable writes to the eeprom
	eeprom_write_disable();
}

/** Reads a single byte to the specified address
 *
 * @param address -- 16 bit address in the EEPROM
 *
 */
uint8_t eeprom_read_byte(uint16_t address)
{
	uint8_t transmit_data[4];
	uint8_t receive_data[4];
	cy_rslt_t rslt;

	// Wait for any outstanding writes to complete
	eeprom_wait_for_write();

	// Enable writes to the eeprom
	eeprom_write_enable();

	transmit_data[0] = EEPROM_CMD_READ;
	transmit_data[1] = (uint8_t)(address >> 8);
	transmit_data[2] = (uint8_t)address;
	transmit_data[3] = 0x00;

	// Set the CS Low
	cyhal_gpio_write(eeprom_cs_pin, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
		eeprom_spi_obj,
		transmit_data,
		4u,
		receive_data,
		4u,
		0xFF);

	CY_ASSERT(rslt == CY_RSLT_SUCCESS); /* Halt MCU if SPI transaction fails*/

	// Set the CS High
	cyhal_gpio_write(eeprom_cs_pin, 1);

	// Disable writes to the eeprom
	eeprom_write_disable();

	// Return the value from the EEPROM to the user
	return receive_data[3];
}