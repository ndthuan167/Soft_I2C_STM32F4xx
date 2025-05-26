/**
 * @file Soft_I2C.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Configuration for Soft SPI in STM32F407VGTx (ARMCortex M4) as Master
 * @date 2025-05-26
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include "Soft_I2C.h"
#include "../GPIO/GPIO.h"
#include "../RCC/RCC.h"

/******************************************************************************
 * Pointer definition to address of GPIO and USART definition
 ******************************************************************************/
GPIOn *gpioB_S_i2c = (GPIOn *)ADDRESS_GPIO_B; // GPIO port B for I2C1/2
GPIOn *gpioH_S_i2c = (GPIOn *)ADDRESS_GPIO_H; // GPIO port H for I2C3

/******************************************************************************
 * Variables definition
 ******************************************************************************/
uint8_t index_i2c;
uint8_t index_arr;
uint8_t index_delay;

bool slave_address_transmited = false;
bool word_address_lower_transmited = false;
bool word_address_upper_transmited = false;

/******************************************************************************
 * Private functions definition
 ******************************************************************************/
void SoftI2C_GPIOConfiguration(uint8_t I2Cx, uint8_t modeRW);
void SoftI2C_Init(uint8_t I2Cx);
void SoftI2C_Start(uint8_t I2Cx);
void SoftI2C_Stop(uint8_t I2Cx);

bool SoftI2C_WriteByte(uint8_t I2Cx, uint8_t data);
void SoftI2C_SendWordAddress(uint8_t I2Cx, uint16_t address);
void SoftI2C_SendSlaveAddress(uint8_t I2Cx, uint8_t address, uint8_t modeRW);

bool SoftI2C_GetSDA(uint8_t I2Cx);
void SoftI2C_SendACK(uint8_t I2Cx);
uint8_t SoftI2C_ReadByte(uint8_t I2Cx);

void SetPortSDA(uint8_t I2Cx, uint8_t bit);
void SetPortSCL(uint8_t I2Cx, uint8_t bit);
void DelayI2CTime(uint16_t time_delay);

/**
*******************************************************************************
* @ Name : SoftI2C_GPIOConfiguration
* @ Parameters: uint8_t I2Cx, uint8_t modeRW
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Enable GPIO Clock and configure GPIO mode of Soft I2C for corresponding I2Cx and I2C mode (Read/Write)
*       + I2C_1: PB6 (SCL), PB7 (SDA)
*       + I2C_2: PB10 (SCL), PB11 (SDA)
*       + I2C_3: PH7 (SCL), PH8 (SDA)
*       + Write mode: MODER_OUTPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING for all 2 pins
*       + Read mode: MODER_INPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING for SDA pin only
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_GPIOConfiguration(uint8_t I2Cx, uint8_t modeRW)
{
    switch (I2Cx)
    {
        case I2C_1:
            // Enable Clock for GPIO_B port for I2C1
            RCC_EnablePeripheralClock(CLOCK_GPIO_B);

            // Set Output for SCL pin
            GPIO_Configuration(gpioB_S_i2c, GPIO_PIN6, MODER_OUTPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            
            // Set Output or Input for SDA pin depending on modeRW (Read/Write)
            if (modeRW == I2C_MODE_WRITE)
                GPIO_Configuration(gpioB_S_i2c, GPIO_PIN7, MODER_OUTPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            else if (modeRW == I2C_MODE_READ)
                GPIO_Configuration(gpioB_S_i2c, GPIO_PIN7, MODER_INPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            break;
        case I2C_2:
            // Enable Clock for GPIO_B port for I2C2
            RCC_EnablePeripheralClock(CLOCK_GPIO_B);

            // Set Output for SCL pin
            GPIO_Configuration(gpioB_S_i2c, GPIO_PIN10, MODER_OUTPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            
            // Set Output or Input for SDA pin depending on modeRW (Read/Write)
            if (modeRW == I2C_MODE_WRITE)
                GPIO_Configuration(gpioB_S_i2c, GPIO_PIN11, MODER_OUTPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            else if (modeRW == I2C_MODE_READ)
                GPIO_Configuration(gpioB_S_i2c, GPIO_PIN11, MODER_INPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            break;
        case I2C_3:
            // Enable Clock for GPIO_H port for I2C3
            RCC_EnablePeripheralClock(CLOCK_GPIO_H);

            // Set Output for SCL pin
            GPIO_Configuration(gpioH_S_i2c, GPIO_PIN7, MODER_OUTPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            
            // Set Output or Input for SDA pin depending on modeRW (Read/Write)
            if (modeRW == I2C_MODE_WRITE)
                GPIO_Configuration(gpioH_S_i2c, GPIO_PIN8, MODER_OUTPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            else if (modeRW == I2C_MODE_READ)
                GPIO_Configuration(gpioH_S_i2c, GPIO_PIN8, MODER_INPUT, OTYPER_OPENDRAIN, OSPEEDR_HIGH, PUPDR_NOTHING);
            break;
        default:
            break;
    }
}


/**
*******************************************************************************
* @ Name : SoftI2C_Init
* @ Parameters: uint8_t I2Cx
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Initialize Soft I2C by set SDA and SCL pin to HIGH before Start I2C and initialize variables
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_Init(uint8_t I2Cx)
{
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);
    slave_address_transmited = false;
    word_address_lower_transmited = false;
    word_address_upper_transmited = false;
    SetPortSDA(I2Cx, SET);
    SetPortSCL(I2Cx, SET);
}


/**
*******************************************************************************
* @ Name : SetPortSDA
* @ Parameters: uint8_t I2Cx, uint8_t bit
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Set SDA pin to HIGH or LOW based on bit parameter (SET/CLEAR) for corresponding I2Cx
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SetPortSDA(uint8_t I2Cx, uint8_t bit)
{
    if (I2Cx == I2C_1)
        GPIO_SettingOutputDataBSRR(gpioB_S_i2c, GPIO_PIN7, bit);
    else if (I2Cx == I2C_2)
        GPIO_SettingOutputDataBSRR(gpioB_S_i2c, GPIO_PIN11, bit);
    else if (I2Cx == I2C_3)
        GPIO_SettingOutputDataBSRR(gpioH_S_i2c, GPIO_PIN8, bit);
}


/**
*******************************************************************************
* @ Name : SetPortSCL
* @ Parameters: uint8_t I2Cx, uint8_t bit
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Set SCL pin to HIGH or LOW based on bit parameter (SET/CLEAR) for corresponding I2Cx
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SetPortSCL(uint8_t I2Cx, uint8_t bit)
{
    if (I2Cx == I2C_1)
        GPIO_SettingOutputDataBSRR(gpioB_S_i2c, GPIO_PIN6, bit);
    else if (I2Cx == I2C_2)
        GPIO_SettingOutputDataBSRR(gpioB_S_i2c, GPIO_PIN10, bit);
    else if (I2Cx == I2C_3)
        GPIO_SettingOutputDataBSRR(gpioH_S_i2c, GPIO_PIN7, bit);
}


/**
*******************************************************************************
* @ Name : DelayI2CTime
* @ Parameters: uint16_t time_delay
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Create a time delay to create clock signal for I2C and set data to SDA pin base on clock
*       + System clock using 16Mhz -> T = 62.5ns
*       -> each for loop is 62.5ns -> base on time_delay to create nescessary delay time.
*       + I2C frequency = 100kHz -> T = 10us -> time_delay = 160 to reach 10us = 160 * 62.5ns
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void DelayI2CTime(uint16_t time_delay)
{
    for (index_delay = 0; index_delay < time_delay; index_delay++)
    {
    }
}

/**
*******************************************************************************
* @ Name : SoftI2C_Start
* @ Parameters: uint8_t I2Cx
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Start I2C by setting SDA pin to LOW from HIGH while SCL pin HIGH for corresponding I2Cx
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_Start(uint8_t I2Cx)
{
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);
    DelayI2CTime(160);
    SetPortSDA(I2Cx, CLEAR);
    DelayI2CTime(80);
    SetPortSCL(I2Cx, CLEAR);
}


/**
*******************************************************************************
* @ Name : SoftI2C_Stop
* @ Parameters: uint8_t I2Cx
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Stop I2C by setting SDA pin to HIGH from LOW while SCL pin HIGH for corresponding I2Cx
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_Stop(uint8_t I2Cx)
{
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);
    SetPortSCL(I2Cx, SET);
    DelayI2CTime(80);
    SetPortSDA(I2Cx, CLEAR);
    DelayI2CTime(160);
    SetPortSDA(I2Cx, SET);
    DelayI2CTime(160);
    SetPortSCL(I2Cx, CLEAR);
    DelayI2CTime(80);
}

void HexToBinConvert(uint8_t hex, uint8_t *bin)
{
    int j;

    for (j = 7; j >= 0; j--)
    {
        if ((hex >> j) & 0x01)
            bin[7 - j] = '1';
        else
            bin[7 - j] = '0';
    }

    bin[8] = '\0'; // Null-terminate the binary string
}


/**
*******************************************************************************
* @ Name : SoftI2C_GetSDA
* @ Parameters: uint8_t I2Cx
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Get data in SDA pin for Read data function for get ACK status for corresponding I2Cx
*       + Need to set SDA mode to input mode to read data in SDA pin
* @ Return value : bool
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
bool SoftI2C_GetSDA(uint8_t I2Cx)
{
    bool SDA_return = CLEAR;
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_READ);

    if (I2Cx == I2C_1)
        SDA_return = GPIO_GetDataInOutputRes(gpioB_S_i2c, GPIO_PIN7);
    else if (I2Cx == I2C_2)
        SDA_return = GPIO_GetDataInOutputRes(gpioB_S_i2c, GPIO_PIN11);
    else if (I2Cx == I2C_3)
        SDA_return = GPIO_GetDataInOutputRes(gpioH_S_i2c, GPIO_PIN8);

    return SDA_return;
}

/**
*******************************************************************************
* @ Name : SoftI2C_SendACK
* @ Parameters: uint8_t I2Cx
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Get data in SDA pin for Read data function for get ACK status for corresponding I2Cx
* @ Return value : bool
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_SendACK(uint8_t I2Cx)
{
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);
    SetPortSCL(I2Cx, CLEAR);
    DelayI2CTime(80);
    SetPortSDA(I2Cx, CLEAR);    // Send SDA pin is LOW to send ACK
    DelayI2CTime(80);
    SetPortSCL(I2Cx, SET);
    DelayI2CTime(160);
    SetPortSCL(I2Cx, CLEAR);
}

/**
*******************************************************************************
* @ Name : SoftI2C_SendSlaveAddress
* @ Parameters: uint8_t I2Cx, uint8_t address, uint8_t modeRW
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Send slave address and mode I2C
*       + Slave address = 7bits + Read/Write 1bit   -> Send 1 byte
*       + modeRW = I2C_MODE_READ (1) / I2C_MODE_WRITE (0)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_SendSlaveAddress(uint8_t I2Cx, uint8_t address, uint8_t modeRW)
{
    uint8_t Slave_Add;
    Slave_Add = (uint8_t)address << 1 | modeRW;

    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);

    if (!SoftI2C_WriteByte(I2Cx, Slave_Add))
    {
    };

    slave_address_transmited = true;
}

/**
*******************************************************************************
* @ Name : SoftI2C_SendWordAddress
* @ Parameters: uint8_t I2Cx, uint8_t address, uint8_t modeRW
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Send word address (where you want to set/get data in memory of slave device))
*       + Seperate address into 2 bytes (upper and lower address) to send each byte/time
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_SendWordAddress(uint8_t I2Cx, uint16_t address)
{
    uint8_t Slave_WAdd_Upper;
    uint8_t Slave_WAdd_Lower;

    Slave_WAdd_Upper = (uint8_t)(address >> 8);
    Slave_WAdd_Lower = (uint8_t)((address & 0xFF00) >> 8);

    // Send Upper Address
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);
    if (!SoftI2C_WriteByte(I2Cx, Slave_WAdd_Upper))
    {
    };
    word_address_upper_transmited = true;

    // Only send Lower byte after upper byte was sent completed...
    if (word_address_upper_transmited == true)
        // Send Lower Address
        if (!SoftI2C_WriteByte(I2Cx, Slave_WAdd_Lower))
        {
        };

    word_address_lower_transmited = true;
}

/**
*******************************************************************************
* @ Name : SoftI2C_WriteByte
* @ Parameters: uint8_t I2Cx, uint8_t data
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Write 1 byte data of slave address/word address/data to SDA pin base on clock signal:
*       + 1. Convert data to bin bit (0/1)
*       + 2. Send each bit to SDA pin base on clock signal (8bits)
*       + 3. Read ACK bit from SDA pin after sending 8 bits data
*       -> If ACK is read is LOW in bit 9 -> return TRUE
* @ Return value : bool
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
bool SoftI2C_WriteByte(uint8_t I2Cx, uint8_t data)
{
    uint8_t bin[8];
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);

    //1. Convert data to bin bit (0/1)
    HexToBinConvert(data, bin);

    //2. Send each bit to SDA pin base on clock signal (8bits)
    for (index_i2c = 0; index_i2c < 8; index_i2c++)
    {
        SetPortSCL(I2Cx, CLEAR);
        DelayI2CTime(80);
        if (bin[index_i2c] == '1')
            SetPortSDA(I2Cx, SET);
        else
            SetPortSDA(I2Cx, CLEAR);
        DelayI2CTime(80);
        SetPortSCL(I2Cx, SET);
        DelayI2CTime(160);
        SetPortSCL(I2Cx, CLEAR);
    }

    //3. Read ACK bit from SDA pin after sending 8 bits data
    SetPortSCL(I2Cx, CLEAR);
    DelayI2CTime(160);
    SetPortSCL(I2Cx, SET);
    DelayI2CTime(160);

    // If ACK is read is LOW in bit 9 -> return TRUE -> completed sending 1 byte
    if (SoftI2C_GetSDA(I2Cx) == CLEAR)
    {
        SetPortSCL(I2Cx, CLEAR);
        DelayI2CTime(80);
        return true;
    }

    return false;
}

/**
*******************************************************************************
* @ Name : SoftI2C_WriteData
* @ Parameters: uint8_t I2Cx, uint8_t Slave_address, uint16_t word_address, uint8_t *data, uint8_t size
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Write data to slave device at specific address:
*       + 1. Initialize I2Cx to set SDA/SCL pin to HIGH to prepare to start I2C
*       + 2. Start I2C
*       + 3. Send Slave address + I2C mode is WRITE
*       + 4. Send Word address (where you want to set data in memory of slave device).
*       + 5. Write data[size] to slave device at specific address
*       + 6. Stop I2C
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_WriteData(uint8_t I2Cx, uint8_t Slave_address, uint16_t word_address, uint8_t *data, uint8_t size)
{
    // 1. Initialize I2Cx to set SDA/SCL pin to HIGH to prepare to start I2C
    SoftI2C_Init(I2Cx);

    // 2. Start I2C
    SoftI2C_Start(I2Cx);

    // 3. Send Slave Address + I2C mode
    if (slave_address_transmited == false)
        SoftI2C_SendSlaveAddress(I2Cx, Slave_address, I2C_MODE_WRITE);

    // 4. Send Word Address
    if (word_address_lower_transmited == false)
        SoftI2C_SendWordAddress(I2Cx, word_address);

    // 5. Write data[size] to slave device at specific address
    for (index_arr = 0; index_arr < size; index_arr++)
    {
        if (!SoftI2C_WriteByte(I2Cx, data[index_arr]))
        {
        };
    }

    // 6. Stop I2C
    SoftI2C_Stop(I2Cx);
}

/**
*******************************************************************************
* @ Name : SoftI2C_ReadByte
* @ Parameters: uint8_t I2Cx, uint8_t data
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Read 1 byte data from SDA pin base on clock signal:
*       + Need to configure SDA pin is INPUT mode to read data in SDA pin
*       + Read each bit (8bits)in SDA pin base on clock and combine to 1 byte data and return
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
uint8_t SoftI2C_ReadByte(uint8_t I2Cx)
{
    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_READ);

    uint8_t Data_read = 0;
    uint8_t index_re;

    for (index_re = 0; index_re < 8; index_re++)
    {
        SetPortSCL(I2Cx, CLEAR);
        DelayI2CTime(160);
        SetPortSCL(I2Cx, SET);
        DelayI2CTime(160);
        Data_read = (uint8_t)(Data_read << 1);

        if (SoftI2C_GetSDA(I2Cx) == SET)
            Data_read |= 0x01;
        else
            Data_read &= ~0x01;

        SetPortSCL(I2Cx, CLEAR);
    }
    return Data_read;
}

/**
*******************************************************************************
* @ Name : SoftI2C_ReadData
* @ Parameters: uint8_t I2Cx, uint8_t Slave_address, uint8_t *data, uint8_t size
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Read data from slave device at specific address:
*       + 1. Initialize I2Cx to set SDA/SCL pin to HIGH to prepare to start I2C
*       + 2. Start I2C
*       + 3. Send Slave address + I2C mode is READ
*       + 4. Send read address (where you want to get data in memory of slave device).
*       + 5. Read each byte data from slave device at specific address and store to address of data array
*       + 5.1. Send ACK bit to slave device after reading each byte data to continue reading next byte data
*       + 6. Stop I2C after reading all data.
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-05-26
*******************************************************************************
*/
void SoftI2C_ReadData(uint8_t I2Cx, uint8_t Slave_address, uint16_t read_address, uint8_t *data, uint8_t size)
{
    // 1. Initialize I2Cx to set SDA/SCL pin to HIGH to prepare to start I2C
    SoftI2C_Init(I2Cx);

    // 2. Start I2C
    SoftI2C_Start(I2Cx);

    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_WRITE);

    // 3. Send Slave Address + I2C mode is READ
    if (slave_address_transmited == false)
        SoftI2C_SendSlaveAddress(I2Cx, Slave_address, I2C_MODE_READ);

    // 4. Send Read Address
    if (word_address_lower_transmited == false)
        SoftI2C_SendWordAddress(I2Cx, read_address);

    SoftI2C_GPIOConfiguration(I2Cx, I2C_MODE_READ);

    // 5. Read each byte data from slave device at specific address and store to address of data array
    for (index_arr = 0; index_arr < size; index_arr++)
    {
        data[index_arr] = SoftI2C_ReadByte(I2Cx);

        // 5.1. Send ACK bit to slave device after reading each byte data to continue reading next byte data
        SoftI2C_SendACK(I2Cx);
    }

    // 6. Stop I2C after reading all data.
    SoftI2C_Stop(I2Cx);
}
