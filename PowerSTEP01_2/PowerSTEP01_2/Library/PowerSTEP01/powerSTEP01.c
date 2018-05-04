/*
 * powerSTEP01_Lib.c
 *
 * Created: 23/03/2018 9:19:23
 * Author : Robbie
 */ 

#include "powerSTEP01.h"

static volatile uint8_t numberOfDevices;

struct motorParam{
	uint8_t RunKval;
	uint8_t HoldKval;
	uint8_t AccKval;
	uint8_t DecKval;
	unsigned long maxSpeed;
	unsigned long Acc;
	unsigned long Dec;
}motorParam;

uint8_t SPIXfer(uint8_t data)
{
	uint8_t temp;
	temp = transfer(data);
	return temp;
}

uint8_t SPIXferMotors(uint8_t motor, uint8_t data)
{
	uint8_t temp[3];
	uint8_t retval;
	uint8_t j;
	if (motor < 1) motor = 1;
	if (motor > 3) motor = 3;
	CSM_Low;
	for(j=0; j<3; j++)
	{
		if(motor == (3-j))
		{
			temp[j] = SPIXfer(data);
		}
		else
		{
			temp[j] = SPIXfer(0x00);
		}
	}		
	CSM_High;
	retval = temp[2-(motor-1)];
	return retval;	
	/*
	uint8_t temp;
	CSM_Low;
	temp = transfer(data);
	CSM_High;
	return temp;
	*/
}

static long SPIXferParam(uint8_t motor, unsigned long data, uint8_t bitLen)
{
	uint8_t byteLen = bitLen/8;
	if (bitLen%8 > 0) byteLen++;
	
	uint8_t temp;
	unsigned long retVal = 0;
	
	for (uint8_t i = 0; i< byteLen; i++)
	{
		retVal = retVal << 8;
		temp = SPIXferMotors(motor, (uint8_t)(data>>((byteLen-i-1)*8)));
		retVal |= temp;
	}
	unsigned long mask = 0xFFFFFFFF >> (32-bitLen);
	return retVal & mask;
}

void motorControl_Init(uint8_t motor)
{
	while(busyCheck(motor));
	uint8_t i;
	uint16_t config;
	
	motorParam.RunKval = 0x30;
	motorParam.HoldKval = 0x30;
	motorParam.AccKval = 0x30;
	motorParam.DecKval = 0x30;
	motorParam.maxSpeed = 200;
	motorParam.Acc = 100;
	motorParam.Dec = 100;
	
	powerSTEP01_GPIO_Init();
	
	#ifdef reset
	releaseReset();
	#endif
	_delay_ms(1);
	CSM_Low;
	SPIXfer(0x00);
	SPIXfer(0x00);
	SPIXfer(0x00);
	CSM_High;
	//setDeviceParam();
	
	config = getParam(motor, CONFIG);
	setSlewRate(motor, SR_520V_us);
	configStepMode(motor, STEP_SEL_1_128);
	setOCD_TH(motor, 0x1F);
	setOCShutdown(motor, OC_SD_ENABLE);
	setPWMFreq(motor, PWM_DIV_1, PWM_MUL_0_75);
	setVoltageComp(motor, VS_COMP_DISABLE);
	setSwitchMode(motor, SW_USER);
	setOscMode(motor, CONFIG_INT_16MHZ_OSCOUT_8MHZ);
	setSTALL_TH(motor, 0x10);
	setMaxSpeed(motor, motorParam.maxSpeed);
	setRunKval(motor, motorParam.RunKval);
	setHoldKval(motor, motorParam.HoldKval);
	setAccKval(motor, motorParam.AccKval);
	setDecKval(motor, motorParam.DecKval);
	setAcc(motor, motorParam.Acc);
	setDec(motor, motorParam.Dec);
	
	setParam(motor, ALARM_EN, 0xAF); //8F
	getStatus(motor);

}

void motorsResetPos(void)
{
	/*
	motor 1 move R untill einderit
	motor 1 move L untill home
	motor 1 setHome()
	motor 2 move R untill einderit
	motor 2 move L untill home
	motor 2 setHome()
	motor 3 move R untill einderit
	motor 3 move L untill home
	motor 3 setHome()
	*/
}

void flagHandler(void)
{
	uint8_t i;
	for(i=1; i<4; i++)
	{
		uint16_t status = (uint16_t)getParam(i, STATUS);
	
		if ((status & 0x0080 ) == 0x0080) //cmd err
		{
			getStatus(i); 		
		}
		if ((status & 0xC000 ) == 0xC000) //stall
		{
			hardStop(i);	
			getStatus(i);
		}
		if ((status & 0x2000 ) == 0x2000) //ocd
		{
			hardHiZ(i);
		}
		if ((status & 0x0800 ) == 0x0800) //warming
		{
			setRunKval(i, ((motorParam.RunKval) / 2));		//Kvals to 50% of  set value
			setAccKval(i, ((motorParam.AccKval) / 2));
			setDecKval(i, ((motorParam.DecKval) / 2));		
			setMaxSpeed(i, ((motorParam.maxSpeed) / 4));				//maxspeed  to 25% of  set value
		}
		if ((status & 0x0800 ) == 0x1000) //th bridge sd
		{
			hardHiZ(i);
			setRunKval(i, 0);		//Kvals to 0 so chip can not dissipate energy and generate more heat
			setAccKval(i, 0);
			setDecKval(i, 0);
			setHoldKval(i, 0);
		
		}
		if ((status & 0x0800 ) == 0x1800) //th device sd
		{
			//seriuously though, how do you even read this if the device shuts down...
			//PORTE &= ~(1<<reset);	//	|	|!
								//	||	|_
		}
		if ((status & 0xFA80 ) == 0xE280)	//normal opperation
		{
			setRunKval(i, (motorParam.RunKval));		//set back to normal values
			setAccKval(i, (motorParam.AccKval));
			setDecKval(i, (motorParam.DecKval));
			setHoldKval(i, (motorParam.HoldKval));
			setMaxSpeed(i, (motorParam.maxSpeed));
		}
	}
}

// There are a number of clock options for this chip- it can be configured to
//  accept a clock, drive a crystal or resonator, and pass or not pass the
//  clock signal downstream. Theoretically, you can use pretty much any
//  frequency you want to drive it; practically, this library assumes it's
//  being driven at 16MHz. Also, the device will use these bits to set the
//  math used to figure out steps per second and stuff like that.
void setOscMode(uint8_t motor, int oscillatorMode)
{
	unsigned long configVal = getParam(motor, CONFIG);
	// These bits are CONFIG 3:0, mask is 0x000F
	configVal &= ~(0x000F);
	//Now, OR in the masked incoming value.
	configVal |= (0x000F&oscillatorMode);
	setParam(motor, CONFIG, configVal);
}

// The switch input can either hard-stop the driver _or_ activate an interrupt.
//  This bit allows you to select what it does.
void setSwitchMode(uint8_t motor, int switchMode)
{
	unsigned long configVal = getParam(motor, CONFIG);
	// This bit is CONFIG 4, mask is 0x0010
	configVal &= ~(0x0010);
	//Now, OR in the masked incoming value.
	configVal |= (0x0010 & switchMode);
	setParam(motor, CONFIG, configVal);
}

void setVoltageComp(uint8_t motor, int vsCompMode)
{
	unsigned long configVal = getParam(motor, CONFIG);
	// This bit is CONFIG 5, mask is 0x0020
	configVal &= ~(0x0020);
	//Now, OR in the masked incoming value.
	configVal |= (0x0020&vsCompMode);
	setParam(motor, CONFIG, configVal);
}

void setPWMFreq(uint8_t motor, uint8_t divisor, uint8_t multiplier)
{
	unsigned long configVal = getParam(motor, CONFIG);
	
	configVal &= ~(F_PWM_DIV);
	configVal &= ~(F_PWM_MUL);
	configVal |= ((F_PWM_DIV & divisor)|(F_PWM_MUL & multiplier));
	setParam(motor, CONFIG, configVal);	
}

void setOCShutdown(uint8_t motor, uint8_t OCShutdown)
{
	unsigned long configVal = getParam(motor, CONFIG);
	
	configVal &= ~(0x0080);
	configVal |= (0x0080 & OCShutdown);
	setParam(motor, CONFIG, configVal);
}

void configStepMode(uint8_t motor, uint8_t stepMode)
{
	uint8_t stepModeConfig = (uint8_t)getParam(motor, STEP_MODE);
	stepModeConfig &= 0xF8;
	
	stepModeConfig |= (stepMode &0x07);
	setParam(motor, STEP_MODE, stepModeConfig);
}

void setMaxSpeed(uint8_t motor, unsigned long stepsPerSecond)
{
	unsigned long integerSpeed = maxSpdCalc(stepsPerSecond);
	
	setParam(motor, MAX_SPEED, integerSpeed);
}

void setSlewRate(uint8_t motor, uint8_t slewRate)
{
	unsigned long configVal = getParam(motor, GATECFG1);
	
	configVal &= (0xFF00);
	
	configVal |= (0x00FF&slewRate);
	setParam(motor, GATECFG1, configVal);
}


void setRunKval(uint8_t motor, uint8_t kval)
{
	setParam(motor, KVAL_RUN, kval);
}

uint8_t getRunKval(uint8_t motor)
{
	return (uint8_t)getParam(motor, KVAL_RUN);
}

void setHoldKval(uint8_t motor, uint8_t kval)
{
	setParam(motor, KVAL_HOLD, kval);
}

uint8_t getHoldKval(uint8_t motor)
{
	return (uint8_t)getParam(motor, KVAL_HOLD);
}

void setAccKval(uint8_t motor, uint8_t kval)
{
	setParam(motor, KVAL_ACC, kval);
}

uint8_t getAccKval(uint8_t motor)
{
	return (uint8_t)getParam(motor, KVAL_ACC);
}

void setDecKval(uint8_t motor, uint8_t kval)
{
	setParam(motor, KVAL_DEC, kval);
}

uint8_t getDecKval(uint8_t motor)
{
	return (uint8_t)getParam(motor, KVAL_DEC);
}

void setOCD_TH(uint8_t motor, uint8_t OCD)
{
	setParam(motor, OCD_TH, OCD);
}

uint8_t getOCD_TH(uint8_t motor)
{
	return (uint8_t)getParam(motor, OCD_TH);
}

void setDeviceParam(uint8_t motor)
{
	
}

void setSTALL_TH(uint8_t motor, uint8_t STALL)
{
	setParam(motor, STALL_TH, STALL);
}

void setAcc(uint8_t motor, unsigned long stepsPerSecondPerSecond)
{
	unsigned long integerAcc = accCalc(stepsPerSecondPerSecond);
	setParam(motor, ACC, integerAcc);
}

void setDec(uint8_t motor, unsigned long stepsPerSecondPerSecond)
{
	unsigned long integerDec = decCalc(stepsPerSecondPerSecond);
	setParam(motor, DECEL, integerDec);
}

uint16_t getStatus(uint8_t motor)
{
	uint16_t temp;
	SPIXferMotors(motor, GET_STATUS);
	temp = SPIXferMotors(motor, 0x00);
	temp = temp << 8;
	temp |= SPIXferMotors(motor, 0x00);
	return temp;
}


void setParam(uint8_t motor, uint8_t param, uint32_t data)
{
	param |= SET_PARAM;
	
	SPIXferMotors(motor, param);
	paramHandler(motor, param, data);
	
}

long getParam(uint8_t motor, uint8_t param)
{
	SPIXferMotors(motor, (param | GET_PARAM));
	return paramHandler(motor, param, 0);
}

void powerSTEP01_GPIO_Init(void)
{
	//initializing CS as output
	DDR_SPI |= (1<<DD_CS0);
	PORT_SPI |= (1<<CSMotor);
#if defined(__AVR_ATmega32U4__) 
	//Initializing Busy and Flag as input+pullup
	DDRC &= ~(1<<DDC7) | ~(DDC6);
	PORTC |= (1<<PORTC7) | (1<<PORTC6);
#endif

	#ifdef reset
		DDR_reset |= (1<<DD_reset);
		PORT_reset &= ~(1<<reset);
	#endif
}

void releaseReset(void)
{
	PORT_reset |= (1<<reset);
}

long paramHandler(uint8_t motor, uint8_t param, uint32_t value)
{
	long retVal = 0;   // This is a temp for the value to return.
	
	// This switch structure handles the appropriate action for each register.
	//  This is necessary since not all registers are of the same length, either
	//  bit-wise or byte-wise, so we want to make sure we mask out any spurious
	//  bits and do the right number of transfers. That is handled by the xferParam()
	//  function, in most cases, but for 1-byte or smaller transfers, we call
	//  SPIXfer() directly.
	switch (param)
	{
		// ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
		//  in two's complement. At power up, this value is 0. It cannot be written when
		//  the motor is running, but at any other time, it can be updated to change the
		//  interpreted position of the motor.
		case ABS_POS:
		retVal = SPIXferParam(motor, value, 22);
		break;
		// EL_POS is the current electrical position in the step generation cycle. It can
		//  be set when the motor is not in motion. Value is 0 on power up.
		case EL_POS:
		retVal = SPIXferParam(motor, value, 9);
		break;
		// MARK is a second position other than 0 that the motor can be told to go to. As
		//  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
		case MARK:
		retVal = SPIXferParam(motor, value, 22);
		break;
		// SPEED contains information about the current speed. It is read-only. It does
		//  NOT provide direction information.
		case SPEED_:
		retVal = SPIXferParam(motor, 0, 20);
		break;
		// ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF
		//  to get infinite acceleration/decelaeration- there is no way to get infinite
		//  deceleration w/o infinite acceleration (except the HARD STOP command).
		//  Cannot be written while motor is running. Both default to 0x08A on power up.
		// AccCalc() and DecCalc() functions exist to convert steps/s/s values into
		//  12-bit values for these two registers.
		case ACC:
		retVal = SPIXferParam(motor, value, 12);
		break;
		case DECEL:
		retVal = SPIXferParam(motor, value, 12);
		break;
		// MAX_SPEED is just what it says- any command which attempts to set the speed
		//  of the motor above this value will simply cause the motor to turn at this
		//  speed. Value is 0x041 on power up.
		// MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
		//  for this register.
		case MAX_SPEED:
		retVal = SPIXferParam(motor, value, 10);
		break;
		// MIN_SPEED controls two things- the activation of the low-speed optimization
		//  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
		//  is the 13th bit, and when it is set, the minimum allowed speed is automatically
		//  set to zero. This value is 0 on startup.
		// MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
		//  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
		case MIN_SPEED:
		retVal = SPIXferParam(motor, value, 13);
		break;
		// FS_SPD register contains a threshold value above which microstepping is disabled
		//  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
		// FSCalc() function exists to convert steps/s value into 10-bit integer for this
		//  register.
		case FS_SPD:
		retVal = SPIXferParam(motor, value, 10);
		break;
		// KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
		//  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
		// The implications of different KVAL settings is too complex to dig into here, but
		//  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
		//  HOLD may result in excessive power dissipation when the motor is not running.
		case KVAL_HOLD:
		retVal = SPIXferParam(motor, value, 8);
		break;
		case KVAL_RUN:
		retVal = SPIXferParam(motor, value, 8);
		break;
		case KVAL_ACC:
		retVal = SPIXferParam(motor, value, 8);
		break;
		case KVAL_DEC:
		retVal = SPIXferParam(motor, value, 8);
		break;
		// INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
		//  compensation functionality. Please see the datasheet for details of this
		//  function- it is too complex to discuss here. Default values seem to work
		//  well enough.
		case INT_SPD:
		retVal = SPIXferParam(motor, value, 14);
		break;
		case ST_SLP:
		retVal = SPIXferParam(motor, value, 8);
		break;
		case FN_SLP_ACC:
		retVal = SPIXferParam(motor, value, 8);
		break;
		case FN_SLP_DEC:
		retVal = SPIXferParam(motor, value, 8);
		break;
		// K_THERM is motor winding thermal drift compensation. Please see the datasheet
		//  for full details on operation- the default value should be okay for most users.
		case K_THERM:
		value &= 0x0F;
		retVal = SPIXferParam(motor, value, 8);
		break;
		// ADC_OUT is a read-only register containing the result of the ADC measurements.
		//  This is less useful than it sounds; see the datasheet for more information.
		case ADC_OUT:
		retVal = SPIXferParam(motor, value, 8);
		break;
		// Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
		//  A set of defined constants is provided for the user's convenience. Default
		//  value is 3.375A- 0x08. This is a 4-bit value.
		case OCD_TH:
		value &= 0x1F;
		retVal = SPIXferParam(motor, value, 8);
		break;
		// Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
		//  4A in 31.25mA steps. This is a 7-bit value.
		case STALL_TH:
		value &= 0x1F;
		retVal = SPIXferParam(motor, value, 8);
		break;
		// STEP_MODE controls the microstepping settings, as well as the generation of an
		//  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
		//  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
		//  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
		//  of the output signal relative to the full-step frequency; see datasheet for
		//  that relationship as it is too complex to reproduce here.
		// Most likely, only the microsteps per step value will be needed; there is a set
		//  of constants provided for ease of use of these values.
		case STEP_MODE:
		retVal = SPIXferParam(motor, value, 8);
		break;
		// ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
		//  is provided to make this easy to interpret. By default, ALL alarms will trigger the
		//  FLAG pin.
		case ALARM_EN:
		retVal = SPIXferParam(motor, value, 8);
		break;
		// GATECFG1 controls driver transistor gate discharging and clock source monitoring
		case GATECFG1:
		retVal = SPIXferParam(motor, value, 16);
		break;
		// GATECFG2 controls driver dead time and blanking
		case GATECFG2:
		retVal = SPIXferParam(motor, value, 8);
		break;
		// CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
		//  set of reasonably self-explanatory constants is provided, but users should refer
		//  to the datasheet before modifying the contents of this register to be certain they
		//  understand the implications of their modifications. Value on boot is 0x2E88; this
		//  can be a useful way to verify proper start up and operation of the dSPIN chip.
		case CONFIG:
		retVal = SPIXferParam(motor, value, 16);
		break;
		// STATUS contains read-only information about the current condition of the chip. A
		//  comprehensive set of constants for masking and testing this register is provided, but
		//  users should refer to the datasheet to ensure that they fully understand each one of
		//  the bits in the register.
		case STATUS:  // STATUS is a read-only register
		retVal = SPIXferParam(motor, 0, 16);;
		break;
		default:
		SPIXferMotors(motor, (uint8_t)value);
		break;
	}
	return retVal;
}

unsigned long maxSpdCalc (unsigned long stepsParSec)
{
	unsigned long temp = ceil(stepsParSec* .065536);
	if (temp > 0x000003FF) return 0x000003FF;
	else return temp;
}

unsigned long spdCalc(unsigned long stepsPerSec)
{
	unsigned long temp = stepsPerSec * 67.106;
	if( temp > 0x000FFFFF) return 0x000FFFFF;
	else return temp;
}

unsigned long accCalc(unsigned long stepsPerSecPerSec)
{
	unsigned long temp = stepsPerSecPerSec * 0.137438;
	if(temp > 0x00000FFF) return 0x00000FFF;
	else return temp;
}

unsigned long decCalc(unsigned long stepsPerSecPerSec)
{
	unsigned long temp = stepsPerSecPerSec * 0.137438;
	if(temp > 0x00000FFF) return 0x00000FFF;
	else return temp;
}

int busyCheck(uint8_t motor)
{
	if ((getParam(motor, STATUS) & 0x0002) == 0x0002) //busy pin
	{
		return 0;
	}
		
	else    
	{
		return 1;
	}                     	
}

void run(uint8_t motor, uint8_t dir, unsigned long stepsPerSec)
{	
	while(busyCheck(motor));
	SPIXferMotors(motor, (RUN | (0x01 & dir)));
	unsigned long integerSpeed = spdCalc(stepsPerSec);
	if (integerSpeed > 0xFFFFF) integerSpeed = 0xFFFFF;

	uint8_t* bytePointer = (uint8_t*)&integerSpeed;

	for (int8_t i = 2; i >= 0; i--)
	{
		SPIXferMotors(motor, bytePointer[i]);
	}
}

void move(uint8_t motor, uint8_t dir, unsigned long numSteps)
{	
	while(busyCheck(motor));
	SPIXferMotors(motor, (MOVE | (0x01 & dir)));
	if (numSteps > 0x3FFFFF) numSteps = 0x3FFFFF;

	uint8_t* bytePointer = (uint8_t*)&numSteps;
	
	for (int8_t i = 2; i >= 0; i--)
	{
		SPIXferMotors(motor, bytePointer[i]);
	}
}

void softStop(uint8_t motor)
{
	SPIXferMotors(motor, SOFT_STOP);
	while(busyCheck(motor));
}

void hardStop(uint8_t motor)
{	
	SPIXferMotors(motor, HARD_STOP);
}

void softHiZ(uint8_t motor)
{
	SPIXferMotors(motor, SOFT_HIZ);
}

void hardHiZ(uint8_t motor)
{
	SPIXferMotors(motor, HARD_HIZ);
}