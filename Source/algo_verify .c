#include "algo_verify.h"

//计算BCC校验码
uint8_t getBccCode(const uint8_t *buff, uint16_t StartIdx, int16_t len)
{
	uint8_t rtn = 0;
	uint16_t i = 0;
	if (len > 0)
	{
		rtn = buff[StartIdx];
		for (i = 0; i < len - 1; i++)
		{
			rtn = (uint8_t)(rtn ^ buff[i + StartIdx + 1]);
		}
	}
	return rtn;
}

//校验车架号
uint8_t CheckVin(char* vin)
{
	const uint8_t weight_coefficient[17] = {8,7,6,5,4,3,2,10,0,9,8,7,6,5,4,3,2};
	uint8_t i;
	uint8_t result = 0;
	uint8_t value_temp = 0;
	uint16_t step_temp = 0;
	if(vin[0] != 'L')//非中国大陆VIN码
	{
		return 0;
	}
	for (i = 0; i != 17; ++i) 
	{
		if (vin[i] >= '0' && vin[i] <= '9') 
		{//Arabic numerals
			value_temp = vin[i] - '0';
		}
		else if (vin[i]>='A' && vin[i] <= 'Z' && vin[i] != 'I' && vin[i] != 'O' && vin[i] != 'Q') 
		{//upper-roman
			if (vin[i] >= 'A' && vin[i] <= 'I')
				value_temp = vin[i] - 'A' + 1;
			else if(vin[i]>='J' && vin[i]<='R')
				value_temp = vin[i] - 'J' + 1;
			else if(vin[i]>='S' && vin[i]<='Z')
				value_temp = vin[i] - 'S' + 2;
		}
		else
		{
			return 0;
		}
		if (i != 8) 
		{
			step_temp += value_temp * weight_coefficient[i];
		}
	}
	result = (uint8_t)(step_temp % 11);
	if (result == 10) 
	{
		result = 'X';
	}
	else 
	{
		result += '0';
	}
	if(result != vin[8])
	{
		return 0;
	}
	return 1;
}
