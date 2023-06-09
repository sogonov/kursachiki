#include <stdint.h>
#include "stdlib.h"
#include "ads1293.h"
#include "main.h"
SPI_HandleTypeDef ads_hspi;

void cs_low_ads(){
	  HAL_GPIO_WritePin(GPIOA, CS_ADS_Pin, GPIO_PIN_RESET); //CS установлен в ноль
	  };
void cs_high_ads(){
	 	  HAL_GPIO_WritePin(GPIOA, CS_ADS_Pin, GPIO_PIN_SET); //CS установлен в единицу
	 	  };


void ADS1293_write(uint8_t address, uint8_t value){
	uint8_t data[2];
	uint8_t command_field = address & ADS1293_WRITE_BIT;//формируем command field
	data[0] = command_field;
	data[1] = value; //байт, который запишем в регистр

	cs_low_ads();
	HAL_SPI_Transmit(&ads_hspi, data, 2, 100);
	//передача двух байт, первый - адрес регистра,
	//второй - то, что хотим записать
	cs_high_ads();
	HAL_Delay(10);
};
uint8_t ADS1293_read(uint8_t address){
	uint8_t command_field = address | ADS1293_READ_BIT;
	uint8_t read_val;
	cs_low_ads();
	HAL_SPI_Transmit(&ads_hspi, &command_field, 1, 100);
	HAL_SPI_Receive(&ads_hspi, &read_val, 1, 100);
	cs_high_ads();
	return read_val;
};

void ADS1293_INIT(){
	//power-down - выкл, stand-by - выкл, start conversion выкл
	ADS1293_write(ADS1293_CONFIG_REG,0b000);
	//POS1=IN2, NEG1=IN1
	ADS1293_write(ADS1293_FLEX_CH1_CN_REG,0b00010001);
	//POS2=IN3, NEG2=IN1
	ADS1293_write(ADS1293_FLEX_CH2_CN_REG,0b00011001);
	// POS3=IN5, NEG3=IN6
	ADS1293_write(ADS1293_FLEX_CH3_CN_REG,0b00101110);
	//common-mode detector -вкл на IN1,IN2,IN3
	ADS1293_write(ADS1293_CMDET_EN_REG,0b00000111);
	//подкл IN4 к RLD control
	ADS1293_write(ADS1293_RLD_CN_REG,0b0000100);
	//подключаем Wilson control
	ADS1293_write(ADS1293_WILSON_EN1_REG,0b001);
	ADS1293_write(ADS1293_WILSON_EN2_REG,0b010);
	ADS1293_write(ADS1293_WILSON_EN3_REG,0b011);
	ADS1293_write(ADS1293_WILSON_CN_REG,0b01);

	//используем внешний кварцевый резонатор
	ADS1293_write(ADS1293_OSC_CN_REG,0b100);


	//настраиваем цифровой фильтр
	ADS1293_write(ADS1293_R2_RATE_REG,0b1000);

	ADS1293_write(ADS1293_R3_RATE1_REG,0b100);
	ADS1293_write(ADS1293_R3_RATE2_REG,0b100);
	ADS1293_write(ADS1293_R3_RATE3_REG,0b100);

	//data ready bar на channel 1 ECG
	ADS1293_write(ADS1293_DRDYB_SRC_REG,0b1000);

	//вкл чтение с CH1 CH2 CH3, вкл status data loop read-back
	ADS1293_write(ADS1293_CH_CNFG_REG,0b1110001);

};

void ADS1293_start_conv(){
	ADS1293_write(ADS1293_CH_CNFG_REG,0b001);
}

void ADS1293_stop_conv(){
	ADS1293_write(ADS1293_CH_CNFG_REG,0b000);
}


void ADS1293_stream_read(uint32_t* databuffer){

	uint32_t result = 0x000000000000000000000000;
	  //////////// Read lead I
	  result=ADS1293_read(ADS1293_DATA_CH1_ECG_H_REG);
	  result=result<<8;
	  result|=ADS1293_read(ADS1293_DATA_CH1_ECG_M_REG);
	  result=result<<8;
	  result|=ADS1293_read(ADS1293_DATA_CH1_ECG_L_REG);
	  databuffer[0]=result;

	  /////////// Read lead II
	  result=ADS1293_read(ADS1293_DATA_CH2_ECG_H_REG);
	  result=result<<8;
	  result|=ADS1293_read(ADS1293_DATA_CH2_ECG_M_REG);
	  result=result<<8;
	  result|=ADS1293_read(ADS1293_DATA_CH2_ECG_L_REG);
	  databuffer[1]=result;

	  //////Read lead V1
	  result=ADS1293_read(ADS1293_DATA_CH3_ECG_H_REG);
	  result=result<<8;
	  result|=ADS1293_read(ADS1293_DATA_CH2_ECG_M_REG);
	  result=result<<8;
	  result|=ADS1293_read(ADS1293_DATA_CH2_ECG_L_REG);
	  databuffer[2]=result;


}



