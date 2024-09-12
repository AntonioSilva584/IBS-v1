#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_adc_cal.h>

#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

void printAddress(DeviceAddress deviceAddress);
void measureTemperature();
void measureVoltage();
void measurecurrent();

#define printVontage
#define printTemperature

void setup(void)
{
  Serial.begin(115200); 

  sensors.begin(); 

  // report parasite power requirements  
  if(sensors.isParasitePowerMode()){Serial.println("Parasite power is ON");}
  else{Serial.println("Parasite power is OFF");}

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");
  //printAddress(insideThermometer);  

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);  
}

void loop(void)
{ 
  measureVoltage();
  measureTemperature();
  delay(50);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void measureTemperature()
{
  sensors.requestTemperatures();      //Send the command to get temperatures 
  float tempC = sensors.getTempC(insideThermometer);

  #ifdef printTemperature  
    if(tempC == DEVICE_DISCONNECTED_C){
    Serial.println("Error: Could not read temperature data");
    }
    Serial.printf("Temp C: %f \r\n", tempC);
  #endif
}

void measureVoltage()
{
  double Calibration_factor = 1.0195;
	float R1_Value = 47000 , R2_Value = 3300; 

  esp_adc_cal_characteristics_t adc_cal;//Estrutura que contem as informacoes para calibracao
	adc1_config_width(ADC_WIDTH_BIT_12);//Configura a resolucao
	adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_0);//Configura a atenuacao  
  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);//Inicializa a estrutura de calibracao

    if (adc_type == ESP_ADC_CAL_VAL_EFUSE_VREF){Serial.printf("ADC CAL", "Vref eFuse encontrado: %umV", adc_cal.vref);}
    else if (adc_type == ESP_ADC_CAL_VAL_EFUSE_TP){Serial.printf("ADC CAL", "Two Point eFuse encontrado");}
    else{ Serial.printf("ADC CAL", "Nada encontrado, utilizando Vref padrao: %umV", adc_cal.vref);}
	
  uint32_t voltage = 0;
		for (int i = 0; i < 100; i++){
			voltage += adc1_get_raw(ADC1_CHANNEL_5);//Obtem o valor RAW do ADC
			delayMicroseconds(30);
		}
		voltage /= 100;
    voltage = esp_adc_cal_raw_to_voltage(voltage, &adc_cal);//Converte e calibra o valor lido (RAW) para mV

  float AverageVolt = voltage * Calibration_factor / (R2_Value/(R1_Value + R2_Value));

    #ifdef printVontage
		   Serial.printf("Voltage: %.2f \r\n", AverageVolt/1000);
    #endif
}

void measurecurrent(){


}