#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_adc_cal.h>

#include <esp_now.h>            // Inclui a biblioteca esp_now para o uso do protocolo de comunicação ESP-NOW
#include <WiFi.h>               // Inclui a biblioteca WiFi para configuração da rede sem fio
//98:CD:AC:A9:7D:A4   espsafada
// 3C:61:05:15:53:10 MAC ALIVE
uint8_t slaveMacAddress[] = {0x3C, 0x61, 0x05, 0x15, 0x53, 0x10};  // Define o endereço MAC do dispositivo escravo. Coloque o endereço MAC de sua placa aqui
typedef struct DataStruct {  // Define a estrutura DataStruct para troca de informações
  float temperature;   
  float voltage;   
  float current;
} DataStruct;

DataStruct message;

esp_now_peer_info_t peerInfo;  // Cria uma estrutura esp_now_peer_info_t, que é utilizada para registrar informações sobre um peer (dispositivo) que será adicionado à rede ESPNOW


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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print(F("\r\n Master packet sent:\t"));
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


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


  WiFi.disconnect();    // Desconecta de qualquer rede WiFi previamente conectada
  WiFi.mode(WIFI_STA);  // Define o modo WiFi como Station (cliente)

  Serial.print("Endereço MAC: ");
  Serial.println(WiFi.macAddress()); // retorna o endereço MAC do dispositivo

  if (esp_now_init() != ESP_OK) {        // Inicializa o ESP-NOW e verifica se há erros
    delay(500);                           // Espera por 2,5 segundos
    ESP.restart();                       // Reinicia o dispositivo
  }
  
  esp_now_register_send_cb(OnDataSent);             // Registra a função de callback que é chamada quando os dados são enviados
  memcpy(peerInfo.peer_addr, slaveMacAddress, 6);  // Copia o endereço MAC do escravo para a estrutura peerInfo
  peerInfo.channel = 0;                            // Define o canal de comunicação como 0 na estrutura peerInfo
  peerInfo.encrypt = false;                        // Define a encriptação como desativada na estrutura peerInfo

  // Tenta adicionar o escravo à lista de pares de comunicação ESP-NOW e verifica se foi bem sucedido
  if (esp_now_add_peer(&peerInfo) != ESP_OK){  // Caso não seja possível adicionar o escravo, exibe mensagem de falha no display, acende LED vermelho e reinicia o dispositivo
    Serial.print("Falha ao adicionar peer");
    delay(500);
    ESP.restart();
  }
}

void loop(void)
{ 
  measureVoltage();
  measureTemperature();

  esp_now_send(slaveMacAddress, (uint8_t *)&message, sizeof(DataStruct));  // Envie os dados para o endereço MAC do dispositivo escravo
  delay(2000);  
}

// function to print a device address temperature
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
  message.temperature = sensors.getTempC(insideThermometer);

  #ifdef printTemperature  
    if(message.temperature == DEVICE_DISCONNECTED_C){
    Serial.println("Error: Could not read temperature data");
    }
    Serial.printf("Temp C: %f \r\n", message.temperature);
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

  message.voltage = (voltage * Calibration_factor / (R2_Value/(R1_Value + R2_Value)))/1000;

    #ifdef printVontage
		   Serial.printf("Voltage: %.2f \r\n", message.voltage);
    #endif
}

void measurecurrent(){
}

