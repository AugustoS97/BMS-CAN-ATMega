/*
   AUTOR: AUGUSTO SAMUEL HERNÁNDEZ MARTÍN
   FECHA: 25/04/2021
   NOMBRE: SMART BMS
   VERSIÓN: V.0.8
   DESCRIPCIÓN: FIRMWARE DEL MICROCONTROLADOR ALOJADO EN LA PCB DEL CONFIGURADOR. SU FUNCIÓN ES CONFIGURAR EL BMS Y OBTENER SUS PARÁMETROS DESDE EL CAN BUS PARA DEVOLVERLOS POR SERIAL
*/
#include "config.h"
#include "Arduino.h"
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"
#include "LTC68042.h"
#include "SPI.h"
#include "mcp2515.h"
#include "EEPROM.h"
#include <LowPower.h>

MCP2515 mcp2515(10); //Se declara el objeto de la clase MCP2515

//Struct para almacenar mensaje CAN recibido de configuracion
struct can_frame can_msg;

//Struct para almacenar los mensajes CAN de la batería
struct can_frame canBatMsg1;
struct can_frame canBatMsg2;
struct can_frame canBatMsg3;

//Struct para almacenar los mensajes CAN de las Temperaturas
struct can_frame canTempMsg1;
struct can_frame canTempMsg2;
struct can_frame canTempMsg3;
struct can_frame canTempMsg4;

//Struct para almacenar el mensaje CAN de la corriente
struct can_frame canCurrentMsg; 

struct can_frame warning; //Mensaje CAN de warning de baja tensión batería, batería en sobretensión o anomalía en Tª

float UVBAT_THR = 12.00 ; //Tensión minima de la batería
float OVBAT_THR = 16.00 ; //Tensión maxima de la batería
float UV_THR = 3.0 ; //Tension minima de una celda
float OV_THR = 4.0; //Tension maxima de una celda

float MAX_VCELL_DIFF = 0.005; //Diferencia de tension maxima entre celdas

uint8_t TOTAL_CELL = 4; //Número de celdas totales
const uint8_t TOTAL_IC = 1;//Númo de IC conectados por isoSPI, que serán direccionados comenzando en 0

//Parametros de las NTC
uint8_t N_NTC = 32; //Numero de NTCs conectadas al BMS
const int BETTA = 1.00000 ; //Betta de la NTC. Hay que ver si este valor es entero o decimal
const int To = 25 ; //To es 25ºC para NTC
const int Ro = 1000 ; //Ro es 1kOhm
const int Vcc = 5; //Vcc es de 5V para sensor de NTC


float cell_temp [32]; //Array 1D con las temperaturas medidas

uint16_t cell_codes[TOTAL_IC][12];//Array 2D de códigos de celda. Se indexan en forma de matriz con fila el IC que mide y columna la celda de la batería
/*
   Sigue el siguiente formado

  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
*/

uint8_t tx_cfg[TOTAL_IC][6];
/*
  El tx_cfg[][6] almacena los datos de configuración del LTC6804 que se van a escribir a los IC que están en Daisy Chain.
  La configuración de los LTC6804 se debe almacenar en bloques de 6 bytes.
  El array 2D es como se muestra a continuación.

  |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
  |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
  |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

*/

uint8_t rx_cfg[TOTAL_IC][8];

/*
  El rx_cfg[][8] almacena los datos que son leidos de los LTC6804-1 en Daisy-chain.
  Los datos de cada IC son almacenados en bloques de 8 bytes.
  A continuación se muestra la distribución del array 2D.

  |rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
  |---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
  |IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |

*/



uint16_t aux_codes[TOTAL_IC][6];

/*
  The GPIO codes will be stored in the aux_codes[][6] array in the following format:

  |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
  |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
  |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
*/





/************************************
  Cargar la configuiracion recibida por CAN en la EEPROM
  **********************************/

void can_msg_rcv(){
  noInterrupts();
  if(mcp2515.readMessage(&can_msg) == MCP2515::ERROR_OK){
    uint8_t can_id = can_msg.can_id;
    uint8_t can_dlc = can_msg.can_dlc;
    switch(can_id){ //Segun el ID recibido se guarda en la EEPROM en la posicion de memoria que corresponde
      case VUV_MSG_ID:{
        EEPROM.write(VUV_addr, can_msg.data[0]); //Todos los mensajes CAN de configuracion son de 1 byte
        #ifdef SERIAL_DEBUG
        Serial.print("VUV:");
        Serial.println(can_msg.data[0],BIN);
        #endif
        break;}
      case VOV_MSG_ID:{
        EEPROM.write(VOV_addr, can_msg.data[0]); //Todos los mensajes CAN de configuracion son de 1 byte
        #ifdef SERIAL_DEBUG
        Serial.print("VOV:");
        Serial.println(can_msg.data[0],BIN);
        #endif
        break;}
      case DCTO_MSG_ID:{
        EEPROM.write(DCTO_addr, can_msg.data[0]); //Todos los mensajes CAN de configuracion son de 1 byte
        #ifdef SERIAL_DEBUG
        Serial.print("DCTO:");
        Serial.println(can_msg.data[0],BIN);
        #endif
        break;}
      case NCELL_MSG_ID:{
        EEPROM.write(NCELL_addr, can_msg.data[0]); //Todos los mensajes CAN de configuracion son de 1 byte
        #ifdef SERIAL_DEBUG
        Serial.print("NCELL:");
        Serial.println(can_msg.data[0],BIN);
        #endif
        break;}
      case N_NTC_MSG_ID:{
        EEPROM.write(N_NTC_addr, can_msg.data[0]); //Todos los mensajes CAN de configuracion son de 1 byte
        #ifdef SERIAL_DEBUG
        Serial.print("N_NTC:");
        Serial.println(can_msg.data[0],BIN);
        #endif
        break;}
      case TSLEEP_MSG_ID:{
        EEPROM.write(TSLEEP_addr, can_msg.data[0]); //Todos los mensajes CAN de configuracion son de 1 byte
        #ifdef SERIAL_DEBUG
        Serial.print("TSLEEP:");
        Serial.println(can_msg.data[0],BIN);
        #endif
        break;}
      case BAL1TO8_MSG_ID:{
        #ifdef SERIAL_DEBUG
        Serial.print("Balanceo celdas 1 a 8:");
        Serial.println(can_msg.data[0], BIN);
        #endif
        force_balancing (TOTAL_IC, tx_cfg, can_msg.data[0], true); //True indica que se balancee en el grupo 1 a 8
        break;}
      case BAL9TO12_MSG_ID:{
        #ifdef SERIAL_DEBUG
        Serial.print("Balanceo celdas 9 a 12: ");
        Serial.println(can_msg.data[0], BIN);
        #endif
        force_balancing (TOTAL_IC, tx_cfg, can_msg.data[0], false); //True indica que se balancee en el grupo 1 a 8
        break;}
      case ASK_CONFIG_MSG_ID:{ //Devuelve las configuraciones almacenadas en la EEPROM
        #ifdef SERIAL_DEBUG
        Serial.print("Parametros de config pedidos.");
        Serial.println(can_msg.data[0], BIN);
        #endif 
        can_msg.can_id = ANSWER_CONFIG_MSG_ID;
        can_msg.can_dlc = 8;
        can_msg.data[0] =   EEPROM.read(VUV_addr);
        can_msg.data[1] =   EEPROM.read(VOV_addr);
        can_msg.data[2] =   EEPROM.read(DCTO_addr);
        can_msg.data[3] =   EEPROM.read(NCELL_addr);
        can_msg.data[4] =   EEPROM.read(N_NTC_addr);
        can_msg.data[5] =   EEPROM.read(MAX_DIFF_CELL_addr);
        wakeup_sleep();
        uint8_t error = LTC6804_rdcfg(TOTAL_IC,rx_cfg); //Se lee la configuracion del LTC
        if (error != -1){
          can_msg.data[6] =   rx_cfg[0][4]; //En bytes 6 y 7 se envian celdas actualmente en balanceo
          can_msg.data[7] =   rx_cfg[0][5] & 0b00001111; //Podria usarse una funcion para obtener config
        }
        mcp2515.sendMessage(&can_msg); //Se envia el mensaje CAN con las configuraciones
        break;}
      case MAX_DIFF_CELL_MSG_ID:{
        EEPROM.write(MAX_DIFF_CELL_addr, can_msg.data[0]); //Todos los mensajes CAN de configuracion son de 1 byte
        MAX_VCELL_DIFF = float( can_msg.data[0] * 0.001); //Se guarda el valor de maxima_diferencia de tension
        #ifdef SERIAL_DEBUG
        Serial.print("Máx. diferencia mV entre celdas:");
        Serial.println(can_msg.data[0],BIN);
        #endif
        break;}
      default:
        break;
    }
  }
  read_eeprom_ltc (TOTAL_IC, tx_cfg); //Se actualiza el array de config del LTC leyendo los parametros de la EEPROM
  read_eeprom_atmega(UV_THR, OV_THR, N_NTC,TOTAL_CELL,UVBAT_THR, OVBAT_THR, MAX_VCELL_DIFF); //Se actualizan los valores de config del ATMega
  LTC6804_wrcfg(TOTAL_IC, tx_cfg); //Se actualiza la configuracion del LTC
  
  #ifdef SERIAL_DEBUG
    Serial.println("Configuracion del LTC:");
    print_config(tx_cfg);
  #endif
  interrupts();
}


/***********************************************************************
  Inicialización
 ***********************************************************************/
void setup(){
  for (uint8_t i=0; i<TOTAL_IC; i++){ //Se inicializa el array de los V de las celdas
    for(uint8_t j=0 ; j<12 ; j++){
      cell_codes[i][j] = 65525;
    }
  }
  #ifdef SERIAL_DEBUG
    Serial.begin(115200);
    print_config(tx_cfg);
  #endif
  Serial.begin(115200);

  
  LTC6804_initialize();  //Inicializa el LTC6804
  init_cfg(tx_cfg);      //Inicializa el array de configuración del 6804 a los valores por defecto
  
  read_eeprom_ltc(TOTAL_IC, tx_cfg); //Actualiza el array de config del LTC6804 con los valores almacenados en la EEPROM
  read_eeprom_atmega(UV_THR, OV_THR, N_NTC,TOTAL_CELL,UVBAT_THR, OVBAT_THR, MAX_VCELL_DIFF); //Se leen las configuraciones del ATMEGA desde EEPROM

  init_mcp2515(CAN_125KBPS, MCP_8MHZ, 0); //Se iniciliaza el MCP2515 para comunicacion CAN a 125 KBPS
  mcp2515.setConfigMode(); //Se configuran las mascaras del MCP2515 para aceptar solo mensajes con ID 0b000000xxxxxx
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0b11111000000);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0b11111000000);
  mcp2515.setNormalMode();
  
  attachInterrupt(0, can_msg_rcv, FALLING); //Se configura interrupcion al recibir mensaje CAN de configuracion

  //Configuración de los mensajes CAN
  config_can_msg (canBatMsg1, BAT_MSG1_ID, 8); //Se indica el mensaje CAN, el ID (entero decimal) y el DLC (entero decimal)
  config_can_msg (canBatMsg2, BAT_MSG2_ID, 8);
  config_can_msg (canBatMsg3, BAT_MSG3_ID, 8);
  config_can_msg (canTempMsg1, TEMP_MSG1_ID, 8);
  config_can_msg (canTempMsg2, TEMP_MSG2_ID, 8);
  config_can_msg (canTempMsg3, TEMP_MSG3_ID, 8);
  config_can_msg (canTempMsg4, TEMP_MSG4_ID, 8);
  config_can_msg (warning, WARNING_ID, 1);
  config_can_msg (canCurrentMsg, CURRENT_MSG_ID, 4);
}

/**********************************************************************
  Loop
***********************************************************************/

void loop() {

  static float T_max = 0.0;
  //Comienza la lectura y conversión de las tensiones leidas por el LTC6804
  read_cell_voltage (TOTAL_IC, TOTAL_CELL, tx_cfg, cell_codes);

  //Se convierten los valores de la tensión de las celdas a mensaje CAN
  cell_voltage_to_can_msg (cell_codes, TOTAL_IC, TOTAL_CELL, canBatMsg1, canBatMsg2, canBatMsg3);

  //Se envia el valor de las tensiones de celdas por CAN
  send_can_msg(canBatMsg1);

  if(TOTAL_CELL > 4){ //Si hay mas de 4 celdas se envia segundo y tercer mensaje CAN
    send_can_msg(canBatMsg2);
  }
  if(TOTAL_CELL >= 9){
    send_can_msg (canBatMsg3);
  }

  //Verificar y activar el balanceo de las celdas necesarias
  balancing(TOTAL_IC,  cell_codes, tx_cfg, MAX_VCELL_DIFF, TOTAL_CELL, OV_THR, UV_THR );
  
  //Calcular tensión total y generar aviso de tension o Temperatura
  float voltaje_total = calc_volt_total(cell_codes, TOTAL_IC, TOTAL_CELL);
  
  bool genera_warning = warning_msg(voltaje_total, UVBAT_THR, OVBAT_THR, T_max, warning);
  if (genera_warning) {
    //Se envia el aviso
    send_can_msg(warning);
  }

  //Se miden todas las temperaturas de las celdas y se guardan en cell_temp. Además se obtiene la T_max
  T_max = measure_all_temp(cell_temp, N_NTC, BETTA, To, Ro, Vcc, PIN_ANALOG_NTC);

  //Se conviertes las temperaturas a mensajes CAN
  temp_to_can_msg(cell_temp, N_NTC, canTempMsg1, canTempMsg2, canTempMsg3, canTempMsg4);

  //Se envian las Temperaturas de las celdas por CAN. Cada mensaje tiene 8 temperaturas
  send_can_msg(canTempMsg1); //Primeras 8 NTC
  if (N_NTC >8){ //NTC de 9 a 16
    send_can_msg(canTempMsg2);  
  }
  if (N_NTC > 16){ //NTC de 17 a 24
    send_can_msg(canTempMsg3);    
  }
  if (N_NTC >24){ //NTC de 24 a 32
    send_can_msg(canTempMsg4);  
  }

  //Se mide la corriente y se envía el valor por CAN
  int current = get_current (SAMPLESNUMBER, SENSIBILITY_CURRENT);
  current_to_can_msg (current, canCurrentMsg);
  send_can_msg(canCurrentMsg);

  
  //Estimar SOH y SOC y enviarlos por CAN

} //FIN DEL LOOP





/***********************************
 * Función para medir la corriente  en mA
 ************************************/
 int get_current (const int samples_number, const float sensibility_current){
  float current_sum = 0;
  for (int i=0; i<samples_number; i++){
    float voltage = analogRead(PIN_CURRENT_SENSOR) * 5.0 /1023.0;
    current_sum += (voltage -2.5)/sensibility_current; //V = 2.5 + K*I
  }
  int current = round((current_sum/samples_number)*1000);
  return(current); 
}


/************************************
 * Algortimo de balanceo
 ***********************************^*/
 void balancing(const uint8_t TOTAL_IC,  uint16_t cell_codes[][12], uint8_t tx_cfg[][6], float max_difference, uint8_t TOTAL_CELL, float VOV_THR, float VUV_THR ){
  float V_min=cell_codes[0][0];
  float V_max = cell_codes[0][0];
  static uint16_t cell_to_balance = 0b0000000000000000;
  
  //Se obtiene V_min y V_max
  for (int i=0; i< TOTAL_IC; i++){
    for(int j=0; j<TOTAL_CELL; j++){ 
      if(V_min < float(cell_codes[i][j]* 0.0001)){//Según datasheet el valor medido se multiplica por 100uV
        V_min = float(cell_codes[i][j]* 0.0001);
      }
      if(V_max > float(cell_codes[i][j]* 0.0001)){
        V_max = float(cell_codes[i][j]* 0.0001);
      }
    }
  }

  //Se busca que celdas balancear
  for (int i=0; i< TOTAL_IC; i++){//Se obtiene V_min y V_max
    for(int j=0; j<TOTAL_CELL; j++){
      float cell_volt = cell_codes[i][j]*0.0001; //Voltaje de la celda j en Voltios
       if(((cell_volt -V_min) > max_difference)||(cell_volt > VOV_THR)){ 
       //Si la celda presenta una diferencia con la de menor V superior a la máx. diferencia o supera la máxima tensión, se balancea.
        cell_to_balance = (cell_to_balance | (0b1 << j));
       }
       else{
        if ((cell_volt < (VOV_THR - 2*max_difference)) || (cell_volt < VUV_THR) || ((cell_volt- V_min) < max_difference)){ 
          //Si la celda ha bajado del umbral de sobrevoltaje o está por debajo de la tensión mínima o dentro del rango de máxima diferencia, se para el balanceo
          cell_to_balance = (cell_to_balance & ~(uint16_t(0b1 << j))); //Pongo a 0 el bit de la celda que no se debe balancear
        }
       }
    }
  }

  //Se balancean las celdas indicadas
   uint8_t cell_to_bal_A = cell_to_balance & 0b0000000011111111; //Se obtiene el byte de las celdas 1 a 8
   uint8_t cell_to_bal_B = ((cell_to_balance >> 8) & 0b0000000000001111); //Se obtiene el byte de las celdas 9 a 12
   force_balancing (TOTAL_IC, tx_cfg, cell_to_bal_A, true); //Se balancea celdas 1 a 8
   force_balancing (TOTAL_IC, tx_cfg, cell_to_bal_B, false); //Se balancea celdas 9 a 12
  
 }





/************************************
 * Forzar balanceo de las celdas indicadas
 ***********************************/
 void force_balancing (const uint8_t TOTAL_IC, uint8_t tx_cfg[][6], uint8_t cells_to_balance, bool group){ //Si group= true. Se balancea grupo celdas 1 a 8
  //Si group = false. Balancea grupo de celdas 9 a 12
  if( group){
    for (uint8_t i=0 ; i< TOTAL_IC; i++){
      tx_cfg[i][4] = cells_to_balance; //Se escriben los 8 bit de las celdas a balancear
    }
  }
  else{
    for (uint8_t i=0 ; i< TOTAL_IC; i++){
      tx_cfg[i][5] = ((tx_cfg[i][5])& 0b11110000) | (cells_to_balance & 0b00001111); //Se escribe los 4 bits de las celdas a balancer en los bit 0 a 3.
    }
  }
 }


/************************************
  Lectura de las configuraciones para el LTC6804 desde la EEPROM
  del mmicrocontrolador
  **********************************/
void read_eeprom_ltc (const uint8_t TOTAL_IC, uint8_t tx_cfg[][6]){
  //La funcion actualiza los valores del array de configuracion tx_cfg[][6]
  uint16_t VUV = EEPROM.read(VUV_addr)*(0.02)*(1/0.0016) -1;
  /*Se convierte el valor de umbral de bajo-voltaje desde la codificacion
  con la que se almacena en la EEPROM, VUV(real)=VUV_eeprom * 0.02 al valor
  que necesita el LTC (VUV_real=(VUV+1)*16*100uV)*/
  uint16_t VOV = EEPROM.read(VOV_addr)*(0.02)*(1/0.0016) -1; //Se procede igual con
  //el umbral de sobre-voltaje
  for (uint8_t i=0 ; i< TOTAL_IC; i++){
    tx_cfg[i][0] = EEPROM.read(CFGR0_addr);
    tx_cfg[i][1] =  VUV & (0b11111111);
    tx_cfg[i][2] = (((VUV>>8) & (0b00001111)) | ((VOV & 0b00001111)<< 4));
    tx_cfg[i][3] = ((VOV >> 4) & (0b11111111));
    tx_cfg[i][5] = ((tx_cfg[i][5] & 0b00001111) |(EEPROM.read(DCTO_addr)<< 4));
    //Se ponen a 0 los 4 bits del Discharge TimeOut y se carga el valor que se tiene
    //almacenado de Discharge TimeOut
  }
}

/************************************
  Lectura de las configuraciones del ATMega desde la EEPROM
  Umbrales de voltaje celda inferior y superior, num de NTC, numero de celdas
  y Umbrales de voltaje de la bateria inferior y superior
*************************************/
void read_eeprom_atmega(float &UV_THR, float &OV_THR, uint8_t &N_NTC,
    uint8_t &TOTAL_CELL, float &UVBAT_THR, float &OVBAT_THR, float &MAX_VCELL_DIFF){
  TOTAL_CELL = uint8_t(EEPROM.read(NCELL_addr));
  UV_THR = float(EEPROM.read(VUV_addr))*0.02; //Se obtiene el valor umbral mínimo tension de la celda
  OV_THR = float(EEPROM.read(VOV_addr))*0.02; //Se obtiene el valor maximo de V de la celda
  N_NTC = uint8_t(EEPROM.read(N_NTC_addr));
  UVBAT_THR= UV_THR * TOTAL_CELL; //voltaje minimo de toda la bateria
  OVBAT_THR = OV_THR * TOTAL_CELL; //voltaje maximo de toda la bateria
  MAX_VCELL_DIFF = float(EEPROM.read(MAX_DIFF_CELL_addr)*0.001);
}


/************************************
  Se comienza la conversión ADC de las tensiones
  de las celdas
 **************************************/
void start_cell_voltage_ADC(const uint8_t TOTAL_IC, uint8_t tx_cfg[][6]) {
#ifdef SERIAL_DEBUG
  print_config(tx_cfg);
  Serial.println("Iniciando conv. ADC de Vcell");
#endif
  wakeup_sleep(); //Despierta el IC del modo sleep
  LTC6804_wrcfg(TOTAL_IC, tx_cfg); //configura el IC
  LTC6804_adcv(); //comienza la conversión ADC de las entradas de voltaje (Cx) del IC
  delay(3);
}

/************************************
  Se leen las tensiones de las celdas
 **************************************/
void read_cell_voltage (const uint8_t TOTAL_IC, const uint8_t TOTAL_CELL, uint8_t tx_cfg[][6], uint16_t cell_codes[][12]) {
  start_cell_voltage_ADC(TOTAL_IC, tx_cfg); //Realiza la conversión ADC de las tensiones
  uint16_t cell_codes_aux[TOTAL_IC][12]; //Se crea un array auxiliar
  for (int i = 0; i < TOTAL_IC; i++) { //Se almacena la lectura anterior en el array auxiliar
    for (int j = 0; j < TOTAL_CELL; j++) {
      cell_codes_aux[i][j] = cell_codes[i][j];
    }
  }
  int8_t error; //Se crea la variable que almacena el estado de error del CRC
  error = LTC6804_rdcv(0, TOTAL_IC, cell_codes); // Se lee y hace un parses de las tensiones de las celdas
  if (error == -1) { //Si el error vale -1 es que hubo un error con el checkeo del CRC
#ifdef SERIAL_DEBUG
    Serial.println("Error en CRC Vcell");
#endif
    for (int i = 0; i < TOTAL_IC; i++) { //Si los valores recibidos no son correctos, se recuperan los valores de la lectura anterior
      for (int j = 0; j < TOTAL_CELL; j++) {
        cell_codes[i][j] = cell_codes_aux[i][j];
      }
    }
  }
  else { //Si no hay error con el CRC se procede a mostrar los valores medidos
#ifdef SERIAL_DEBUG
    print_cells(TOTAL_IC, cell_codes); //Se muestran los valores de tensión medidos
#endif
  }
}

/************************************
  Se configura el mensaje CAN
 **************************************/
void config_can_msg (struct can_frame &can_msg, const uint8_t can_id, const uint8_t can_dlc) {
  can_msg.can_id = can_id;
  can_msg.can_dlc = can_dlc;
}


/************************************
 * Función que codifica la corriente en un mensaje CAN. Se dividen los 32 bits del float en 4 bytes del mensaje CAN
 ****************************************/
void current_to_can_msg ( int current, struct can_frame &canCurrentMsg){
  canCurrentMsg.data[0] = (current) ; //Los 8 bits menos significados son el primer byte
  canCurrentMsg.data[1] = (current >> 8) ; //Los siguientes 8 bits del numero de 32 bits son el byte1
  canCurrentMsg.data[2] = (current >> 16);
  canCurrentMsg.data[3] = (current >> 24);
}

 
/************************************
  Se codifican las tensiones en mensajes CAN. Función
  Sigue este formato

  |  can_frame[7]      |    can_frame[6]     |    .....     |     can_frame[1]   |      can_frame[0]   |
  |--------------------|---------------------|--------------|--------------------|---------------------|
  | cell3 MSB          |     cell3 LSB       |    .....     |    cell0 MSB       |     cell0 LSB       |
  | cell_code[0][3] MSB|  cell_code[0][3] LSB|    .....     | cell_code[0][0] MSB|  cell_code[0][0] LSB|

 **************************************/
void cell_voltage_to_can_msg ( uint16_t cell_codes[][12], const uint8_t TOTAL_IC, const uint8_t TOTAL_CELL, struct can_frame &canBatMsg1) {
  for (uint8_t i = 0; i < TOTAL_IC; i++) {
    for (uint8_t j = 0 ; j < TOTAL_CELL; j++) { //Cada medición ocupa 2 bytes (cell_codes son arrays de 16 bits)
      //Este algoritmo permite rellenar un mensaje de 8 bytes que contenga los 4 valores de 16 bits de las celdas. Con MSB los valores de la celda más alta
      //y LSB los valores LSB de la celda más baja
      canBatMsg1.data[2 * j] = cell_codes[i][j]; //Se toma la parte baja del registro de 16 bits
      canBatMsg1.data[2 * j + 1] = cell_codes[i][j] >> 8; //Se toma la parte alta del registro de 16 bits y se guarda en el byte siguiente del mnsje
    }
  }
}

void cell_voltage_to_can_msg (const uint16_t cell_codes[][12], const uint8_t TOTAL_IC, const uint8_t TOTAL_CELL, struct can_frame &canBatMsg1,
                              struct can_frame &canBatMsg2) {
  for (uint8_t i = 0; i < TOTAL_IC; i++) {
    //Las primeras 4 celdas en el Mensaje 1 (canBatMsg1)
    for (uint8_t j = 0 ; j < 4; j++) { //Cada medición ocupa 2 bytes (cell_codes son arrays de 16 bits)
      //Este algoritmo permite rellenar un mensaje de 8 bytes que contenga los 4 valores de 16 bits de las celdas. Con MSB los valores de la celda más alta
      //y LSB los valores LSB de la celda más baja
      canBatMsg1.data[2 * j] = cell_codes[i][j]; //Se toma la parte baja del registro de 16 bits
      canBatMsg1.data[2 * j + 1] = cell_codes[i][j] >> 8; //Se toma la parte alta del registro de 16 bits y se guarda en el byte siguiente del mnsje
    }
  }
  for (uint8_t i = 0; i < TOTAL_IC; i++) {
    //Las celdas 4-7 en el Mensaje 2 (canBatMsg2)
    for (uint8_t j = 4 ; j < 8; j++) { //Cada medición ocupa 2 bytes (cell_codes son arrays de 16 bits)
      //Este algoritmo permite rellenar un mensaje de 8 bytes que contenga los 4 valores de 16 bits de las celdas. Con MSB los valores de la celda más alta
      //y LSB los valores LSB de la celda más baja
      canBatMsg2.data[2*(j - 4)] = cell_codes[i][j]; //Se toma la parte baja del registro de 16 bits
      canBatMsg2.data[2*(j - 4) + 1] = cell_codes[i][j] >> 8; //Se toma la parte alta del registro de 16 bits y se guarda en el byte siguiente del mnsje
    }
  }
}

void cell_voltage_to_can_msg (const uint16_t cell_codes[][12], const uint8_t TOTAL_IC, const uint8_t TOTAL_CELL, struct can_frame &canBatMsg1,
                              struct can_frame &canBatMsg2, struct can_frame &canBatMsg3 ) {
  for (uint8_t i = 0; i < TOTAL_IC; i++) {
    //Las primeras 4 celdas en el Mensaje 1 (canBatMsg1)
    for (uint8_t j = 0 ; j < 4; j++) { //Cada medición ocupa 2 bytes (cell_codes son arrays de 16 bits)
      //Este algoritmo permite rellenar un mensaje de 8 bytes que contenga los 4 valores de 16 bits de las celdas. Con MSB los valores de la celda más alta
      //y LSB los valores LSB de la celda más baja
      canBatMsg1.data[2 * j] = cell_codes[i][j]; //Se toma la parte baja del registro de 16 bits
      canBatMsg1.data[(2 * j) + 1] = cell_codes[i][j] >> 8; //Se toma la parte alta del registro de 16 bits y se guarda en el byte siguiente del mnsje
    }
  }
  for (uint8_t i = 0; i < TOTAL_IC; i++) {
    //Las celdas 4-7 en el Mensaje 2 (canBatMsg2)
    for (uint8_t j = 4 ; j < 8; j++) { //Cada medición ocupa 2 bytes (cell_codes son arrays de 16 bits)
      //Este algoritmo permite rellenar un mensaje de 8 bytes que contenga los 4 valores de 16 bits de las celdas. Con MSB los valores de la celda más alta
      //y LSB los valores LSB de la celda más baja
      canBatMsg2.data[2*(j - 4)] = cell_codes[i][j]; //Se toma la parte baja del registro de 16 bits
      canBatMsg2.data[2*(j - 4) + 1] = cell_codes[i][j] >> 8; //Se toma la parte alta del registro de 16 bits y se guarda en el byte siguiente del mnsje  
    }
  }
  for (uint8_t i = 0; i < TOTAL_IC; i++) {
    //Las celdas 4-7 en el Mensaje 2 (canBatMsg2)
    for (uint8_t j = 8 ; j < 12; j++) { //Cada medición ocupa 2 bytes (cell_codes son arrays de 16 bits)
      //Este algoritmo permite rellenar un mensaje de 8 bytes que contenga los 4 valores de 16 bits de las celdas. Con MSB los valores de la celda más alta
      //y LSB los valores LSB de la celda más baja
      canBatMsg3.data[2 * (j - 8)] = cell_codes[i][j]; //Se toma la parte baja del registro de 16 bits
      canBatMsg3.data[2 * (j - 8) + 1] = cell_codes[i][j] >> 8; //Se toma la parte alta del registro de 16 bits y se guarda en el byte siguiente del mnsje
    }
  }
}

/************************************
  Se selecciona el canal del MUX SPI
 **************************************/
/*
  Estructura del mensaje del SPI del MUX ADG731:

  |  DB7 (MSB)      | DB6             |  DB5            |       DB4       |      DB3        |      DB2        |       DB1       |  DB0 (LSB)    |
  |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|
  |   NOT(EN)       |     NOT(CS)     |       x         |      A4         |       A3        |     A2          |       A1        |      A0       |
  ENABLE  CS deben valer 0 durante la selección según la tabla de verdad
*/
void select_channel_MUX(const uint8_t channel) {
#ifdef SERIAL_DEBUG
  Serial.print("Canal selecc.: ");
  Serial.println(channel);
#endif
  if (channel <= 32) {
    uint8_t msg = (0b11000000 | channel); //Se genera el mensaje con el valor de 1 en Enable y CSA y se concatena el canal a elegir
#ifdef SERIAL_DEBUG
    Serial.print("Mensaje SPI es: ");
    Serial.println(msg, BIN); //Se imprime el mensaje a enviar por SPI en binario
#endif
    SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE1)); //HAY QUE REVISAR EL MODO DEL SPI PROBÁNDOLO
    digitalWrite(PIN_SYNC_MUX, LOW); //Se pone a baja el CS del ADG731
    SPI.transfer(msg); //Se envia el mensaje por SPI
    digitalWrite(PIN_SYNC_MUX, HIGH);
    SPI.endTransaction();
  }
}

/************************************
  Se leen las temperaturas
 **************************************/
float measure_temp(const int BETTA, const int To, const int Ro, const int Vcc, const uint8_t channel, const uint8_t AnalogInput) { //
  select_channel_MUX(channel); //Se elige la entrada a medir llamando a la función anterior
  float rntc = Ro * (Vcc / (analogRead(AnalogInput) * 5.0 / 1023.0) - 1); //Vdivisor=R_o/(R_o+R_NTC)
  float t_medida = BETTA / (log(rntc / Ro) + (BETTA / To));
  //t_medida = 28.3; //ESTA LINEA ESTA PARA PRUEBA **********************************************************************************************************************************************************************************
  #ifdef SERIAL_DEBUG
    Serial.println("Channel  | Tmedida ºC");
    Serial.print(channel);
    Serial.print("  | ");
    Serial.println(t_medida);
  #endif
  return t_medida;
}


/************************************
  Se leen las temperaturas y se almacenan en un array pasado por referencia
 **************************************/
float measure_all_temp(float array_temp[], const uint8_t N_NTC, const int BETTA, const int To, const int Ro, const int Vcc, const uint8_t AnalogInput) {
  float T_max = 0;
  for (uint8_t i = 0; i < N_NTC ; i++) {
    array_temp[i] = measure_temp(BETTA, To, Ro, Vcc, i,  AnalogInput);
    if (array_temp[i] > T_max) {
      T_max = array_temp[i];
    }
  }
  return T_max;
}



/************************************
  Se calcula la tensión total de la batería
 **************************************/
float calc_volt_total(const uint16_t cell_codes[][12], const uint8_t TOTAL_IC, const uint8_t TOTAL_CELL) {
  float voltage_total = 0;
  for (uint8_t i = 0; i < TOTAL_IC ; i++) {
    for (uint8_t j = 0 ; j < TOTAL_CELL; j++) {
      voltage_total += (cell_codes[i][j] * 0.0001); //Se pasa la tensión a decimal y se suman
    }
  }
  return voltage_total;
}


/************************************
  Se verifica si generar un aviso de tensión baja de batería
 **************************************/
bool warning_msg(const float voltaje_total, const float umbral_vacio, const float umbral_cargado, const float T_max, struct can_frame warning) {
  static uint8_t estado_V = 1; //Se parte en el caso 1, de batería no llena pero no descargada
  static uint8_t estado_ant_V = 1;
  static uint8_t estado_T = 4; //Se parte en el caso 1, de batería no llena pero no descargada
  static uint8_t estado_ant_T = 4;

  //Caso de voltaje muy bajo
  if (voltaje_total < (umbral_vacio - 0.1)) { //Se restan 0.1 V para dejar una histéresis
    warning.data[0] = warning.data[0] | 0b00000001; //Bit 0 a 1 almacena info de bajo voltaje. A 0 no hay bajo V
    warning.data[0] = warning.data[0] & 0b11111101; //pone bit 1 a 0 para indicar batería no cargada. Valor dle bit 1 a 1 indica batería cargada
    estado_ant_V = estado_V;
    estado_V = 0;
#ifdef SERIAL_DEBUG
    Serial.println("Vpack muy bajo");
#endif
  }
  //Caso voltaje entre mínimo y máximo
  else if ((voltaje_total > (umbral_vacio + 0.1)) && voltaje_total < umbral_cargado) {
    warning.data[0] = warning.data[0] & 0b11111110; //Bit 0 a 0 para indicar que no hay voltaje demasiado bajo
    warning.data[0] = warning.data[0] & 0b11111101 ; //Bit 1 a 0 indica batería NO está cargada totalmente
    estado_ant_V = estado_V;
    estado_V = 1;
#ifdef SERIAL_DEBUG
    Serial.println("Vpack OK");
#endif
  }
  //Caso de batería llena
  else if (voltaje_total >= umbral_cargado) {
    warning.data[0] = warning.data[0] & 0b11111110; //Poner bit 1 a 0 para indicar batería NO tiene bajo voltaje
    warning.data[1] = warning.data[0] | 0b00000010; //Batería sobre-cargada. Poner bit 1 a valor 1 para indicar batería por encima del V máximo
    estado_ant_V = estado_V;
    estado_V = 2;
#ifdef SERIAL_DEBUG
    Serial.println("Vpack superior al máximo");
#endif
  }


  if (T_max > 60) { //si supera 60 se avisa de sobretemperatura
    warning.data[0] = warning.data[0] | 0b00000100;  //Bit 2 a 1 indica Temperatura máxima excedida
    estado_ant_T = estado_T;
    estado_T = 3;
#ifdef SERIAL_DEBUG
    Serial.println("Temp. máx. excedida");
#endif
  }
  else if (T_max < 50) { //Hasta no bajar a 50ºC no se quita aviso de sobretemperatura
    warning.data[0] = warning.data[0] & 0b11111011; //se pone a 0 el bit 2 al bajar la Tª nuevamente por debajo de 50ºC
    estado_ant_T = estado_T;
    estado_T = 4;
#ifdef SERIAL_DEBUG
    Serial.println("Temp. adecuada");
#endif
  }


  if ((estado_ant_V != estado_V) || (estado_ant_T != estado_T)) { //Si hay algún cambio de estado
#ifdef SERIAL_DEBUG
    Serial.println("Cambios en las advertencias. Activando Warning");
    Serial.println("Estado anterior(V) | Estado actual(V)");
    Serial.print(estado_ant_V);
    Serial.print("     |   ");
    Serial.println(estado_V);
    Serial.println("Estado anterior(T) | Estado actual(T)");
    Serial.print(estado_ant_T);
    Serial.print("        |   ");
    Serial.println(estado_T);
#endif
    return true; //Devuelve true si hay un cambio de estado para así enviar un nuevo mensaje warning
  }
#ifdef SERIAL_DEBUG
  Serial.println("No hay cambio en advertencias. No warning");
#endif
  return false; //Devuelve false si no hay cambio para así no enviar un nuevo warning
}

/************************************
  Se codifican las temperaturas en mensajes CAN
 **************************************/
void temp_to_can_msg(float array_temp[], const uint8_t N_NTC, struct can_frame &T_msg1, struct can_frame &T_msg2,
                     struct can_frame &T_msg3, struct can_frame &T_msg4) { //
  //Se estructuran siendo T[0] la situada en el LSByte y T[7] la situada en el MSByte del mensaje CAN
  for (uint8_t i = 0 ; i < 8 ; i++) { //8 primeras Temperaturas en el mensaje 1
    T_msg1.data[i] = uint8_t((array_temp[i] / 0.3) + 5);
  }
  for (uint8_t i = 0 ; i < 8 ; i++) { //Siguientes 8 temperaturas
    T_msg2.data[i] = uint8_t((array_temp[i+8] / 0.3) + 5);
  }
  for (uint8_t i = 0 ; i < 8 ; i++) { //8 temperaturas siguientes
    T_msg3.data[i] = uint8_t((array_temp[i+16] / 0.3) + 5);
  }
  for (uint8_t i = 0 ; i < 8 ; i++) {
    T_msg4.data[i] = uint8_t((array_temp[i+24] / 0.3) + 5);
  }
}

/************************************
  Se calcula el SOC del pack de baterías
 **************************************/
void calculate_SOC() { //

}

/************************************
  Se calcula el SOH del pack de baterías
 **************************************/
void calculate_SOH() { //

}

/************************************
  codifica los SOH y SOC así como el estado
  carga/descarga en mensajes CAN
 **************************************/
void SOx_can_msg() { //

}


/************************************
  Envia el mensaje CAN deseado
 **************************************/
void send_can_msg(const struct can_frame &can_msg) { //
  #ifdef SERIAL_DEBUG
  //Serial.println("Mensaje CAN enviado:");
  //Serial.print(can_msg.can_id, HEX); // print ID
  //Serial.print(" ");
  //Serial.print(can_msg.can_dlc, HEX); // print DLC
  //Serial.print(" ");
  //for (int i = 0; i < can_msg.can_dlc; i++)  { // print the data
  //  Serial.print(can_msg.data[i], HEX);
  //  Serial.print(" ");
  //}
  //Serial.println();
  #endif
  mcp2515.sendMessage(&can_msg);
  delay(50);
}


/************************************
  Inicializar módulo MCP2515
  0 para NormalMode
  1 para LoopbackMode
  2 para ListenOnlyMode
  Ejemplo de llamada init_mcp2515(CAN_125KBPS, MCP8MHZ, 0);
 **************************************/
void init_mcp2515(const CAN_SPEED canSpeed, CAN_CLOCK canClock, int mode) {
  mcp2515.reset();
  mcp2515.setBitrate(canSpeed, canClock);
  if (mode == 0) {
    mcp2515.setNormalMode();
  }
  else if (mode == 1) {
    mcp2515.setLoopbackMode();
  }
  else {
    mcp2515.setListenOnlyMode();
  }
}


/************************************
  Se inicializa el paquete de datos a los valores de configuración
 **************************************/
void init_cfg(uint8_t tx_cfg[][6]) {
  for (int i = 0; i < TOTAL_IC; i++) {
    tx_cfg[i][0] = 0xFE;
    tx_cfg[i][1] = 0x00 ;
    tx_cfg[i][2] = 0x00 ;
    tx_cfg[i][3] = 0x00 ;
    tx_cfg[i][4] = 0x00 ;
    tx_cfg[i][5] = 0x00 ;
  }
}


/*************************************************************
  Imprime los voltajes de las Celdas Por puerto Serial
 *************************************************************/
void print_cells(const uint8_t TOTAL_IC, uint16_t cell_codes[][12]) {
  //void print_cells(const uint8_t TOTAL_IC, uint16_t *cell_codes){
  Serial.println("Voltaje de las Celdas");
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    for (int cell = 0; cell < 12; cell++) {
      Serial.print(" C");
      Serial.print(cell + 1, DEC);
      Serial.print(":");
      Serial.print(cell_codes[current_ic][cell] * 0.0001, 4);
      Serial.print(",");
    }
    Serial.println();
  }
  Serial.println();
}

/*****************************************************************************
  Imprime por puerto Serial los Voltajes de los GPIO leidos y el valor de Vref2
 *****************************************************************************/
void print_aux(uint16_t aux_codes[][6]) {
  //void print_aux(uint16_t *aux_codes){

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 5; i++) {
      Serial.print(" GPIO-");
      Serial.print(i + 1, DEC);
      Serial.print(":");
      Serial.print(aux_codes[current_ic][i] * 0.0001, 4);
      Serial.print(",");
    }
    Serial.print(" Vref2");
    Serial.print(":");
    Serial.print(aux_codes[current_ic][5] * 0.0001, 4);
    Serial.println();
  }
  Serial.println();
}

/*******************************************************************************
  Imprime por Puerto Serial la Configuración que va a escribirse en el LTC6804
 ********************************************************************************/
void print_config(uint8_t tx_cfg[][6]) {
  int cfg_pec;

  Serial.println("Configuración: ");
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    Serial.print(": ");
    Serial.print("0x");
    serial_print_hex(tx_cfg[current_ic][0]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][1]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][2]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][3]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][4]);
    Serial.print(", 0x");
    serial_print_hex(tx_cfg[current_ic][5]);
    Serial.print(", CRC : 0x");
    cfg_pec = pec15_calc(6, &tx_cfg[current_ic][0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(", 0x");
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}

/******************************************************************
  Imprime por Puerto Serial la configuración que se ha leido del LTC6804
 *******************************************************************/
void print_rxconfig(uint8_t rx_cfg[][8]) {
  Serial.println("Configuración recibida: ");
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(" IC ");
    Serial.print(current_ic + 1, DEC);
    Serial.print(": 0x");
    serial_print_hex(rx_cfg[current_ic][0]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][1]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][2]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][3]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][4]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][5]);
    Serial.print(", CRC recibido: 0x");
    serial_print_hex(rx_cfg[current_ic][6]);
    Serial.print(", 0x");
    serial_print_hex(rx_cfg[current_ic][7]);
    Serial.println();
  }
  Serial.println();
}

/******************************************************************
  Imprime por Puerto Serial en Hexadecimal
 *******************************************************************/
void serial_print_hex(uint8_t data) {
  if (data < 16) {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  }
  else
    Serial.print((byte)data, HEX);
}
