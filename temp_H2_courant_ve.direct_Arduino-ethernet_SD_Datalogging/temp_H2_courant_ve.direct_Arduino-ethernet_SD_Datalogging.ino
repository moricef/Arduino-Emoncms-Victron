#include <stdlib.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h>
#include <OneWire.h>
#include <Wire.h>
#include "EmonLib.h"
#include "SdFat.h"
#include <RTClib.h>

#include "config.h"
RTC_DS1307 rtc;

/***************************************************************************/
/* Lecture de température sur un unique capteur DS18B20 sur un bus 1-Wire. */

/* Broche du bus 1-Wire */
const byte BROCHE_ONEWIRE = 2;

/* Code de retour de la fonction getTemperature() */
enum DS18B20_RCODES {
  READ_OK,  // Lecture ok
  NO_SENSOR_FOUND,  // Pas de capteur
  INVALID_ADDRESS,  // Adresse reçue invalide
  INVALID_SENSOR  // Capteur invalide (pas un DS18B20)
};


/* Création de l'objet OneWire pour manipuler le bus 1-Wire */
OneWire ds(BROCHE_ONEWIRE);


/**
   Fonction de lecture de la température via un capteur DS18B20.
*/
byte getTemperature(float *temperature, byte reset_search)
{
  byte data[9], addr[8];
  // data[] : Données lues depuis le scratchpad
  // addr[] : Adresse du module 1-Wire détecté

  /* Reset le bus 1-Wire ci nécessaire (requis pour la lecture du premier capteur) */
  if (reset_search)
  {
    ds.reset_search();
  }

  /* Recherche le prochain capteur 1-Wire disponible */
  if (!ds.search(addr))
  {
    // Pas de capteur
    return NO_SENSOR_FOUND;
  }

  /* Vérifie que l'adresse a été correctement reçue */
  if (OneWire::crc8(addr, 7) != addr[7])
  {
    // Adresse invalide
    return INVALID_ADDRESS;
  }

  /* Vérifie qu'il s'agit bien d'un DS18B20 */
  if (addr[0] != 0x28)
  {
    // Mauvais type de capteur
    return INVALID_SENSOR;
  }

  /* Reset le bus 1-Wire et sélectionne le capteur */
  ds.reset();
  ds.select(addr);

  /* Lance une prise de mesure de température et attend la fin de la mesure  (Lecture toutes les 5 secondes)*/
  ds.write(0x44, 1);
  delay(800);

  /* Reset le bus 1-Wire, sélectionne le capteur et envoie une demande de lecture du scratchpad */
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  /* Lecture du scratchpad */
  for (byte i = 0; i < 9; i++)
  {
    data[i] = ds.read();
  }

  /* Calcul de la température en degré Celsius */
  *temperature = ((data[1] << 8) | data[0]) * 0.0625;

  // Pas d'erreur
  return READ_OK;
}

/*******************Demo for MQ-8 Gas Sensor Module V1.0*****************************
  Contact: support[at]sandboxelectronics.com

  Lisence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)

  Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application.

                                                    Sandbox Electronics    2014-02-03
************************************************************************************/

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (3)     //define which analog input channel you are going to use
#define         RL_VALUE                     (10)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.21)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
//normal operation

/**********************Application Related Macros**********************************/
#define         GAS_H2                      (0)

/*****************************Globals***********************************************/
float           H2Curve[3]  =  {2.3, 0.93, -1.44};   //two points are taken from the curve in datasheet.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, lg8.5), point2: (lg10000, lg0.03)

float           Ro           =  10;                  //Ro is initialized to 10 kilo ohms

/****************************************/
/****************** MQResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
  Input:   mq_pin - analog channel
  Output:  Ro of the sensor
  Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}
/*****************************  MQRead *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
  Output:  ppm of the target gas
  Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_H2) {
    return MQGetPercentage(rs_ro_ratio, H2Curve);
  }
  return 0;
}

/*****************************  MQGetPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}

/************************************************************************************
************************************************************************************/

/* La ventilation se déclenche à 27°C ou à H2 = 41000 ppm*/
float tempMax = 27.0;
float valH2max = 41000;
//float valH2max = 29.00;


/*******************************************************/
//Arduino Misuratore di potenzza e corrente elettrica con SCT-013-030

EnergyMonitor emon1, emon2;  // Create instances

// Sensor pins
const int currentSensor1Pin = A0;
const int currentSensor2Pin = A1;

//Calibrations
const int volt = 230; // France 230V
const float ct_calibration = 29; //Corrente Const= Ratio/Res. Burder. 1800/62 = 29
float Irms1 = 0;
float Irms2 = 0;
/********************************************************/
float temperature;
/********************************************************/

/************************ Tableaux des données Victron************************/
//MPPT 150/35
char receivedChars1[buffsize];                       // an array to store the received data
char tempChars1[buffsize];                           // an array to manipulate the received data
char recv_label1[num_keywords1][label_bytes]  = {0};  // {0} tells the compiler to initalize it with 0.
char recv_value1[num_keywords1][value_bytes]  = {0};  // That does not mean it is filled with 0's
char value1[num_keywords1][value_bytes]       = {0};  // The array that holds the verified data

static byte blockindex1 = 0;
bool new_data1 = false;
bool blockend1 = false;

//PHOENIX INVERTER 24V 500VA
char receivedChars2[buffsize];                       // an array to store the received data
char tempChars2[buffsize];                           // an array to manipulate the received data
char recv_label2[num_keywords2][label_bytes]  = {0};  // {0} tells the compiler to initalize it with 0.
char recv_value2[num_keywords2][value_bytes]  = {0};  // That does not mean it is filled with 0's
char value2[num_keywords2][value_bytes]       = {0};  // The array that holds the verified data

static byte blockindex2 = 0;
bool new_data2 = false;
bool blockend2 = false;

//BMV700
char receivedChars3[buffsize];                       // an array to store the received data
char tempChars3[buffsize];                           // an array to manipulate the received data
char recv_label3[num_keywords3][label_bytes]  = {0};  // {0} tells the compiler to initalize it with 0.
char recv_value3[num_keywords3][value_bytes]  = {0};  // That does not mean it is filled with 0's
char value3[num_keywords3][value_bytes]       = {0};  // The array that holds the verified data

static byte blockindex3 = 0;
bool new_data3 = false;
bool blockend3 = false;

/********************* SdFat  *******************/
const uint8_t chipSelect = 4;
SdFat sd;

#define FILE_BASE_NAME "Data"

// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))

/******************* Ethernet *******************/
//Ethernet wired
static byte mac[] = {0x90, 0xA2, 0xDA, 0x00, 0x69, 0xD5};

static IPAddress ip(192, 168, 5, 53);
static IPAddress subnet(255, 255, 255, 0);
static IPAddress dns(88, 202, 50, 3);
static IPAddress gw(192, 168, 5, 1);

EthernetClient client;

//Emoncms configurations
//char server[] = "genatfab.ovh";     // name address for genatfab.ovh
//IPAddress server(87.98.154.146);  // numeric IP for genatfab.ovh (no DNS)
char server[] = "192.168.5.51";

//String apikey = "cd5992c7fc54b2802ba38a4dbfc46ef9";  //genatfab.ovh api key
String apikey = "8bbd25e3b62b4245fea52c4534637ea8";  //localhost api key
int node = 0; //if 0, not used

unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds
boolean lastConnected = false;                 // state of the connection last time through the main loop
const unsigned long postingInterval = 10 * 1000; // delay between updates, in milliseconds

/**************************** Write,read, send and erase data on SDcard ********************************/
//=============================================================
void dateTime(uint16_t* date, uint16_t* time) {

  DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

//============================================================

void writeSD() {

  File logFile;
  char fileName[13] = FILE_BASE_NAME "0000";
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

  if (BASE_NAME_SIZE > 8) {
    error("FILE_BASE_NAME too long");
  }

  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 3] != '9') {
      fileName[BASE_NAME_SIZE + 3]++;
    } else if (fileName[BASE_NAME_SIZE + 2] != '9') {
      fileName[BASE_NAME_SIZE + 3] = '0';
      fileName[BASE_NAME_SIZE + 2]++;
    } else if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 3] = '0';
      fileName[BASE_NAME_SIZE + 2] = '0';
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 3] = '0';
      fileName[BASE_NAME_SIZE + 2] = '0';
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      error("Can't create file name");
    }
  }

  DateTime now = rtc.now();
  unsigned long currentTime = now.unixtime();

  SdFile::dateTimeCallback(dateTime);
  if (!logFile.open(fileName, O_CREAT | O_WRITE | O_EXCL )) {
    error("write_logFile.open");
  }
  logFile.print("GET /emoncms/input/bulk?data=[[");
  logFile.print("34,0,{\"temp\":");                                      // Node 0 : température
  logFile.print(temperature);
  logFile.print("}],[");
  logFile.print("34,1,{\"H2\":");                                        // Node 1 : MQ-8
  logFile.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_H2));
  logFile.print("}],[");
  logFile.print("34,2,{\"power1\":");                                    // Node 2 : CT-1
  logFile.print(Irms1 * volt);
  logFile.print("}],[");
  logFile.print("34,3,{\"power2\":");                                    // Node 3 : CT-2
  logFile.print(Irms2 * volt);
  logFile.print("}],[");
  logFile.print("34,4,{\"V\":");                                         // Node 4 : MPPT 150/35
  logFile.print(value1[3]);
  logFile.print("},{\"I\":");
  logFile.print(value1[4]);
  logFile.print("},{\"VPV\":");
  logFile.print(value1[5]);
  logFile.print("},{\"PPV\":");
  logFile.print(value1[6]);
  logFile.print("},{\"CS\":");
  logFile.print(value1[7]);
  logFile.print("},{\"ERR\":");
  logFile.print(value1[8]);
  logFile.print("},{\"H19\":");
  logFile.print(value1[10]);
  logFile.print("},{\"H20\":");
  logFile.print(value1[11]);
  logFile.print("},{\"H21\":");
  logFile.print(value1[12]);
  logFile.print("},{\"H22\":");
  logFile.print(value1[13]);
  logFile.print("},{\"H23\":");
  logFile.print(value1[14]);
  logFile.print("},{\"HSDS\":");
  logFile.print(value1[15]);
  logFile.print("}],[");
  logFile.print("34,5,{\"MODE\":");          // Node 5 : Inverter_24V_500VA
  logFile.print(value2[3]);
  logFile.print("},{\"CS\":");
  logFile.print(value2[4]);
  logFile.print("},{\"AC_OUT_V\":");
  logFile.print(value2[5]);
  logFile.print("},{\"AC_OUT_I\":");
  logFile.print(value2[6]);
  logFile.print("},{\"V\":");
  logFile.print(value2[7]);
  logFile.print("},{\"AR\":");
  logFile.print(value2[8]);
  logFile.print("},{\"WARN\":");
  logFile.print(value2[9]);
  logFile.print("}],[");
  logFile.print("34,6,{\"V\":");           // Node 6 : BMV700
  logFile.print(value3[1]);
  logFile.print("},{\"I\":");
  logFile.print(value3[2]);
  logFile.print("},{\"P\":");
  logFile.print(value3[3]);
  logFile.print("},{\"CE\":");
  logFile.print(value3[4]);
  logFile.print("},{\"SOC\":");
  logFile.print(value3[5]);
  logFile.print("},{\"TTG\":");
  logFile.print(value3[6]);
  logFile.print("},{\"AR\":");
  logFile.print(value3[9]);
  logFile.print("},{\"H1\":");
  logFile.print(value3[13]);
  logFile.print("},{\"H2\":");
  logFile.print(value3[14]);
  logFile.print("},{\"H3\":");
  logFile.print(value3[15]);
  logFile.print("},{\"H4\":");
  logFile.print(value3[16]);
  logFile.print("},{\"H5\":");
  logFile.print(value3[17]);
  logFile.print("},{\"H6\":");
  logFile.print(value3[18]);
  logFile.print("},{\"H7\":");
  logFile.print(value3[19]);
  logFile.print("},{\"H8\":");
  logFile.print(value3[20]);
  logFile.print("},{\"H9\":");
  logFile.print(value3[21]);
  logFile.print("},{\"H10\":");
  logFile.print(value3[22]);
  logFile.print("},{\"H11\":");
  logFile.print(value3[23]);
  logFile.print("},{\"H12\":");
  logFile.print(value3[24]);
  logFile.print("},{\"H17\":");
  logFile.print(value3[25]);
  logFile.print("},{\"H18\":");
  logFile.print(value3[26]);
  logFile.print("}]]&time=");
  logFile.print(currentTime - 7200);
  logFile.print("&apikey=");
  logFile.print(apikey);
  logFile.println(" HTTP/1.1");
  //logFile.println("Host:genatfab.ovh");
  logFile.println("Host:localhost");
  logFile.println("User-Agent: Arduino-ethernet");
  logFile.println("Connection: keep-alive");
  logFile.println();
  // Force data to SD and update the directory entry to avoid data loss.
  if (!logFile.sync() || logFile.getWriteError()) {
    error("logFile write error");
  }
  logFile.close();
  Serial.print(F("Writing "));
  Serial.print(fileName);
  Serial.println(F(" on SD card"));
  Serial.print(F("\r\n"));
}

/********************* this method makes a HTTP connection to the server: ************************/
void sendData() {

  DateTime now = rtc.now();
  unsigned long currentTime = now.unixtime();

  client.print("GET /emoncms/input/bulk?data=[[");
  client.print("34,0,{\"temp\":");                                      // Node 0 : température
  client.print(temperature);
  client.print("}],[");
  client.print("34,1,{\"H2\":");                                        // Node 1 : MQ-8
  client.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_H2));
  client.print("}],[");
  client.print("34,2,{\"power1\":");                                    // Node 2 : CT-1
  client.print(Irms1 * volt);
  client.print("}],[");
  client.print("34,3,{\"power2\":");                                    // Node 3 : CT-2
  client.print(Irms2 * volt);
  client.print("}],[");
  client.print("34,4,{\"V\":");                                         // Node 4 : MPPT 150/35
  client.print(value1[3]);
  client.print("},{\"I\":");
  client.print(value1[4]);
  client.print("},{\"VPV\":");
  client.print(value1[5]);
  client.print("},{\"PPV\":");
  client.print(value1[6]);
  client.print("},{\"CS\":");
  client.print(value1[7]);
  client.print("},{\"ERR\":");
  client.print(value1[8]);
  client.print("},{\"H19\":");
  client.print(value1[10]);
  client.print("},{\"H20\":");
  client.print(value1[11]);
  client.print("},{\"H21\":");
  client.print(value1[12]);
  client.print("},{\"H22\":");
  client.print(value1[13]);
  client.print("},{\"H23\":");
  client.print(value1[14]);
  client.print("},{\"HSDS\":");
  client.print(value1[15]);
  client.print("}],[");
  client.print("34,5,{\"MODE\":");          // Node 5 : Inverter_24V_500VA
  client.print(value2[3]);
  client.print("},{\"CS\":");
  client.print(value2[4]);
  client.print("},{\"AC_OUT_V\":");
  client.print(value2[5]);
  client.print("},{\"AC_OUT_I\":");
  client.print(value2[6]);
  client.print("},{\"V\":");
  client.print(value2[7]);
  client.print("},{\"AR\":");
  client.print(value2[8]);
  client.print("},{\"WARN\":");
  client.print(value2[9]);
  client.print("}],[");
  client.print("34,6,{\"V\":");           // Node 6 : BMV700
  client.print(value3[1]);
  client.print("},{\"I\":");
  client.print(value3[2]);
  client.print("},{\"P\":");
  client.print(value3[3]);
  client.print("},{\"CE\":");
  client.print(value3[4]);
  client.print("},{\"SOC\":");
  client.print(value3[5]);
  client.print("},{\"TTG\":");
  client.print(value3[6]);
  client.print("},{\"AR\":");
  client.print(value3[9]);
  client.print("},{\"H1\":");
  client.print(value3[13]);
  client.print("},{\"H2\":");
  client.print(value3[14]);
  client.print("},{\"H3\":");
  client.print(value3[15]);
  client.print("},{\"H4\":");
  client.print(value3[16]);
  client.print("},{\"H5\":");
  client.print(value3[17]);
  client.print("},{\"H6\":");
  client.print(value3[18]);
  client.print("},{\"H7\":");
  client.print(value3[19]);
  client.print("},{\"H8\":");
  client.print(value3[20]);
  client.print("},{\"H9\":");
  client.print(value3[21]);
  client.print("},{\"H10\":");
  client.print(value3[22]);
  client.print("},{\"H11\":");
  client.print(value3[23]);
  client.print("},{\"H12\":");
  client.print(value3[24]);
  client.print("},{\"H17\":");
  client.print(value3[25]);
  client.print("},{\"H18\":");
  client.print(value3[26]);
  client.print("}]]&time=");
  client.print(currentTime - 7200);
  client.print("&apikey=");
  client.print(apikey);
  client.println(" HTTP/1.1");
  //client.println("Host:genatfab.ovh");
  client.println("Host:localhost");
  client.println("User-Agent: Arduino-ethernet");
  client.println("Connection: close");
  client.println();
}
//============================================================

// Serial Handling
// ---
// This block handles the serial reception of the data in a
// non blocking way. It checks the Serial line for characters and
// parses them in fields. If a block of data is send, which always ends
// with "Checksum" field, the whole block is checked and if deemed correct
// copied to the 'value' array.

/****************************************** Serial1 ***************************************/
void RecvWithEndMarker1() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial1.available() > 0 && new_data1 == false) {
    rc = Serial1.read();
    if (rc != endMarker) {
      receivedChars1[ndx] = rc;
      ndx++;
      if (ndx >= buffsize) {
        ndx = buffsize - 1;
      }
    }
    else {
      receivedChars1[ndx] = '\0'; // terminate the string
      ndx = 0;
      new_data1 = true;
    }
    yield();
  }
}
//============================================================
void ParseData1() {
  char * strtokIndx1; // this is used by strtok() as an index
  strtokIndx1 = strtok(tempChars1, "\t");     // get the first part - the label
  // The last field of a block is always the Checksum
  if (strcmp(strtokIndx1, "Checksum") == 0) {
    blockend1 = true;
  }
  strcpy(recv_label1[blockindex1], strtokIndx1); // copy it to label

  // Now get the value
  strtokIndx1 = strtok(NULL, "\r");    // This continues where the previous call left off until '/r'.
  if (strtokIndx1 != NULL) {           // We need to check here if we don't receive NULL.
    strcpy(recv_value1[blockindex1], strtokIndx1);
  }
  blockindex1++;

  if (blockend1) {
    // We got a whole block into the received data.
    // Check if the data received is not corrupted.
    // Sum off all received bytes should be 0;
    byte checksum = 0;
    for (int x = 0; x < blockindex1; x++) {
      // Loop over the labels and value gotten and add them.
      // Using a byte so the the % 256 is integrated.
      char *v1 = recv_value1[x];
      char *l1 = recv_label1[x];
      while (*v1) {
        checksum += *v1;
        v1++;
      }
      while (*l1) {
        checksum += *l1;
        l1++;
      }
      // Because we strip the new line(10), the carriage return(13) and
      // the horizontal tab(9) we add them here again.
      checksum += 32;
    }
    // Checksum should be 0, so if !0 we have correct data.
    if (!checksum) {
      // Since we are getting blocks that are part of a
      // keyword chain, but are not certain where it starts
      // we look for the corresponding label. This loop has a trick
      // that will start searching for the next label at the start of the last
      // hit, which should optimize it.
      int start = 0;
      for (int i = 0; i < blockindex1; i++) {
        for (int j = start; (j - start) < num_keywords1; j++) {
          if (strcmp(recv_label1[i], keywords1[j % num_keywords1]) == 0) {
            // found the label, copy it to the value array
            strcpy(value1[j], recv_value1[i]);
            start = (j + 1) % num_keywords1; // start searching the next one at this hit +1
            break;
          }
        }
      }
    }
    // Reset the block index, and make sure we clear blockend1.
    blockindex1 = 0;
    blockend1 = false;
  }
}
//============================================================
void HandleNewData1() {
  // We have gotten a field of data
  if (new_data1 == true) {
    //Copy it to the temp array because parseData will alter it.
    strcpy(tempChars1, receivedChars1);
    ParseData1();
    new_data1 = false;
  }
}
//============================================================
void PrintValues1() {
  for (int i = 0; i < num_keywords1; i++) {
    Serial.print(keywords1[i]);
    Serial.print("\t");
    Serial.println(value1[i]);
  }
}
//============================================================
void PrintEverySecond1() {
  static unsigned long prev_millis;
  if (millis() - prev_millis > 1000) {
    PrintValues1();
    prev_millis = millis();
  }
}

/****************************************** Serial2 ***************************************/
void RecvWithEndMarker2() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial2.available() > 0 && new_data2 == false) {
    rc = Serial2.read();
    if (rc != endMarker) {
      receivedChars2[ndx] = rc;
      ndx++;
      if (ndx >= buffsize) {
        ndx = buffsize - 1;
      }
    }
    else {
      receivedChars2[ndx] = '\0'; // terminate the string
      ndx = 0;
      new_data2 = true;
    }
    yield();
  }
}
//============================================================
void ParseData2() {
  char * strtokIndx2; // this is used by strtok() as an index
  strtokIndx2 = strtok(tempChars2, "\t");     // get the first part - the label
  // The last field of a block is always the Checksum
  if (strcmp(strtokIndx2, "Checksum") == 0) {
    blockend2 = true;
  }
  strcpy(recv_label2[blockindex2], strtokIndx2); // copy it to label

  // Now get the value
  strtokIndx2 = strtok(NULL, "\r");    // This continues where the previous call left off until '/r'.
  if (strtokIndx2 != NULL) {           // We need to check here if we don't receive NULL.
    strcpy(recv_value2[blockindex2], strtokIndx2);
  }
  blockindex2++;

  if (blockend2) {
    // We got a whole block into the received data.
    // Check if the data received is not corrupted.
    // Sum off all received bytes should be 0;
    byte checksum = 0;
    for (int x = 0; x < blockindex2; x++) {
      // Loop over the labels and value gotten and add them.
      // Using a byte so the the % 256 is integrated.
      char *v2 = recv_value2[x];
      char *l2 = recv_label2[x];
      while (*v2) {
        checksum += *v2;
        v2++;
      }
      while (*l2) {
        checksum += *l2;
        l2++;
      }
      // Because we strip the new line(10), the carriage return(13) and
      // the horizontal tab(9) we add them here again.
      checksum += 32;
    }
    // Checksum should be 0, so if !0 we have correct data.
    if (!checksum) {
      // Since we are getting blocks that are part of a
      // keyword chain, but are not certain where it starts
      // we look for the corresponding label. This loop has a trick
      // that will start searching for the next label at the start of the last
      // hit, which should optimize it.
      int start = 0;
      for (int i = 0; i < blockindex2; i++) {
        for (int j = start; (j - start) < num_keywords2; j++) {
          if (strcmp(recv_label2[i], keywords2[j % num_keywords2]) == 0) {
            // found the label, copy it to the value array
            strcpy(value2[j], recv_value2[i]);
            start = (j + 1) % num_keywords2; // start searching the next one at this hit +1
            break;
          }
        }
      }
    }
    // Reset the block index, and make sure we clear blockend1.
    blockindex2 = 0;
    blockend2 = false;
  }
}
//============================================================
void HandleNewData2() {
  // We have gotten a field of data
  if (new_data2 == true) {
    //Copy it to the temp array because parseData will alter it.
    strcpy(tempChars2, receivedChars2);
    ParseData2();
    new_data2 = false;
  }
}
//============================================================
void PrintValues2() {
  for (int i = 0; i < num_keywords2; i++) {
    Serial.print(keywords2[i]);
    Serial.print("\t");
    Serial.println(value2[i]);
  }
}
//============================================================
void PrintEverySecond2() {
  static unsigned long prev_millis;
  if (millis() - prev_millis > 1000) {
    PrintValues2();
    prev_millis = millis();
  }
}

/****************************************** Serial3 ***************************************/
void RecvWithEndMarker3() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial3.available() > 0 && new_data3 == false) {
    rc = Serial3.read();
    if (rc != endMarker) {
      receivedChars3[ndx] = rc;
      ndx++;
      if (ndx >= buffsize) {
        ndx = buffsize - 1;
      }
    }
    else {
      receivedChars3[ndx] = '\0'; // terminate the string
      ndx = 0;
      new_data3 = true;
    }
    yield();
  }
}
//============================================================
void ParseData3() {
  char * strtokIndx3; // this is used by strtok() as an index
  strtokIndx3 = strtok(tempChars3, "\t");     // get the first part - the label
  // The last field of a block is always the Checksum
  if (strcmp(strtokIndx3, "Checksum") == 0) {
    blockend3 = true;
  }
  strcpy(recv_label3[blockindex3], strtokIndx3); // copy it to label

  // Now get the value
  strtokIndx3 = strtok(NULL, "\r");    // This continues where the previous call left off until '/r'.
  if (strtokIndx3 != NULL) {           // We need to check here if we don't receive NULL.
    strcpy(recv_value3[blockindex3], strtokIndx3);
  }
  blockindex3++;

  if (blockend3) {
    // We got a whole block into the received data.
    // Check if the data received is not corrupted.
    // Sum off all received bytes should be 0;
    byte checksum = 0;
    for (int x = 0; x < blockindex3; x++) {
      // Loop over the labels and value gotten and add them.
      // Using a byte so the the % 256 is integrated.
      char *v3 = recv_value3[x];
      char *l3 = recv_label3[x];
      while (*v3) {
        checksum += *v3;
        v3++;
      }
      while (*l3) {
        checksum += *l3;
        l3++;
      }
      // Because we strip the new line(10), the carriage return(13) and
      // the horizontal tab(9) we add them here again.
      checksum += 32;
    }
    // Checksum should be 0, so if !0 we have correct data.
    if (!checksum) {
      // Since we are getting blocks that are part of a
      // keyword chain, but are not certain where it starts
      // we look for the corresponding label. This loop has a trick
      // that will start searching for the next label at the start of the last
      // hit, which should optimize it.
      int start = 0;
      for (int i = 0; i < blockindex3; i++) {
        for (int j = start; (j - start) < num_keywords3; j++) {
          if (strcmp(recv_label3[i], keywords3[j % num_keywords3]) == 0) {
            // found the label, copy it to the value array
            strcpy(value3[j], recv_value3[i]);
            start = (j + 1) % num_keywords3; // start searching the next one at this hit +1
            break;
          }
        }
      }
    }
    // Reset the block index, and make sure we clear blockend1.
    blockindex3 = 0;
    blockend3 = false;
  }
}
//============================================================
void HandleNewData3() {
  // We have gotten a field of data
  if (new_data3 == true) {
    //Copy it to the temp array because parseData will alter it.
    strcpy(tempChars3, receivedChars3);
    ParseData3();
    new_data3 = false;
  }
}
//============================================================
void PrintValues3() {
  for (int i = 0; i < num_keywords3; i++) {
    Serial.print(keywords3[i]);
    Serial.print("\t");
    Serial.println(value3[i]);
  }
}
//============================================================
void PrintEverySecond3() {
  static unsigned long prev_millis;
  if (millis() - prev_millis > 1000) {
    PrintValues3();
    prev_millis = millis();
  }
}

/*************************************************************************************************/
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//============================================================
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(19200);
  Serial1.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);

  Wire.begin();
  rtc.begin();

  if (! rtc.isrunning()) {
    Serial.println(F("RTC ne fonctionne PAS!"));
    // La ligne qui suit ajuste le RTC à la date et time du moment de compilation
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }

  /* Calibrating the gaz sensor. Please make sure the sensor is in clean air
    when you perform the calibration */
  Serial.print("Calibrating...\n");
  Ro = MQCalibration(MQ_PIN);

  Serial.print("Calibration is done...\n");
  Serial.print(F("Ro="));
  Serial.print(Ro);
  Serial.print(F("kohm"));
  Serial.print(F("\n"));


  // Affectation des pins PD3 et PD7 à la ventilation (relais 2 bobines)
  pinMode(3, OUTPUT);
  pinMode(7, OUTPUT);

  emon1.current(currentSensor1Pin, ct_calibration);
  emon2.current(currentSensor2Pin, ct_calibration);

  // Mega
  pinMode(53, OUTPUT);
  // disable w5100 SPI while setting up SD
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  // enable SD SPI
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  if (!sd.begin(chipSelect, SPI_FULL_SPEED))
  {
    sd.initErrorHalt();
  }

  Ethernet.begin(mac, ip, gw, subnet);
  digitalWrite(10, HIGH);
  delay(2000);

  Serial.println(Ethernet.localIP());
}
//============================================================
void doHTTP() {

  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  if (!client.connected() && (millis() - lastConnectionTime > postingInterval)) {
    if (client.connect(server, 80)) {
      Serial.println(F("\r\n"));
      Serial.println(F("Connecting ..."));
      readSD();
      sendData();
      lastConnectionTime = millis();
    }

    // if the server's disconnected more than 2 min, stop the client and write data on sd card:
    if (!client.connected() && (millis() - lastConnectionTime >=  postingInterval * 1)) {
      Serial.println(F("disconnecting from server."));
      client.stop();
      writeSD();
      //delay(5000);
    }
  }

  lastConnected = client.connected();
}

//============================================================
void readSD() {

  File logFile;
  char fileName;
  sd.ls();
  int i = 0;
  sd.vwd()->rewind();
  while (logFile.openNext(sd.vwd(), O_READ | O_WRITE) && i < 10) {
    String dataBuff = logFile.readStringUntil('\0');
    client.print(dataBuff);
    Serial.println(dataBuff);
    if (!logFile.remove()) Serial.println(F("Error logFile.remove"));
    delay(50);
    i++;
  }

  if (!sd.exists(fileName)) {
    Serial.println(F("Nothing to send from SD card"));
  }
}
//============================================================
void loop() {

  // Receive information on Serial from MPPT
  RecvWithEndMarker1();
  HandleNewData1();
  RecvWithEndMarker2();
  HandleNewData2();
  RecvWithEndMarker3();
  HandleNewData3();

  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  if (!client.connected() && (millis() - lastConnectionTime > postingInterval)) {

    // Lit la température ambiante à ~1Hz
    if (getTemperature(&temperature, true) != READ_OK)
    {
      Serial.println(F("Erreur de lecture du capteur"));
      return;
    }

    // Lit la valeur d'hydrogène
    MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_H2);
    // Lit les valeurs de courant
    Irms1 = emon1.calcIrms(1480);
    Irms2 = emon2.calcIrms(1480);




    // Affiche la température
    Serial.print(F("Température : "));
    Serial.print(temperature, 2);
    Serial.write("\u00B0"); // Symbole ° en utf)8
    Serial.print(F("C"));
    Serial.println(F("\n"));

    //Affiche la valeur d'hydrogène
    Serial.print(F("H2 : "));
    Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_H2));
    Serial.println(F( " ppm" ));

    //Affiche les valeurs de courant
    Serial.print(F("Courant_1 : "));
    Serial.print(Irms1);
    Serial.print(F(" A"));
    Serial.print(F(" Power1 : "));
    Serial.print(Irms1 * volt);
    Serial.println(F(" W"));
    Serial.print(F("Courant_2 : "));
    Serial.print(Irms2);
    Serial.print(F(" A"));
    Serial.print(F(" Power2 : "));
    Serial.print(Irms2 * volt);
    Serial.println(F(" W"));
    Serial.println(F("\n"));

    // Actions sur les relais
    if (temperature >= tempMax || MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_H2) >= valH2max) {
      digitalWrite(7, HIGH);
      digitalWrite(3, LOW);
    }
    else {
      digitalWrite(7, LOW);
      digitalWrite(3, HIGH);
    }

    // Just print the values every second,
    // Add your own code here to use the data.
    // Make sure to not used delay(X)s of bigger than 50ms,
    // so make use of the same principle used in PrintEverySecond()
    // or use some sort of Alarm/Timer Library
    PrintEverySecond1();
    PrintEverySecond2();
    PrintEverySecond3();

    if (client.connect(server, 80)) {
      Serial.println(F("\r\n"));
      Serial.println(F("Connecting ..."));
      readSD();
      sendData();
      lastConnectionTime = millis();
    }

    // if the server's disconnected more than 2 min, stop the client and write data on sd card:
    if (!client.connected() && (millis() - lastConnectionTime >=  postingInterval * 1)) {
      Serial.println(F("disconnecting from server."));
      client.stop();
      writeSD();
      //delay(5000);
    }
  }

  lastConnected = client.connected();
  //Serial.println(freeRam());
}
