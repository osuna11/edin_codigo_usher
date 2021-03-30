 /**Attached interrupts
  *GPIO27   --->    SW_PUSH_MENIQUE
  *GPIO15   --->    SW_PUSH_ANULAR
  *GPIO14   --->    SW_PUSH_MEDIO
  *GPIO04   --->    SW_PUSH_INDICE
  ***********
  *PWM MOTORS 
  * GPIO23    --->    MTR_5
  * GPIO19    --->    MTR_4
  * GPIO18    --->    MTR_3
  * GPIO05    --->    MTR_2
  * GPIO32    --->    MTR_1
  * GPIO33    --->    ENABLE
  * 
   */
  

// PubNub MQTT example using ESP32.
#include <WiFi.h>
#include <ArduinoJson.h>                                                                                  //----------------------------JSON Arduino Library
#include <PubSubClient.h>
#include <TinyPICO.h>
#include "settings.h"
#include <stdlib.h>

TinyPICO tp = TinyPICO();

//*****************************************************************************Definicion de Variables Globales
String Sipaddrs = "000.000.000.000";
uint32_t chipId = 0;                                                                                      //variable donde se alamacena el identificador de cliente para el servicio de MQTT (OJO Este debe ser un identificador unico para cada dispositivo fisico sino se reiniciara el servidor MQTT)
int retry = 0;                                                                                            //variable donde se almacenan los intentos de conexion fallidos
char dedo_I[20];
char dedo_p[20];
char dedo_medi[20];
char dedo_a[20];
char dedo_meni[20];
String repuesta;
int segundos=0;

char Id_mano[10];
int cantidad_movimiento;
/////////////////////////////////////////Control de carga //////////////////////////////////////////////////
// Interval between internal temperature reads
unsigned long next_temp_read = 0;   // Next time step in milliseconds
uint8_t temp_read_interval = 1000;  // This is in milliseconds

unsigned long lastMillis_BAT;
unsigned long lastMillis_LED;
 unsigned long current_millis=0;
 int estado=digitalRead(14);
 unsigned long Last_Normal_Reset_Millis;                                                             //Variable para llevar conteo del tiempo desde la ultima publicacion
unsigned long Last_Update_Millis; 




//void NormalReset() {
 // if (millis() - Last_Normal_Reset_Millis > 60 * 60 * Universal_1_sec_Interval) {
  //  hora++;
   // if (hora > 24) {
    //  msg = ("24h Normal Reset");
     // VBat = 4.2; //Bateria();
     // publishRF_ID_Manejo();        //publishRF_ID_Manejo (String IDModulo,String MSG,float vValue, int fail,String Tstamp)
     // hora = 0;
     // ESP.restart();
   // }
   // Last_Normal_Reset_Millis = millis(); //Actulizar la ultima hora de envio
  //}
//}
void bateria_estado(){
   if (millis() - lastMillis_BAT > 60000) {
    lastMillis_BAT = millis();
    float battery =  tp.GetBatteryVoltage();
    bool ischarging = tp.IsChargingBattery();
    Serial.print(F("voltaje de la bateria: "));
    Serial.println( battery);
    Serial.print(F("La bateria esta cargando"));
    Serial.println(ischarging);
     if (millis() - lastMillis_LED > 30000) {
   if(battery>=4.51){
   tp.DotStar_SetPixelColor( 26, 255, 0);
   }
   if(battery<=4.50 && battery>=4.00){
   tp.DotStar_SetPixelColor( 255, 171, 0);
   }
    if(battery<=3.9 && battery>=3.7){
   tp.DotStar_SetPixelColor( 255, 0, 0);
   }
  
   }
   }
   
  
}
//******************************************************************************FSM Settings
bool mesgrecv = false;
static enum {STATE_IDLE, STATE_PREGUNTA, STATE_TRANSMIT_RESPUESTA} fsm_state = STATE_IDLE;

//******************************************************************************Variables de mensaje desde concentrador MQTT para motores. 

int motor_channel, motor_time, motor_effect;

int idOperation       = 0;
int idModule          = 0;
int gloveCode         = 0;
int answer            = 0;
int vibraciones ;
//******************************************************************************Variables de Botones. 
struct boton {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  volatile bool pressed;
};

boton btn_4 = {4 ,0, false};
boton btn_3 = {14, 0, false};
boton btn_2 = {15, 0, false};
boton btn_1 = {27, 0, false};

void IRAM_ATTR isr_btn1(){
  btn_1.numberKeyPresses += 1;
  btn_1.pressed = true;
   
}

void IRAM_ATTR isr_btn2(){
  btn_2.numberKeyPresses += 1;
  btn_2.pressed = true;
}

void IRAM_ATTR isr_btn3(){
  btn_3.numberKeyPresses += 1;
  btn_3.pressed = true;

 unsigned long t_inicio=0;
  unsigned long t_inicio2=0;
  unsigned long mili2;
  unsigned long calculo;
  Serial.println(estado);
if(estado!=false){
  t_inicio=current_millis;
  Serial.println("Button press");
  }
  if(estado==1  && t_inicio>=30000){
     Serial.println("Button long pressed");
   Serial.println(F("enviando respuesta"));
  }
  

   
        
     //     if(button_pressduration >=t_presionado){
       //     Serial.println(F("enviando respuesta"));
         //   String resultado=String("BTN_1")+btn_1.numberKeyPresses+String("BTN_2")+btn_2.numberKeyPresses+String("BTN_3")+btn_3.numberKeyPresses+String("BTN_4")+btn_4.numberKeyPresses;
           // resultado=repuesta;
            //Serial.println(repuesta);
            //fsm_state = STATE_TRANSMIT_RESPUESTA;
            
            //}
            

    
}

void IRAM_ATTR isr_btn4(){
  btn_4.numberKeyPresses += 1;
  btn_4.pressed = true;
}


//*****************************************************************************Variables de Motores

// Pin para inicio y frenado de Motores
int enable1Pin = 33; 
volatile bool EnablePinState;

 
// Propiedades de PWM que funcionan ocn driver y motores. 
const int freq = 3000;
const int resolution = 8;

uint16_t  dutyCyclefull = 255;
uint16_t  dutyCycleHalf = 150;
uint16_t  mediano = 180;
uint16_t  dutyCycleLow = 15;
uint16_t  initDutyCycle = 0;

uint16_t  clickTime = 0;

//******************************************************************************Estructura de puertos a Motores 
struct vmotor {
  const uint8_t motorPin;     //Puerto fisico al que se encuntra cableado el puerto de PWM del Chip DRV2603
  const uint8_t pwmChannel;  //Canal logico de comunicaciones para el puerto
};

//Mapa de asignacion de puertos y canales para PWM ver esquematico para confirmar puertos.
#ifdef IZQUIERDA
vmotor pulgar = {23, 4}; //GPIO23 canal 4
vmotor indice = {19, 3}; //GPIO19 canal 3
vmotor medio = {18, 2}; //GPIO18 canal 2
vmotor anular = {5,  1}; //GPIO5 canal 1
vmotor menique = {32, 0}; //GPIO32 canal 0

#elif DERECHA

vmotor pulgar = {32, 0}; //GPIO23 canal 4
vmotor indice = {5,  1}; //GPIO19 canal 3
vmotor medio = {18, 2}; //GPIO18 canal 2
vmotor anular = {19, 3} //GPIO5 canal 1
vmotor menique = {23, 4};; //GPIO32 canal 0

#endif 
//*******************************************************************************Funcion de PWM para puerto 32 y 33 del ESP32
void aPinMode(int pinNum, int pinDir) {
  // Enable GPIO32 or 33 as output. 
  if (pinNum == 32 || pinNum == 33) {
    uint64_t gpioBitMask = (pinNum == 32) ? 1ULL<<GPIO_NUM_32 : 1ULL<<GPIO_NUM_33;
    gpio_mode_t gpioMode = (pinDir == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = gpioMode;
    io_conf.pin_bit_mask = gpioBitMask;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
  } else pinMode(pinNum, pinDir);
}
void menos_indice(){
  ledcWrite(menique.pwmChannel, initDutyCycle);
   ledcWrite(pulgar.pwmChannel, initDutyCycle);
    ledcWrite(anular.pwmChannel, initDutyCycle);
     ledcWrite(medio.pwmChannel, initDutyCycle);
  }
  void menos_menique(){
     ledcWrite(indice.pwmChannel, initDutyCycle);
   ledcWrite(pulgar.pwmChannel, initDutyCycle);
    ledcWrite(anular.pwmChannel, initDutyCycle);
     ledcWrite(medio.pwmChannel, initDutyCycle);
  }
  void menos_medio(){
     ledcWrite(menique.pwmChannel, initDutyCycle);
   ledcWrite(pulgar.pwmChannel, initDutyCycle);
    ledcWrite(anular.pwmChannel, initDutyCycle);
     ledcWrite(indice.pwmChannel, initDutyCycle);
  }
  void menos_anular(){
     ledcWrite(menique.pwmChannel, initDutyCycle);
   ledcWrite(pulgar.pwmChannel, initDutyCycle);
    ledcWrite(indice.pwmChannel, initDutyCycle);
     ledcWrite(medio.pwmChannel, initDutyCycle);
  }
  void menos_pulgar(){
     ledcWrite(menique.pwmChannel, initDutyCycle);
   ledcWrite(indice.pwmChannel, initDutyCycle);
    ledcWrite(anular.pwmChannel, initDutyCycle);
     ledcWrite(medio.pwmChannel, initDutyCycle);
  }
void breakMotors(int stop_enableMotor_pin) {
  tp.DotStar_SetPixelColor( 255,0, 255   );
  Serial.println("enable pin LOW");
  digitalWrite(stop_enableMotor_pin, LOW); 
}

void enableMotors(int start_enableMotor_pin){
  tp.DotStar_SetPixelColor( 6, 14, 255   );
  Serial.println("enable pin HIGH");
  digitalWrite(start_enableMotor_pin, HIGH);
}


void rampUp(int up_motor_channel){
  enableMotors(enable1Pin);
  tp.DotStar_SetPixelColor( 0, 255, 255   );
  for(int dutyCycle = 170; dutyCycle <= 255; dutyCycle++){
    // changing the LED brightness with PWM
    Serial.print("Set dutyCycle:");
    Serial.println(dutyCycle);
    // changing the LED brightness with PWM
    ledcWrite(up_motor_channel, dutyCycle);
    delay(68);
  }
  breakMotors(enable1Pin);     
}
void multi(int up_motor_channel){
  enableMotors(enable1Pin); 
    ledcWrite(up_motor_channel, mediano);
    delay(500);
    ledcWrite(up_motor_channel, initDutyCycle);
 delay(500);
 ledcWrite(up_motor_channel, mediano);
    delay(500);
    ledcWrite(up_motor_channel, initDutyCycle);
 delay(500);
     ledcWrite(up_motor_channel, dutyCyclefull);
     delay(1500);
  breakMotors(enable1Pin);     
}

void divi(int up_motor_channel){
  enableMotors(enable1Pin); 
    ledcWrite(up_motor_channel, dutyCyclefull);
    delay(1500);
    ledcWrite(up_motor_channel, initDutyCycle);
 delay(100);
    ledcWrite(up_motor_channel, mediano);
 delay(1000);
  breakMotors(enable1Pin);     
}

void rampDown(int down_motor_channel){
  enableMotors(enable1Pin);
  tp.DotStar_SetPixelColor( 255, 255, 0  );
  for(int dutyCycle = 255; dutyCycle >= 160; dutyCycle--){
    // changing the LED brightness with PWM
    Serial.print("Set dutyCycle:");
    Serial.println(dutyCycle);
    // changing the LED brightness with PWM
    ledcWrite(down_motor_channel, dutyCycle);
    delay(65);
  }
  breakMotors(enable1Pin);    
}


void motorClick(int motor_channel, int motor_time){
  enableMotors(enable1Pin);
  Serial.print("Click Motor channel:");
  Serial.println(motor_channel);
  Serial.print(" for time in ms:");
  Serial.println(motor_time);
  delay(100);
  ledcWrite(motor_channel, dutyCyclefull);
  delay(motor_time);
  ledcWrite(motor_channel, dutyCycleHalf);
  breakMotors(enable1Pin);
}
void vibra_resultado(){
  menos_pulgar();
  menos_medio();
  menos_indice();
  menos_menique();
  menos_anular();
 enableMotors(enable1Pin);
  ledcWrite(pulgar.pwmChannel, dutyCyclefull );
  ledcWrite(anular.pwmChannel, dutyCyclefull );
   ledcWrite(medio.pwmChannel, dutyCyclefull );
    ledcWrite(indice.pwmChannel, dutyCyclefull );
     ledcWrite(menique.pwmChannel, dutyCyclefull );
 delay(500);
 ledcWrite(pulgar.pwmChannel, initDutyCycle );
  ledcWrite(anular.pwmChannel, initDutyCycle );
   ledcWrite(medio.pwmChannel, initDutyCycle );
    ledcWrite(indice.pwmChannel, initDutyCycle );
     ledcWrite(menique.pwmChannel, initDutyCycle );
 delay(100);
   ledcWrite(pulgar.pwmChannel, dutyCyclefull );
  ledcWrite(anular.pwmChannel, dutyCyclefull );
   ledcWrite(medio.pwmChannel, dutyCyclefull );
    ledcWrite(indice.pwmChannel, dutyCyclefull );
     ledcWrite(menique.pwmChannel, dutyCyclefull );
 delay(500);
  ledcWrite(pulgar.pwmChannel, initDutyCycle );
  ledcWrite(anular.pwmChannel, initDutyCycle );
   ledcWrite(medio.pwmChannel, initDutyCycle );
    ledcWrite(indice.pwmChannel, initDutyCycle );
     ledcWrite(menique.pwmChannel, initDutyCycle );
 delay(100);
  breakMotors(enable1Pin);
}
void mov_dedo_indice(int mov_intensidad, int mov_tipo,int mov_cantidad){
    if(mov_tipo == 2){
Serial.print("tipo de vibracion 1");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
   menos_indice();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampUp(indice.pwmChannel);
 delay(10);  
}
 breakMotors(enable1Pin);

}
  if(mov_tipo==1){
    Serial.print("tipo de vibracion 2");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_indice();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  ledcWrite(indice.pwmChannel, dutyCyclefull );
 delay(500);
 ledcWrite(indice.pwmChannel, initDutyCycle);
 delay(500);  
  }
   breakMotors(enable1Pin);
  }
  
  if(mov_tipo==3){
    Serial.print("tipo de vibracion 3");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
          menos_indice();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampDown(indice.pwmChannel);
 delay(10);  
  }
   breakMotors(enable1Pin);
  }

  if(mov_tipo==4){
    Serial.print("tipo de vibracion 4");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_indice();
 enableMotors(enable1Pin);
       multi(indice.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
   if(mov_tipo==5){
    Serial.print("tipo de vibracion 5");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_indice();
 enableMotors(enable1Pin);
       divi(indice.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
}

void mov_dedo_menique(int mov_intensidad, int mov_tipo,int mov_cantidad){
  if(mov_tipo==1){
    Serial.print("tipo de vibracion 1");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
           menos_menique();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  ledcWrite(menique.pwmChannel, dutyCyclefull );
 delay(500);
 ledcWrite(menique.pwmChannel, initDutyCycle);
 delay(500);  
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==2){
    Serial.print("tipo de vibracion 2");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
          menos_menique();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampUp(menique.pwmChannel);
 delay(10);  
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==3){
    Serial.print("tipo de vibracion 3");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
           menos_menique();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampDown(menique.pwmChannel);
 delay(10);  
  }
  breakMotors(enable1Pin);
  }
   if(mov_tipo==4){
    Serial.print("tipo de vibracion 4");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_menique();
 enableMotors(enable1Pin);
       multi(menique.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==5){
    Serial.print("tipo de vibracion 5");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_menique();
 enableMotors(enable1Pin);
       divi(menique.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
}

void mov_dedo_medio(int mov_intensidad, int mov_tipo,int mov_cantidad){
    if(mov_tipo==1){
    Serial.print("tipo de vibracion 1");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
           menos_medio();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  ledcWrite(medio.pwmChannel, dutyCyclefull );
 delay(500);
 ledcWrite(medio.pwmChannel, initDutyCycle);
 delay(500);  
      
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==2){
    Serial.print("tipo de vibracion 2");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
           menos_medio();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampUp(medio.pwmChannel);
 delay(10);  
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==3){
    Serial.print("tipo de vibracion 3");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
       menos_medio();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampDown(medio.pwmChannel);
 delay(10); 
  }
   breakMotors(enable1Pin);
  }
   if(mov_tipo==4){
    Serial.print("tipo de vibracion 4");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_medio();
 enableMotors(enable1Pin);
       multi(medio.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==5){
    Serial.print("tipo de vibracion 5");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_medio();
 enableMotors(enable1Pin);
       divi(medio.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
}

void mov_dedo_anular(int mov_intensidad, int mov_tipo,int mov_cantidad){
  if(mov_tipo==1){
    Serial.print("tipo de vibracion 1");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
             menos_anular();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  ledcWrite(anular.pwmChannel, dutyCyclefull );
 delay(500);
 ledcWrite(anular.pwmChannel, initDutyCycle);
 delay(500); 
  }
  breakMotors(enable1Pin);
  }
  if(mov_tipo==2){
    Serial.print("tipo de vibracion 2");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
              menos_anular();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampUp(anular.pwmChannel);
 delay(10);  
  }
  breakMotors(enable1Pin);
  }
  if(mov_tipo==3){
    Serial.print("tipo de vibracion 3");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
            menos_anular();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampDown(anular.pwmChannel);
 delay(10); 
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==4){
    Serial.print("tipo de vibracion 4");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_anular();
 enableMotors(enable1Pin);
       multi(anular.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
   if(mov_tipo==5){
    Serial.print("tipo de vibracion 5");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_anular();
 enableMotors(enable1Pin);
       divi(anular.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
    
}

void mov_dedo_pulgar(int mov_intensidad, int mov_tipo,int mov_cantidad){
  if(mov_tipo==1){
    Serial.print("tipo de vibracion 1");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
     menos_pulgar();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  ledcWrite(pulgar.pwmChannel, dutyCyclefull );
 delay(500);
 ledcWrite(pulgar.pwmChannel, initDutyCycle);
 delay(500); 
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==2){
    Serial.print("tipo de vibracion 2");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
               menos_pulgar();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampUp(pulgar.pwmChannel);
 delay(10);  
  }
  breakMotors(enable1Pin);
  }
  if(mov_tipo==3){
    Serial.print("tipo de vibracion 3");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
            menos_pulgar();
      enableMotors(enable1Pin);
          Serial.println(vibraciones);
  rampDown(pulgar.pwmChannel);
 delay(10); 
  }
  breakMotors(enable1Pin);
  }
  if(mov_tipo==4){
    Serial.print("tipo de vibracion 4");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_pulgar();
 enableMotors(enable1Pin);
       multi(pulgar.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
  if(mov_tipo==5){
    Serial.print("tipo de vibracion 5");
for(vibraciones=0 ; vibraciones<mov_cantidad; vibraciones++){
 menos_pulgar();
 enableMotors(enable1Pin);
       divi(pulgar.pwmChannel); 
  }
   breakMotors(enable1Pin);
  }
}

void operation (byte* payloadrsp){
  Serial.println(F("mensaje   recibido de topico operacion"));
  const int capacity = JSON_OBJECT_SIZE(208);
  StaticJsonDocument<capacity> edin_operation_json;
  
  DeserializationError err = deserializeJson(edin_operation_json, payloadrsp);
  
  if (err) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(err.c_str());
    return;
  }
  strlcpy(dedo_I, edin_operation_json["dedo_I"] | "dedo_I", 20);
  Serial.print("dedo_I:  ");
  Serial.println(dedo_I);
  strlcpy(dedo_p, edin_operation_json["dedo_p"] | "dedo_p", 20);
  Serial.print("dedo_p: ");
  Serial.println(dedo_p);
   strlcpy(dedo_medi, edin_operation_json["dedo_medi"] | "dedo_medi", 20);
  Serial.print("dedo_medi: ");
  Serial.println(dedo_medi);
   strlcpy(dedo_a, edin_operation_json["dedo_a"] | "dedo_a", 20);
  Serial.print("dedo_a: ");
  Serial.println(dedo_a);
  strlcpy(dedo_meni, edin_operation_json["dedo_meni"] | "dedo_meni", 20);
  Serial.print("dedo_meni: ");
  Serial.println(dedo_meni);
  strlcpy(Id_mano, edin_operation_json["Id_mano"] | "Id_mano", 10);
  
  Serial.print("Id_mano: ");
  Serial.println(Id_mano);
  //auto cantidad_movimiento=edin_operation_json["M_I"][2].as<int>();
  

 if(strcmp (mano_izquierda,Id_mano)==0){
   if(strcmp (dedo_Indice,dedo_I)==0){
  Serial.println(F("mover el indice"));
  mov_dedo_indice(edin_operation_json["M_I"][0].as<int>(),edin_operation_json["M_I"][1].as<int>(),edin_operation_json["M_I"][2].as<int>());
  }
  else{Serial.println(F("NO mover el indice"));}
  
   if(strcmp (dedo_menique,dedo_meni)==0){
  Serial.println(F("mover el meñique"));
  mov_dedo_menique(edin_operation_json["M_meni"][0].as<int>(),edin_operation_json["M_meni"][1].as<int>(),edin_operation_json["M_meni"][2].as<int>());
  
  }
  else{Serial.println(F("NO mover el menique"));}
  
   if(strcmp (dedo_medio,dedo_medi)==0){
  Serial.println(F("mover el medio"));
   mov_dedo_medio(edin_operation_json["M_Medi"][0].as<int>(),edin_operation_json["M_Medi"][1].as<int>(),edin_operation_json["M_Medi"][2].as<int>());
  }
  else{Serial.println(F("no hay medio"));}
  
   if(strcmp (dedo_anular,dedo_a)==0){
  Serial.println(F("mover el anular"));
  mov_dedo_anular(edin_operation_json["M_A"][0].as<int>(),edin_operation_json["M_A"][1].as<int>(),edin_operation_json["M_A"][2].as<int>());
  }
  else{Serial.println(F("no hay anular"));}
   if(strcmp (dedo_pulgar,dedo_p)==0){
  Serial.println(F("mover el pulgar"));
   mov_dedo_pulgar(edin_operation_json["M_P"][0].as<int>(),edin_operation_json["M_P"][1].as<int>(),edin_operation_json["M_P"][2].as<int>());
  }else{Serial.println(F("no hay pulgar"));}
  
  }

  tp.DotStar_Clear();
 //fsm_state = STATE_PREGUNTA;
  fsm_state = STATE_IDLE;
  exit(0);
          
}


void callback(char* topic, byte* payload, unsigned int length) {
  String payload_buff;
  for (int i=0;i<length;i++) {
    payload_buff = payload_buff+String((char)payload[i]);
  }
  Serial.println(payload_buff); // Print out messages.
  
  if (strcmp (rebootTopic, topic) == 0) {                                                                 //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting in 5 seconds..."));                                                                    //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    delay(5000);
    ESP.restart();                                                                                          //Emitir comando de reinicio para ESP32
  }
 
  if (strcmp (operationTopic, topic) == 0) {                                                                 //verificar si el topico conicide con el Topico updateTopic[] definido en el archivo settings.h local
    operation(payload);                                                                                //enviar a la funcion handleUpdate el contenido del mensaje para su parseo.
  }  
    if (strcmp (cambio, topic) == 0) {
     Serial.println(F("CAMBIANDO A PREGUTNAR.")); 
         vibra_resultado();                                                                   //verificar si el topico conicide con el Topico cambio[] definido en el archivo settings.h local
       fsm_state = STATE_PREGUNTA;                                                                             //eenvia al estado de respoder
  }  
  

   

}


WiFiClient MQTTclient;
PubSubClient client(mqttServer, mqttPort, callback, MQTTclient);


long lastReconnectAttempt = 0;

boolean reconnect() {
  if (client.connect(clientId,"edin","edin")) {
    client.subscribe(rebootTopic); // Subscribe to channel.
    client.subscribe(operationTopic); // Subscribe to channel.    
  }
  return client.connected();
}

//**********************************************************************************Get Chip ID*********************************************************************************
void GloveID(){
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  
  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getSdkVersion(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getFlashChipSpeed());
  Serial.print("Chip ID: "); Serial.println(chipId);
  
  delay(3000);
}
void configuration_json (){
  const int capacity = JSON_OBJECT_SIZE(10);
  StaticJsonDocument<capacity> edin_json_config_doc;
  // create an object
  JsonObject object = edin_json_config_doc.to<JsonObject>();

  object["AliveUpdate:hrs"]   = isalivemsg_interval/3600000;
  object["MqttServer"]        = mqttServer;
  char buf[16];
  sprintf(buf, "IP:%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
  Serial.println(String(buf));
  object["MqttPort"]          = mqttPort;
  object["gloveCode"]         = chipId ;
  object["IpNodo"]            = String (buf);
  object["ClientID"]          = String(clientId);
    
  String output;
  size_t n = serializeJson(object, output);                                                                  //SAve CPU cycles by calculatinf the size.
  Serial.println(F("publishing device manageTopic metadata:"));
  Serial.println(output);
  if (!client.connected()) {
    reconnect();
  }
  if (client.publish(responseTopic, output.c_str(), n)) {
    Serial.println(F("device Publish ok"));
  }else {
    Serial.println(F("device Publish failed:"));
  }

}

void setup() {
  Serial.begin(115200);
  //Setting up Button 1
  Serial.print("Setting pin button:");
  pinMode(btn_1.PIN, INPUT_PULLUP);
  attachInterrupt(btn_1.PIN, isr_btn1, FALLING);
  //Setting up Button #2
  Serial.print("Setting pin button:");
  Serial.println(btn_2.PIN);
  pinMode(btn_2.PIN, INPUT_PULLUP);
  attachInterrupt(btn_2.PIN, isr_btn2, FALLING);
  //Setting up Button #3
  Serial.print("Setting pin button:");
  Serial.println(btn_3.PIN);
  pinMode(btn_3.PIN, INPUT_PULLUP);
  attachInterrupt(btn_3.PIN, isr_btn3, FALLING);
  //Button #4
  Serial.print("Setting pin button:");
  Serial.println(btn_4.PIN);
  pinMode(btn_4.PIN, INPUT_PULLUP);
  attachInterrupt(btn_4.PIN, isr_btn4, FALLING);
  
  tp.DotStar_SetPixelColor( 255, 128, 0 ); //<===================================================saber que color?
  //-------------------------------------------------------------------testing the setting
  //iniciamos desplegando informacion sobre el chip y la version de firmware. 
  Serial.println(F("")); 
  Serial.println(F("Inicializacion de programa de boton con identificacion RFID;"));
  Serial.println(F("Parametros de ambiente de funcionamiento:"));
  Serial.print(F("            CHIPID: "));
  GloveID();
  Serial.print(F("            HARDWARE: "));
  Serial.println(HardwareVersion);
  Serial.print(F("            FIRMWARE: "));
  Serial.println(FirmwareVersion);
  //si hubiera un servidot de NTP.
  /*
  Serial.print(F("            Servidor de NTP: "));
  Serial.println(ntpServerName);
  */
  Serial.print(F("            Servidor de MQTT: "));
  Serial.println(mqttServer);
  Serial.print(F("            Client ID: "));
  Serial.println(chipId); 
  delay(500);
  //-----------------------------------------------------------------------incializando puertos fisicos
  // SET ENABLE PIN AND PRINT ITS STATE
  Serial.print("Setting Enable PIN to:");
  Serial.println(enable1Pin);
  pinMode (enable1Pin, OUTPUT);
  digitalWrite(enable1Pin, LOW);
  Serial.print("Setting Enable PIN to:");
  EnablePinState = digitalRead(enable1Pin);
  Serial.println(EnablePinState);
  delay(100);

  //SET PWM PIN AND CONFIGURE INITIAL PARAMETERS:
  
  Serial.print("Setting PWM PIN TO:");
  Serial.println(pulgar.motorPin);
  ledcAttachPin(pulgar.motorPin, pulgar.pwmChannel);
  delay(100);
  Serial.println(indice.motorPin);
  ledcAttachPin(indice.motorPin, indice.pwmChannel);
  delay(100);
  Serial.println(medio.motorPin);
  ledcAttachPin(medio.motorPin, medio.pwmChannel);
  delay(100);
  Serial.println(anular.motorPin);
  ledcAttachPin(anular.motorPin, anular.pwmChannel);
  delay(100);
  Serial.println(menique.motorPin);
  ledcAttachPin(menique.motorPin, menique.pwmChannel);
  delay(100);

  // attach the channel to the GPIO to be controlled
  Serial.print("Setting PWM PIN CHANNEL TO:");
  Serial.println(pulgar.pwmChannel);
  Serial.print("Setting PWM PIN CHANNEL TO:");
  Serial.println(indice.pwmChannel);
  Serial.print("Setting PWM PIN CHANNEL TO:");
  Serial.println(medio.pwmChannel);
  Serial.print("Setting PWM PIN CHANNEL TO:");
  Serial.println(anular.pwmChannel);
  Serial.print("Setting PWM PIN CHANNEL TO:");
  Serial.println(menique.pwmChannel);
  Serial.print("Setting PWM PIN FREQ TO:");
  Serial.println(freq);
  Serial.print("Setting PWM PIN RESOLUTION TO:");
  Serial.println(resolution);
  ledcSetup(pulgar.pwmChannel, freq, resolution);
  delay(50); 
  ledcSetup(indice.pwmChannel, freq, resolution);
  delay(50); 
  ledcSetup(medio.pwmChannel, freq, resolution);
  delay(50); 
  ledcSetup(anular.pwmChannel, freq, resolution);
  delay(50); 
  ledcSetup(menique.pwmChannel, freq, resolution);
  delay(50);  
  //SET THE INITIAL DUTY CICLY TO 0 
  Serial.print("Setting PWM Initial dutycycle TO:");
  Serial.println(initDutyCycle);
  ledcWrite(pulgar.pwmChannel, initDutyCycle);
  delay(50); 
  ledcWrite(indice.pwmChannel, initDutyCycle);
  delay(50); 
  ledcWrite(medio.pwmChannel, initDutyCycle);
  delay(50); 
  ledcWrite(anular.pwmChannel, initDutyCycle);
  delay(50);
  ledcWrite(menique.pwmChannel, initDutyCycle);
  delay(50);
    
  //-----------------------------------------------------------------------Conecting to WIFI
  Serial.println("Attempting to connect...");
  WiFi.begin(ssid, password); // Connect to WiFi.
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Couldn't connect to WiFi.");
      tp.DotStar_SetPixelColor( 255, 255, 255);
      while(1) delay(100);
  }
   
  
    
  lastReconnectAttempt = 0;
  Serial.println(F("Finalizing Setup"));                                                                  //enviamos un mensaje de depuracion
  configuration_json();
  tp.DotStar_Clear();
  fsm_state = STATE_IDLE; //inciar el estado del la maquina de stado finito
  // put your setup code here, to run once:
}

//----------------------------------------------------------------------------------------------------//Fundiones de Motores



void motorPWM_0(int motor_channel){
  Serial.print("PWM  to 0 for Motor channel:");
  Serial.println(motor_channel);
  enableMotors(enable1Pin);
  ledcWrite(motor_channel, initDutyCycle);
  delay(100);
  breakMotors(enable1Pin);
}

void findemensaje(){
  enableMotors(enable1Pin);
  ledcWrite(indice.pwmChannel, dutyCycleHalf);
  ledcWrite(medio.pwmChannel, dutyCycleHalf);
  ledcWrite(anular.pwmChannel, dutyCycleHalf);
  ledcWrite(menique.pwmChannel, dutyCycleHalf);
  delay(100);
  breakMotors(enable1Pin);
  delay(50);
  enableMotors(enable1Pin);
  ledcWrite(indice.pwmChannel, dutyCycleHalf);
  ledcWrite(medio.pwmChannel, dutyCycleHalf);
  ledcWrite(anular.pwmChannel, dutyCycleHalf);
  ledcWrite(menique.pwmChannel, dutyCycleHalf);
  breakMotors(enable1Pin);  
}

//----------------------------------------------------------------------------------------------------------------------------------------//Funciones de MQTT
void check_for_connection (){
  if (!client.connected()) {
    Serial.println(F("reconectando"));
     tp.DotStar_SetPixelColor( 239, 250, 0);
    long now = millis();
    if (now - lastReconnectAttempt > 5000) { // Try to reconnect.
      lastReconnectAttempt = now;
      if (reconnect()) { // Attempt to reconnect.
        lastReconnectAttempt = 0;
      }
    }
  }else { // Connected.
    client.loop();
  }
}
//----------------------------------------------------------------------------------------------------------------------------------------//funcion para traducir mensaje a vibraciones.
//void traducir_a_vibraciones(int pwm_motor_channel, int pwm_motor_time_ms, int pwm_motor_effect){
  //Serial.println(F("DEBUG: empezandotraduccion a vibraciones"));
  //if(pwm_motor_effect = 1){
    //Serial.println(F("DEBUG: el mensaje contenia un rampup"));
   // rampUp(pwm_motor_channel);
  //}
  //if(pwm_motor_effect = 2){
    //Serial.println(F("DEBUG: el mensaje contenia un rampDown"));
   // rampDown(pwm_motor_channel);
  //}
  //if(pwm_motor_effect = 3){
  //  Serial.println(F("DEBUG: el mensaje contenia un motorClick"));
    //motorClick(pwm_motor_channel, pwm_motor_time_ms);
  //}
  //else if (pwm_motor_effect = 4){
    //Serial.println(F("DEBUG: eso era toda la pregunta find de mensaje"));
   // findemensaje();
 // }
  
  //Serial.println(F("DEBUG: Cambiando de estado de pregunta a respuesta"));
  //fsm_state = STATE_TRANSMIT_RESPUESTA;
//}
void funciona_botones(){
 Serial.println(F("btn1:"));
   Serial.println(btn_1.numberKeyPresses);
Serial.println(F("__---------------------------------"));
Serial.println(F("btn2:"));
   Serial.println(btn_2.numberKeyPresses);
   Serial.println(F("__---------------------------------"));
Serial.println(F("btn3:"));
   Serial.println(btn_3.numberKeyPresses);
   Serial.println(F("__---------------------------------"));
Serial.println(F("btn4:"));
   Serial.println(btn_4.numberKeyPresses); 
 Serial.println(F("__---------------------------------"));
 delay(10000);
    String repuesta=String("BTN_1  ")+btn_1.numberKeyPresses+String("BTN_2  ")+btn_2.numberKeyPresses+String("BTN_3  ")+btn_3.numberKeyPresses+String("BTN_4  ")+btn_4.numberKeyPresses;
    
       Serial.println(repuesta);
}
void milis_prueba(){
 unsigned long t_inicio=0;
  unsigned long t_inicio2=0; 
  unsigned long res;
  
 if (millis() - last_Case_Status_Millis > 35000){
          last_Case_Status_Millis = millis();
          res=millis()-  last_Case_Status_Millis;
          Serial.println(res);
          Serial.print(F("respuesta enviada "));
           fsm_state = STATE_TRANSMIT_RESPUESTA;
         }
  //t_inicio=current_millis;
  //Serial.println(t_inicio);
  //res= current_millis-t_inicio;
  //Serial.println(F("respuesta de milis"));
 // Serial.println(res);

}
void capturar_respuesta_de_botones(){
   Serial.println(F("signo igual ahora:"));
  delay(15000);
    repuesta=String("BTN_1")+btn_1.numberKeyPresses+String("BTN_2")+btn_2.numberKeyPresses+String("BTN_3")+btn_3.numberKeyPresses+String("BTN_4")+btn_4.numberKeyPresses;
    Serial.println(F("respuesta de una"));
  fsm_state = STATE_TRANSMIT_RESPUESTA;

}

void publicar_la_respuesta_a_servidor(int idoperacion, int idguante, String answer){
  //String response = ("{    "idOperation":12,    "gloveCode":"1941238-1458400",    "answer":2 })";
  const int capacity = JSON_OBJECT_SIZE(250);
  StaticJsonDocument<capacity> edin_json_response_doc;
  // create an object
  JsonObject object = edin_json_response_doc.to<JsonObject>();
  answer=repuesta;
  Serial.println(repuesta); 
  object["idOperation"]   = idoperacion;
  object["gloveCode"]     = idguante;
  object["answer"]        = repuesta;
  Serial.println(repuesta);
      
  String output;
  size_t n = serializeJson(object, output);                                                                  //SAve CPU cycles by calculatinf the size.

  Serial.println(F("publishing device response to server:"));
  Serial.println(output);
  Serial.println(repuesta);
  

  if (!client.connected()) {
    check_for_connection();
  }
  if (client.publish(responseTopic, output.c_str(), n)) {
    Serial.println(F("device Publish ok"));
  }else {
    Serial.println(F("device Publish failed:"));
    tp.DotStar_SetPixelColor( 255, 255, 255);
  }
}

//************************************************************************************************************************************* SETUP ***************************************************************************************
void loop() {
  bateria_estado();
  switch(fsm_state){                                                                                         //Iniciamos el switch case
    
    case STATE_IDLE:                                                                                        //Que debe hacer la maquina cuando esta en estado de IDLE
         
         if (millis() - last_Case_Status_Millis > intervalo_Case_Status_Millis){
          last_Case_Status_Millis = millis();
          Serial.print(F("fsm_state: "));
          Serial.println(fsm_state);
          Serial.println(F(""));
          Serial.print(F("message_received_status: "));
          Serial.println(mesgrecv);
          Serial.println(F(""));
         }
         
        
        
        check_for_connection();
               
    break;
    
    case STATE_PREGUNTA:
         Serial.println(F("Switch case state Pregunta: PREGUNTA"));
         delay(500);
          btn_1.numberKeyPresses=0;
   btn_2.numberKeyPresses=0;
    btn_3.numberKeyPresses=0;
     btn_4.numberKeyPresses=0;
     
     // funciona_botones();
     capturar_respuesta_de_botones(); 
      // milis_prueba();
                           
    break;
    
    case STATE_TRANSMIT_RESPUESTA:
        Serial.println(F("Switch case state: RESPUESTA"));
        //insertar a qui un While
        publicar_la_respuesta_a_servidor(idOperation, gloveCode, repuesta);
                
        fsm_state = STATE_IDLE;
    break;
    
    default:
        fsm_state = STATE_IDLE;
    break;
  }
}
