#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" 
#include "soc/rtc_cntl_reg.h"  
#include "esp_http_server.h"
#include <BluetoothSerial.h>


//  ============================================================
//  Configurações do Bluetooth
//  ============================================================
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
char valor_recebido;
int valor_giro;


//  ============================================================
//  Configurações do WiFi
//  ============================================================
// const char* ssid     = "Vivo-Internet-ED42";
// const char* password = "32425121";
const char* ssid     = "2G_CLARO_IGOR";
const char* password = "12121998";


//  ============================================================
//  Configurações do Sensor Ultrassônico
//  ============================================================
const int pin_trigger = 12; // 5
const int pin_echo = 14; // 18
#define SOUND_SPEED 0.034
long duration;
float distanciaCm;
int distanciaminima = 10;

//  ============================================================
//  Configurações do Buzzer
//  ============================================================
const int pin_buzzer = 27; // 32

//  ============================================================
//  Configurações dos Motores
//  ============================================================
const int pin_motor_4 = 25; // 14
const int pin_motor_3 = 33; // 27
const int pin_motor_2 = 32; // 26
const int pin_motor_1 = 35; // 25
const int pin_motor_velocidade_direita = 34; // 12
const int pin_motor_velocidade_esquerda = 33; // 33

//  Função de movimento
void movement(char move, int giro)
{
  //  Curva
  int vel_esquerda;
  int vel_direita;

  if(giro < 0){
    giro = giro * (-1);
    vel_esquerda = 80 + (giro * 15);
    vel_direita = 230;
  }
  else if(giro > 0){
    vel_esquerda = 230;
    vel_direita = 80 + (giro * 15);
  }
  else{
    vel_esquerda = 230;
    vel_direita = 230;
  }

  //  Frente
  if(move == 'w'){
    digitalWrite(pin_motor_1, HIGH);
    digitalWrite(pin_motor_2, LOW);
    digitalWrite(pin_motor_3, LOW);
    digitalWrite(pin_motor_4, HIGH);
    analogWrite(pin_motor_velocidade_direita,vel_direita);
    analogWrite(pin_motor_velocidade_esquerda,vel_esquerda);
  }
  //  Direita
  else if(move == 'd'){
    digitalWrite(pin_motor_1, LOW);
    digitalWrite(pin_motor_2, HIGH);
    digitalWrite(pin_motor_3, LOW);
    digitalWrite(pin_motor_4, HIGH);
  }
  //  Esquerda
  else if(move == 'a'){
    digitalWrite(pin_motor_1, HIGH);
    digitalWrite(pin_motor_2, LOW);
    digitalWrite(pin_motor_3, HIGH);
    digitalWrite(pin_motor_4, LOW);
  }
  //  Ré
  else if(move == 's'){
    digitalWrite(pin_motor_1, LOW);
    digitalWrite(pin_motor_2, HIGH);
    digitalWrite(pin_motor_3, HIGH);
    digitalWrite(pin_motor_4, LOW);
    // analogWrite(pin_motor_velocidade_direita,0);
    // analogWrite(pin_motor_velocidade_esquerda,80);
    // analogWrite(pin_motor_velocidade_esquerda,255);
    analogWrite(pin_motor_velocidade_direita,vel_direita);
    analogWrite(pin_motor_velocidade_esquerda,vel_esquerda);
  }
  //  Parada
  else if(move == 'p'){
    digitalWrite(pin_motor_1, LOW);
    digitalWrite(pin_motor_2, LOW);
    digitalWrite(pin_motor_3, LOW);
    digitalWrite(pin_motor_4, LOW);
  }
}

//  ============================================================
//  Setup
//  ============================================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(1000);
  Serial.println("Disciplina LIEC: Projeto MAREA");
  SerialBT.begin("Projeto MAREA");
  delay(1000);

  //  Sensor Ultrassonico
  pinMode(pin_trigger, OUTPUT);
  pinMode(pin_echo, INPUT);

  //  Buzzer
  pinMode(pin_buzzer, OUTPUT);

  //  Motores
  pinMode(pin_motor_1, OUTPUT);
  pinMode(pin_motor_2, OUTPUT);
  pinMode(pin_motor_3, OUTPUT);
  pinMode(pin_motor_4, OUTPUT);
  pinMode(pin_motor_velocidade_direita, OUTPUT);
  pinMode(pin_motor_velocidade_esquerda, OUTPUT);
}

//  ============================================================
//  Loop
//  ============================================================
void loop() {
  if (SerialBT.available() > 0){
  // if(begin == 1){
    char letter_move = 's';
    valor_recebido =(char)SerialBT.read();
    valor_giro =(int)SerialBT.read();

    //  Le as informações em cm do sensor ultrassônico
    digitalWrite(pin_trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(pin_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_trigger, LOW);
    duration = pulseIn(pin_echo, HIGH);
    distanciaCm = duration * SOUND_SPEED/2;

    //  Impressão de resultados 
    Serial.print("Distancia em cm: ");
    Serial.println(distanciaCm);
    // Serial.println(valor_recebido);

    //  Movimento Botões
    // movement(letter_move, 8);
    if(valor_recebido == 'A'){
      letter_move = 'w';
      movement(letter_move, valor_giro);
    }
    else if(valor_recebido == 'Z'){
      letter_move = 'p';
      movement(letter_move, valor_giro);
    }
    else if(valor_recebido == 'R'){
      letter_move = 's';
      movement(letter_move, valor_giro);
    }
  }
}

