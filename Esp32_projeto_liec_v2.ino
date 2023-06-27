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
#include <analogWrite.h>


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
const char* ssid = "2G_CLARO_IGOR";
const char* password = "12121998";


//  ============================================================
//  Configurações do Sensor Ultrassônico
//  ============================================================
const int pin_trigger = 33;  // 5
const int pin_echo = 32;     // 18
#define SOUND_SPEED 0.034
long duration;
float distanciaCm;
int distanciaminima = 10;

//  ============================================================
//  Configurações do Buzzer
//  ============================================================
const int pin_buzzer = 34;  // 32

//  ============================================================
//  Configurações dos Motores
//  ============================================================
const int pin_motor_4 = 14;                    // 14
const int pin_motor_3 = 27;                    // 27
const int pin_motor_2 = 26;                    // 26
const int pin_motor_1 = 25;                    // 25
const int pin_motor_velocidade_direita = 12;   // 12
const int pin_motor_velocidade_esquerda = 13;  // 33

//  Função de movimento
void movement(char move, int giro) {
  //  Curva
  int vel_esquerda;
  int vel_direita;

  if (giro < 10) {
    vel_esquerda = giro * 51;
    vel_esquerda += 306;
    vel_direita = 255;
  } else if (giro > 10  && giro<16) {
    vel_esquerda = 255;
    vel_direita = giro * 51;
    vel_direita *= -1;
    vel_direita += 816;
  } else if (giro >= 16) {
    vel_esquerda = 255;
    vel_direita = 0;
  } else {
    vel_esquerda = 255;
    vel_direita = 255;
  }

  Serial.print("Velocidade na esquerda: ");
  Serial.println(vel_esquerda);
  Serial.print("Velocidade na direita: ");
  Serial.println(vel_direita);
  Serial.print("Valor de giro: ");
  Serial.println(giro);
  Serial.print("Letter move: ");
  Serial.println(move);
  //  Frente
  if (move == 'w') {
    digitalWrite(pin_motor_1, HIGH);
    digitalWrite(pin_motor_2, LOW);
    digitalWrite(pin_motor_3, LOW);
    digitalWrite(pin_motor_4, HIGH);
    analogWrite(pin_motor_velocidade_direita, vel_direita);
    analogWrite(pin_motor_velocidade_esquerda, vel_esquerda);
  }
  //  Direita
  else if (move == 'd') {
    digitalWrite(pin_motor_1, HIGH);
    digitalWrite(pin_motor_2, HIGH);
    digitalWrite(pin_motor_3, HIGH);
    digitalWrite(pin_motor_4, HIGH);
  }
  //  Esquerda
  else if (move == 'a') {
    digitalWrite(pin_motor_1, LOW);
    digitalWrite(pin_motor_2, LOW);
    digitalWrite(pin_motor_3, HIGH);
    digitalWrite(pin_motor_4, LOW);

  }
  //  Ré
  else if (move == 's') {
    digitalWrite(pin_motor_1, LOW);
    digitalWrite(pin_motor_2, HIGH);
    digitalWrite(pin_motor_3, HIGH);
    digitalWrite(pin_motor_4, LOW);
    // analogWrite(pin_motor_velocidade_direita,0);
    // analogWrite(pin_motor_velocidade_esquerda,80);
    // analogWrite(pin_motor_velocidade_esquerda,255);
    analogWrite(pin_motor_velocidade_direita, vel_direita);
    analogWrite(pin_motor_velocidade_esquerda, vel_esquerda);
  }
  //  Parada
  else if (move == 'p') {
    digitalWrite(pin_motor_1, LOW);
    digitalWrite(pin_motor_2, LOW);
    digitalWrite(pin_motor_3, LOW);
    digitalWrite(pin_motor_4, LOW);
    analogWrite(pin_motor_velocidade_direita, 0);
    analogWrite(pin_motor_velocidade_esquerda, 0);
  }
}

// Função sensor ultrassonico
float distance_sensor() {
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  duration = pulseIn(pin_echo, HIGH);
  float distancia = duration * SOUND_SPEED / 2;
  return distancia;
}
// Função do buzzer
void reverse_sound(float distancia) {
  int valor_som = distancia * -36.428571;
  valor_som += 255;
  analogWrite(pin_buzzer, valor_som);
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
  if (SerialBT.available() > 0) {
    // if(begin == 1){
    distance_sensor();
    reverse_sound(distance_sensor());
    char letter_move = 'p';
    valor_recebido = (char)SerialBT.read();
    valor_giro = (int)SerialBT.read();




    //  Le as informações em cm do sensor ultrassônico

    //  Impressão de resultados

    //  Movimento Botões
    // movement(letter_move, 8);
    if (valor_recebido == 'A') {
      letter_move = 'w';
      Serial.print("Valor recebido: ");
      Serial.println(valor_recebido);
      Serial.print("Valor de giro: ");
      Serial.println(valor_giro);
      while (valor_recebido != 'Z') {
        valor_recebido = (char)SerialBT.read();
        valor_giro = (int)SerialBT.read();
        if (valor_recebido == 'Z') {
          letter_move = 'p';
          movement(letter_move, valor_giro);
        } else if (valor_giro > 0 && valor_giro < 20) {
          valor_recebido = (char)SerialBT.read();
          if (valor_recebido == 'Z') {
            letter_move = 'p';
            movement(letter_move, valor_giro);
          } else {
            movement('w', valor_giro);
          }
        }
      }
    } else if (valor_recebido == 'R' && distance_sensor() > 10) {
      letter_move = 's';
      movement(letter_move, valor_giro);
      while (valor_recebido != 'Z' && distance_sensor() > 10) {
        Serial.print("Distancia: ");
        Serial.println(distance_sensor());
        reverse_sound(distance_sensor());
        valor_giro = (int)SerialBT.read();
        valor_recebido = (char)SerialBT.read();
        if (distance_sensor() < 10 || valor_recebido == 'Z') {
          letter_move = 'p';
          movement(letter_move, valor_giro);
        } else if (valor_giro > 0 && valor_giro < 20 && valor_recebido != 'Z') {
          valor_recebido = (char)SerialBT.read();
          if (distance_sensor() < 10 || valor_recebido == 'Z') {
            letter_move = 'p';
            movement(letter_move, valor_giro);
          }
          movement(letter_move, valor_giro);
        }
      }
    } else if (valor_recebido == 'Z' || distance_sensor() < 10) {
      reverse_sound(distance_sensor());
      Serial.print("Valor recebido: ");
      Serial.println(valor_recebido);
      letter_move = 'p';
      movement(letter_move, valor_giro);
    }
    // else if(valor_recebido == 'D'){
    //   letter_move = 'd';
    //   movement(letter_move, valor_giro);
    // }
    // else if(valor_recebido == 'E'){
    //   letter_move = 'a';
    //   movement(letter_move, valor_giro);
    // }
  }
}
