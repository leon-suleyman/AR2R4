#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>


/* ---- Pines motores (PWM) ---- */
#define PIN_MOTOR_IZQ_A 5
#define PIN_MOTOR_IZQ_B 6
#define PIN_MOTOR_DER_A 10
#define PIN_MOTOR_DER_B 11

/* ---- Pines encoders (interrupciones UNO) ---- */
#define PIN_ENCODER_DER 2  // INT0
#define PIN_ENCODER_IZQ 3   // INT1

/* ---- Pines sonares ---- */
#define T_SONAR_F 9
#define E_SONAR_F 8
#define T_SONAR_I 7
#define E_SONAR_I 4
#define T_SONAR_D 12
#define E_SONAR_D 13

/* ---- LEDs delanteros ---- */
#define LED_T A3
#define LED_IZQ A4
#define LED_DER A5

/* ---- Constantes sonar / comportamiento ---- */
const float DISTANCIA_SIN_ECO = 10000.0;
const float DIST_LLEGO_CM     = 5.0;    // <= 5 cm: llegó
const float DIST_MAX_DET_CM   = 50.0;   // si está más lejos: “no detecta”
const unsigned long BLINK_MS  = 200;    // parpadeo LEDs mientras se acerca

/* ---- PID / control ---- */
#define PWM_MAX 255
#define PWM_MIN_DER 30
#define PWM_MIN_IZQ 30
#define P 40.0
#define I 20.0
#define D 4.0
#define TIEMPO_CONTROL 50UL  // ms
#define VEL_BAJA 0.5
#define VEL_MEDIA 0.7
#define VEL_ALTA 0.9

static const int RANURAS_POR_VUELTA = 20; // 20 ticks = 1 vuelta

/*Medidas del coche (en metros)*/
#define DISTANCIA_ENTRE_RUEDAS 0.204
#define RADIO_DE_LAS_RUEDAS 0.0325

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

#if defined(LED_BUILTIN)
  #define LED_PIN LED_BUILTIN
#else
  #define LED_PIN 13
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){digitalWrite(LED_T, HIGH); delay(500); digitalWrite(LED_T, LOW);}}

/* Constantes de configuración */
float acumuladas_frontales[3] = {0,0,0};
int indice_frontales = 0;
float acumuladas_izquierdas[3] = {0,0,0};
int indice_izquierdas = 0;
float acumuladas_derechas[3] = {0,0,0};
int indice_derechas = 0;

/* ---- Estados y sonares ---- */
enum SONARES { SONAR_FRONTAL, SONAR_IZQ, SONAR_DER };
enum ESTADOS {AVANZAR, GIRAR_DER, GIRAR_IZQ, RETROCEDER};

ESTADOS estado = AVANZAR;

/* ---- Encoder ticks (ISR) ---- */
volatile long ticks_derecha = 0;
volatile long ticks_izquierda = 0;

/* Dirección recordada para dar signo a ticks
   Convención del robot (como tu actividad 5):
   - PWM NEGATIVO = ADELANTE
   - PWM POSITIVO = ATRÁS
   Para que la velocidad real sea POSITIVA cuando avanza,
   guardamos dir = +1 cuando PWM es negativo (adelante). */
volatile int8_t dir_der = +1; // +1 = adelante, -1 = atrás
volatile int8_t dir_izq = +1;

/* PID por rueda */
double integral_I_der = 0.0;
double integral_I_izq = 0.0;
double error_anterior_der = 0.0;
double error_anterior_izq = 0.0;

unsigned long tiempo_desde_control = 0;

/*variables de odometria*/
double posicion_x = 0.0;
double posicion_y = 0.0;
double posicion_theta = 0.0;

double velocidad_actual_lineal = 0.0;
double velocidad_actual_angular = 0.0;

/* Parpadeo LEDs */
unsigned long lastBlink = 0;
bool blinkOn = false;

/* Setpoints (velocidad deseada en grados/ms, SIEMPRE >= 0 para avanzar) */
double vel_deseada_der = 0.0;
double vel_deseada_izq = 0.0;

/* variables de micro-ros, publicadores y subscriptores */
String ros_namespace = "AR2R4";

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t publisher_sonar_frontal;
std_msgs__msg__Float32 msg_sonar_frontal;
rcl_publisher_t publisher_sonar_derecho;
std_msgs__msg__Float32 msg_sonar_derecho;
rcl_publisher_t publisher_sonar_izquierdo;
std_msgs__msg__Float32 msg_sonar_izquierdo;

rcl_publisher_t publisher_velocidad_derecha;
std_msgs__msg__Float32 msg_velocidad_derecha;
rcl_publisher_t publisher_velocidad_izquierda;
std_msgs__msg__Float32 msg_velocidad_izquierda;

/*
rcl_publisher_t publisher_IR_derecho;
std_msgs__msg__Float32 msg_IR_derecho;
rcl_publisher_t publisher_IR_izquierdo;
std_msgs__msg__Float32 msg_IR_izquierdo;
*/

rcl_publisher_t publisher_odometria;
geometry_msgs__msg__Twist msg_odometria;

/* Prototipos */
void tick_encoder_derecha();
void tick_encoder_izquierda();

float sonar(int trigger, int echo);
float medir_sonar(int trigger, int echo, float acumuladas[], int &indice_acumuladas, rcl_publisher_t &publisher, std_msgs__msg__Float32 &msg);
float revisarSonares(int id_sonar);
bool detectaObjetivo(float d_cm);

void setup_micro_ros();

double calcular_velocidad(long ticks, unsigned long tiempo_ms);
void control_PID(const unsigned long dt_ms);
int ciclo_control_PID_mag(double vel_deseada, double vel_real,
                          double& integral_I, double& error_previo);

void aplicar_PWM(int pwm_izq, int pwm_der);
void aplicar_PWM_a_motor(int pwm, uint8_t pin_A, uint8_t pin_B);

void cambiar_estado(int sonarActivo, float distActiva);

void calcular_posicion_actual(const unsigned long dt_ms);

void error_loop(){
  while(1){
    digitalWrite(LED_T, !digitalRead(LED_T));
    digitalWrite(LED_DER, !digitalRead(LED_DER));
    digitalWrite(LED_IZQ, !digitalRead(LED_IZQ));
    delay(100);
  }
}

/*
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}
*/

int tiempo_giro = 0;

/* =========================
   SETUP
   ========================= */
void setup() {
  pinMode(PIN_MOTOR_IZQ_A, OUTPUT);
  pinMode(PIN_MOTOR_IZQ_B, OUTPUT);
  pinMode(PIN_MOTOR_DER_A, OUTPUT);
  pinMode(PIN_MOTOR_DER_B, OUTPUT);

  pinMode(T_SONAR_I, OUTPUT); pinMode(E_SONAR_I, INPUT);
  pinMode(T_SONAR_F, OUTPUT); pinMode(E_SONAR_F, INPUT);
  pinMode(T_SONAR_D, OUTPUT); pinMode(E_SONAR_D, INPUT);

  pinMode(LED_T, OUTPUT);
  pinMode(LED_IZQ, OUTPUT);
  pinMode(LED_DER, OUTPUT);

  pinMode(PIN_ENCODER_DER, INPUT);
  pinMode(PIN_ENCODER_IZQ, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_DER), tick_encoder_derecha, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_IZQ), tick_encoder_izquierda, RISING);

  //Serial.begin(9600);
  digitalWrite(LED_T, HIGH);
  delay(500);
  digitalWrite(LED_T, LOW);
  setup_micro_ros();
  
  aplicar_PWM(0, 0);
  tiempo_desde_control = millis();
  vel_deseada_der = -VEL_MEDIA;
  vel_deseada_izq = VEL_MEDIA;
  estado = GIRAR_IZQ;
  //aplicar_PWM(0,53);
}

/* =========================
   LOOP
   ========================= */
void loop() {

  //revisamos nuestros sonares y vemos si tenemos objetos cercanos 
  float sonar_frontal = revisarSonares(SONAR_FRONTAL);
  float sonar_izquierdo = revisarSonares(SONAR_IZQ);
  float sonar_derecho = revisarSonares(SONAR_DER);
  //Serial.print("frontal: "); Serial.print(sonar_frontal);Serial.print("izquierdo: "); Serial.print(sonar_izquierdo);Serial.print("derecho: "); Serial.println(sonar_derecho);

  bool obj_frente = hay_algo_cerca(sonar_frontal);
  bool obj_izquierda = hay_algo_cerca(sonar_izquierdo);
  bool obj_derecha = hay_algo_cerca(sonar_derecho);

  
  //en base a que objetos tenemos cercanos, cambiamos nuestro estado
  cambiar_estado(obj_frente, obj_izquierda, obj_derecha);
  long tiempo = millis();
  if (tiempo - tiempo_desde_control >= TIEMPO_CONTROL) {
    long dt_ms = tiempo - tiempo_desde_control;
    tiempo_desde_control = tiempo;
    calcular_posicion_actual(dt_ms);
    control_PID(dt_ms);
  }
}

void setup_micro_ros(){
  //parametros de set_microros_wifi_transorts: ("Nombre de red WIFI a conectar", "Contraseña de red WIFI", "IP de la computadora donde corre el agente de micro-ros donde comunicar", "puerto por el que el agente de micro-ros está conectado")
  set_microros_wifi_transports("EVENTO", "UBA-2022", "10.142.7.135", 8888);

  //Señal de que se conectó bien a la red WiFi
  digitalWrite(LED_T, HIGH);
  delay(500);
  digitalWrite(LED_T, LOW);
  //delay para estabilizar conexión con el agente de micro-ros
  delay(1500);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "AR2R4", "", &support));
  String topic_name = "";

  // create publisher
  topic_name = ros_namespace + "/sonar_frontal";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_sonar_frontal,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
    topic_name.c_str()));

  msg_sonar_frontal.data = 0.0;

  topic_name = ros_namespace + "/sonar_derecho";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_sonar_derecho,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    topic_name.c_str()));

  msg_sonar_derecho.data = 0.0;
  
  topic_name = ros_namespace + "/sonar_izquierdo";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_sonar_izquierdo,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    topic_name.c_str()));

  msg_sonar_izquierdo.data = 0.0;

  topic_name = ros_namespace + "/velocidad_derecha";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_velocidad_derecha,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    topic_name.c_str()));

  msg_velocidad_derecha.data = 0.0;

  topic_name = ros_namespace + "/velocidad_izquierda";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_velocidad_izquierda,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    topic_name.c_str()));

  msg_velocidad_izquierda.data = 0.0;

  /* 
  topic_name = ros_namespace + "/IR_derecho";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IR_derecho,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    topic_name.c_str()));

  msg_IR_derecho.data = 0.0;

 topic_name = ros_namespace + "/IR_izquierdo";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IR_izquierdo,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    topic_name.c_str()));

  msg_IR_izquierdo.data = 0.0;
  */

  topic_name = ros_namespace + "/odometria";
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_odometria,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    topic_name.c_str()));

  msg_odometria.linear.x = 0.0;
  msg_odometria.linear.y = 0.0;
  msg_odometria.linear.z = 0.0;

  msg_odometria.angular.x = 0.0;
  msg_odometria.angular.y = 0.0;
  msg_odometria.angular.z = 0.0;
}

void cambiar_estado(bool objeto_al_frente, bool objeto_a_la_izquierda, bool objeto_a_la_derecha){
  int t = millis();
    //dependiendo de nuestro estado actual, reaccionamos diferente:
    switch(estado){
        case AVANZAR:
            //Si el hay un objeto en frente, cambiamos al estado RETROCEDER y movemos las ruedas para atras.
            if(objeto_al_frente){
                estado = RETROCEDER;
                digitalWrite(LED_T, HIGH);
                vel_deseada_der =   VEL_ALTA;
                vel_deseada_izq =   VEL_BAJA;
                integral_I_der = 0.0;
                integral_I_izq = 0.0;
            }else{
                //Si no hay un objeto en frente, revisamos si hay un objeto a la izquierda o derecha y de haberlo giramos al lado correspondiente.
                if(objeto_a_la_derecha && !objeto_a_la_izquierda){
                    estado = GIRAR_IZQ;
                    digitalWrite(LED_IZQ, HIGH);
                    vel_deseada_der =  -VEL_BAJA;
                    vel_deseada_izq =   VEL_BAJA;
                    integral_I_izq = 0.0;
                }
                if(!objeto_a_la_derecha && objeto_a_la_izquierda){
                    estado = GIRAR_DER;
                    digitalWrite(LED_DER, HIGH);
                    vel_deseada_der =   VEL_BAJA;
                    vel_deseada_izq =  -VEL_BAJA;
                    integral_I_der = 0.0;
                }
            }
            break;

        case GIRAR_IZQ:
            //si ya no hay un objeto a la derecha o hay un objeto a ambos lado, avanzamos
            /*
            if(!objeto_a_la_derecha || (objeto_a_la_derecha && objeto_a_la_izquierda)){
                estado = AVANZAR;
                digitalWrite(LED_IZQ, LOW);
                vel_deseada_der =  -VEL_MEDIA;
                vel_deseada_izq =  -VEL_MEDIA;
                integral_I_izq = 0.0;
            }
            */
            if(t - tiempo_giro > 5000){
              tiempo_giro = t;
              estado = GIRAR_DER;
              digitalWrite(LED_IZQ, LOW);
              digitalWrite(LED_DER, HIGH);
              vel_deseada_der =  VEL_MEDIA;
              vel_deseada_izq =  -VEL_MEDIA;
            }
            break;

        case GIRAR_DER:
            //si ya no hay un objeto a la izquierda o hay un objeto a ambos lados, avanzamos
            /*
            if(!objeto_a_la_izquierda || (objeto_a_la_izquierda && objeto_a_la_derecha)){
                estado = AVANZAR;
                digitalWrite(LED_DER, LOW);
                vel_deseada_der =  -VEL_MEDIA;
                vel_deseada_izq =  -VEL_MEDIA;
                integral_I_der = 0.0;
            }
            */
            if(t - tiempo_giro > 5000){
              tiempo_giro = t;
              estado = GIRAR_IZQ;
              digitalWrite(LED_DER, LOW);
              digitalWrite(LED_IZQ, HIGH);
              vel_deseada_der =  -VEL_MEDIA;
              vel_deseada_izq =  VEL_MEDIA;
            }
            break;

        case RETROCEDER:
            //si ya no hay un objeto al frente, giramos a la izquierda por 0,2 segundos y luego cambiamos a avanzar
            if(!objeto_al_frente){
                estado = AVANZAR;
                digitalWrite(LED_T, LOW);
                vel_deseada_der =  -VEL_MEDIA;
                vel_deseada_izq =  -VEL_MEDIA;
                integral_I_der = 0.0;
                integral_I_izq = 0.0;
            }
            break;
    }
    //Serial.print("Estado actual: "); Serial.println(print_estado(estado));
    return;
}

String print_estado(ESTADOS est){
  String res = "";
  switch(est){
    case AVANZAR:
      res = "AVANZAR";
      break;
    case GIRAR_IZQ:
      res = "GIRAR_IZQ";
      break;
    case GIRAR_DER:
      res = "GIRAR_DER";
      break;
    case RETROCEDER:
      res = "RETROCEDER";
      break;
  }
  return res;
}

/* Odometria */
void calcular_posicion_actual(long dt_ms){
  double delta_izq = M_PI * 2 * RADIO_DE_LAS_RUEDAS * ticks_izquierda / RANURAS_POR_VUELTA;
  double delta_der = M_PI * 2 * RADIO_DE_LAS_RUEDAS * ticks_derecha / RANURAS_POR_VUELTA;

  double delta_theta = (delta_der - delta_izq) / DISTANCIA_ENTRE_RUEDAS;
  double delta_distancia = (delta_izq + delta_der) / 2;

  double delta_x = delta_distancia * cos( posicion_theta );
  double delta_y = delta_distancia * sin( posicion_theta );

  double delta_t = (double) dt_ms;

  posicion_x += delta_x;
  posicion_y += delta_y;
  posicion_theta += delta_theta;
  posicion_theta = posicion_theta - (360 * floor(posicion_theta/360.0));

  velocidad_actual_lineal = delta_distancia / delta_t;
  velocidad_actual_angular = posicion_theta / delta_t;

  msg_odometria.linear.x = posicion_x;
  msg_odometria.linear.y = posicion_y;

  msg_odometria.angular.z = posicion_theta;

  RCSOFTCHECK(rcl_publish(&publisher_odometria, &msg_odometria, NULL));
}

/* =========================
   PID (devuelve magnitud 0..255)
   ========================= */

void control_PID(long dt_ms){
  // Copia atómica de ticks (evita que cambien en medio de la lectura)
  long ticks_der_local, ticks_izq_local;
  noInterrupts();
  ticks_der_local = ticks_derecha;
  ticks_izq_local = ticks_izquierda;
  ticks_derecha = 0;
  ticks_izquierda = 0;
  interrupts();

  // Velocidad actual (con signo) en grados/ms
  const double vel_actual_der = calcular_velocidad(ticks_der_local, dt_ms, dir_der);
  msg_velocidad_derecha.data = vel_actual_der;
  const double vel_actual_izq = calcular_velocidad(ticks_izq_local, dt_ms, dir_izq);
  msg_velocidad_izquierda.data = vel_actual_izq;

  RCSOFTCHECK(rcl_publish(&publisher_velocidad_derecha, &msg_velocidad_derecha, NULL));
  RCSOFTCHECK(rcl_publish(&publisher_velocidad_izquierda, &msg_velocidad_izquierda, NULL));
  // PID por rueda -> PWM con signo
  int pwm_der = ciclo_control_PID(vel_deseada_der, vel_actual_der, integral_I_der, error_anterior_der, PWM_MIN_DER);
  int pwm_izq = ciclo_control_PID(vel_deseada_izq, vel_actual_izq, integral_I_izq, error_anterior_izq, PWM_MIN_IZQ);

  // Recordar dirección por software para que el encoder tenga signo
  // Convención: PWM > 0 => adelante, PWM < 0 => atrás
  noInterrupts();
  dir_der = (pwm_der >= 0) ? +1 : -1;
  dir_izq = (pwm_izq >= 0) ? +1 : -1;
  interrupts();

  aplicar_PWM(pwm_izq, pwm_der);
  /*
  Serial.print("dt(ms)="); Serial.print(dt_ms);
  Serial.print(" | vel_izq="); Serial.print(vel_actual_izq, 4);
  Serial.print(" | vel_der="); Serial.print(vel_actual_der, 4);
  Serial.print(" | pwm_izq="); Serial.print(pwm_izq);
  Serial.print(" | pwm_der="); Serial.println(pwm_der);
  Serial.print(" | encoder_izq="); Serial.print(ticks_izq_local);
  Serial.print(" | encoder_der="); Serial.println(ticks_der_local);
  */
  // Debug útil

  return;
}


double calcular_velocidad(long ticks, unsigned long tiempo_ms, int direccion) {
  if (tiempo_ms == 0) return 0.0;

  // ticks/ms (con signo)
  const double ticks_por_ms = (double)ticks / (double)tiempo_ms;

  // ticks -> grados: (ticks / ranuras_por_vuelta) * 360
  const double grados_por_ms = ticks_por_ms * 360.0 / (double)RANURAS_POR_VUELTA;
  return grados_por_ms * direccion;
}

int ciclo_control_PID(double vel_deseada, double vel_real, double& integral_I, double& error_previo, int pwm_min) {
  const double error_actual = vel_deseada - vel_real;

  // P
  const double termino_P = P * error_actual;

  // I (anti-windup por saturación del acumulador)
  integral_I += error_actual;
  const double LIM_I = (double)PWM_MAX / (I > 0.0 ? I : 1.0); // límite razonable para integral
  if (integral_I > LIM_I) integral_I = LIM_I;
  if (integral_I < -LIM_I) integral_I = -LIM_I;
  const double termino_I = I * integral_I;

  // D
  const double termino_D = D * (error_actual - error_previo);
  error_previo = error_actual;

  // Salida PID
  double pwm_pid = termino_P + termino_I + termino_D;

  // Saturación a rango PWM
  if (pwm_pid > (double)PWM_MAX) pwm_pid = (double)PWM_MAX;
  if (pwm_pid < -(double)PWM_MAX) pwm_pid = -(double)PWM_MAX;

  // "Deadzone" opcional: PWM_MIN (aquí PWM_MIN=0, pero lo dejamos por claridad)
  
  const int signo = (pwm_pid > 0.0) ? +1 : ((pwm_pid < 0.0) ? -1 : 0);
  int pwm_out = (pwm_pid < 1 && pwm_pid > -1) ? 0 : (int)pwm_pid + (pwm_min * signo);


  if (pwm_out > PWM_MAX) pwm_out = PWM_MAX;
  if (pwm_out < -PWM_MAX) pwm_out = -PWM_MAX;

  return pwm_out;
}

void tick_encoder_derecha() {
  // Sumar ticks con signo según dirección recordada
  ticks_derecha ++;
}

void tick_encoder_izquierda() {
  ticks_izquierda ++;
}

/* =========================
   Motores (PWM con signo)
   NEGATIVO = adelante, POSITIVO = atrás (como tu actividad 5)
   ========================= */
void aplicar_PWM(int pwm_izq, int pwm_der) {
  //Serial.print("Izquierda: "); Serial.print(pwm_izq); Serial.print(" Derercha: "); Serial.println(pwm_der);
  aplicar_PWM_a_motor(pwm_izq, PIN_MOTOR_IZQ_A, PIN_MOTOR_IZQ_B);
  aplicar_PWM_a_motor(pwm_der, PIN_MOTOR_DER_A, PIN_MOTOR_DER_B);
}

void aplicar_PWM_a_motor(int pwm, uint8_t pin_A, uint8_t pin_B) {
  if (pwm > PWM_MAX) pwm = PWM_MAX;
  if (pwm < -PWM_MAX) pwm = -PWM_MAX;

  int pwm_A = 0, pwm_B = 0;

  if (pwm > 0) {         // atrás
    pwm_A = pwm;
    pwm_B = 0;
  } else if (pwm < 0) {  // adelante
    pwm_A = 0;
    pwm_B = -pwm;
  }

  analogWrite(pin_A, pwm_A);
  analogWrite(pin_B, pwm_B);
}


float sonar(int trigger, int echo) {
  float pulse_duration;
  /* silenciamos para que no haya interferecia */
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
  /* mandamos la señal */
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  /* esperamos a escuchar el éco, cuando llega medimos cuanto dura*/
  pulse_duration = pulseIn(echo, HIGH, 14550);
  /* devovlemos la medición de distancia en centimetros */

  if (pulse_duration == 0) return DISTANCIA_SIN_ECO;
  else return pulse_duration / 58.2;
  
}

float medir_sonar(int trigger, int echo, float acumuladas[], int &indice_acumuladas, rcl_publisher_t &publisher, std_msgs__msg__Float32 &msg){
  float lectura_sonar = sonar(trigger, echo);
  acumuladas[indice_acumuladas] = lectura_sonar;
  indice_acumuladas = (indice_acumuladas + 1) % 3;
  lectura_sonar = 0;
  for(int i = 0; i < 3; i++){
    lectura_sonar += acumuladas[i];
  }
  lectura_sonar = lectura_sonar / 3;
  msg.data = lectura_sonar;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  return lectura_sonar;
}

float revisarSonares(int id_sonar) {
  float lectura_sonar;
  lectura_sonar = 10000;
  if (id_sonar == SONAR_FRONTAL) {
    lectura_sonar = medir_sonar(T_SONAR_F, E_SONAR_F, acumuladas_frontales, indice_frontales, publisher_sonar_frontal, msg_sonar_frontal);
  } else {
    if (id_sonar == SONAR_DER) {
        lectura_sonar = medir_sonar(T_SONAR_D, E_SONAR_D, acumuladas_derechas, indice_derechas, publisher_sonar_derecho, msg_sonar_derecho);
    } else {
      if (id_sonar == SONAR_IZQ) {
        lectura_sonar = medir_sonar(T_SONAR_I, E_SONAR_I, acumuladas_izquierdas, indice_izquierdas, publisher_sonar_izquierdo, msg_sonar_izquierdo);
        }
    }
  }

  return lectura_sonar;
}

bool hay_algo_cerca(float distancia) {
  return (distancia <= 25);
}