/*
   Software de validación de receptor y motores para versión con STM32F103C8.
   Más información sobre su uso en https://arduproject.es/pcb-drone-software-validacion-basico
*/

#define tiempo_ciclo 5000
#define pin_RC        PA8   // Pin para lectura del mando vía PPM
#define pin_motor1    PB15  // Pin motor 1
#define pin_motor2    PB14  // Pin motor 2
#define pin_motor3    PB13  // Pin motor 3
#define pin_motor4    PB12  // Pin motor 4
#define pin_LED_azul  PA4   // Pin LED azul
#define pin_BOTON     PB9   // Pin BOTÓN

//---------- RECEPTOR RC ----------//
#define numero_canales 8
/*
   Mando_canal[0] = -
   Mando_canal[1] = ROLL
   Mando_canal[2] = PITCH
   Mando_canal[3] = THROTTLE
   Mando_canal[4] = YAW
   Mando_canal[5] = SWD
   Mando_canal[6] = SWC
   Mando_canal[7] = -
*/

uint32_t pulso_instante[numero_canales * 2 + 2], rise_instante_ant;
uint16_t Mando_canal[numero_canales], canal_ant[numero_canales], Mando_Throttle;
uint16_t contador_flaco = 1;

//---------- Tiempos ciclo ----------//
uint32_t tiempo_nuevo_ciclo, tiempo_motores_start, contador_ciclos;

//---------- PWM ----------//
uint16_t ESC1_us, ESC2_us, ESC3_us, ESC4_us;

//=============================================//////////////////// SETUP ////////////////////=============================================
void setup() {
  //---------- COMUNICACION SERIE ----------//
  Serial.begin(115200);   // Para Serial.print()

  //--------- Declaración de pines ---------//
  pinMode(pin_LED_azul, OUTPUT);  // LED Azul
  pinMode(pin_BOTON, INPUT);      // BOTON
  pinMode(pin_motor1, OUTPUT);    // MOTOR 1
  pinMode(pin_motor2, OUTPUT);    // MOTOR 2
  pinMode(pin_motor3, OUTPUT);    // MOTOR 3
  pinMode(pin_motor4, OUTPUT);    // MOTOR 4

  digitalWrite(pin_motor1, LOW);  // MOTOR 1
  digitalWrite(pin_motor2, LOW);  // MOTOR 2
  digitalWrite(pin_motor3, LOW);  // MOTOR 3
  digitalWrite(pin_motor4, LOW);  // MOTOR 4

  // Parpadeo de arranque inicial
  for (int i = 0; i < 10; i++) {
    digitalWrite(pin_LED_azul, HIGH); // LED Azul
    delay(50);
    digitalWrite(pin_LED_azul, LOW);  // LED Azul
    delay(50);
  }

  //---------- INICIAR BOTON ----------//
  attachInterrupt(digitalPinToInterrupt(pin_BOTON), interrupcion_BOTON, CHANGE);

  //---------- INTERRUPCION MANDO RC ----------//
  pinMode(pin_RC, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_RC), button_ISR, CHANGE);
}

// =============================================//////////////////// LOOP ////////////////////=============================================
void loop() {
  while (micros() - tiempo_nuevo_ciclo < tiempo_ciclo);
  tiempo_nuevo_ciclo = micros();

  RECEPTOR_RC(); // Leer mando RC
  CTRL_VUELO();  // Generar señales PWM para los motores

  // Función para hacer parpadear LED azul
  if (contador_ciclos % 20 == 0) {
    if (digitalRead(pin_LED_azul) == LOW) digitalWrite(pin_LED_azul, HIGH);
    else digitalWrite(pin_LED_azul, LOW);
  }

  // Visualizar por Monitor Serie
  Serial.print(Mando_canal[3]);
  Serial.print("\t");
  Serial.println(Mando_Throttle);

  contador_ciclos++;
}

void RECEPTOR_RC() {
  // Más información en https://arduproject.es/ppm-radiocontrol-arduino-stm32/
  // El receptor es de 8 canales (aunque el mando solo tenga 6), y recibimos 8*2+2 (18 flancos).
  // Solo ejecutamos esta parte si hemos recibido todo el 'burst', los 18 flancos con la informacion
  // de todos los canales.
  if (contador_flaco == 18) {
    for (uint8_t i = 1; i <= numero_canales; i++) {
      // De estos 18 flancos, el primero y el último no nos aportan información. Recorremos los demás
      // flancos. Para calcular la lontigud de cada pulso, hacemos la resta del flanco actual, menos el
      // flanco anterior. Al haber guardado el instante (micros()) en el que se da cada flanco, con esta
      // resta calculamos la anchura de cada pulso.
      Mando_canal[i] = pulso_instante[2 * i] - pulso_instante[2 * i - 1];

      // De forma aleatoria el repector envía señales erroneas (ruido). Es necesario filtrar.
      if (i != 5 && canal_ant[i] > 500 && abs(Mando_canal[i] - canal_ant[i]) > 500)Mando_canal[i] = canal_ant[i];
      if (abs(Mando_canal[5] - canal_ant[5]) > 2000)Mando_canal[5] = canal_ant[5];

      canal_ant[i] = Mando_canal[i];
    }
  }

  // Mapeamos las lecturas del mando RC de 1000 a 2000.
  Mando_Throttle  = map(Mando_canal[3], 729, 1600, 1000, 2000);
}

void button_ISR() {
  // Interrupcion (CHANGE) del pin del mando RC para modo PPM. El receptor mantiene el canal el estado HIGH,
  // y lo fuerza a LOW para transmitir los pulsos. Una vez en LOW, transmite los pulsos partiendo de ese estado.
  // https://www.youtube.com/watch?v=wVRcnMpqPZE (minuto 4:56).
  // Por lo tanto, para transmitir 'n' canales, recibiremos n+2 flancos.

  // Para saber cuando el mando RC ha terminado de enviar el 'burst' con la informacion de todos los canales,
  // se mira si entre flancos han pasado más de 2500ms:
  if (micros() - pulso_instante[contador_flaco - 1] > 2500) contador_flaco = 0;
  // Guardamos en esta variable el instante (micros()) en el que se lee un flanco, tanto positivo como negativo:
  pulso_instante[contador_flaco] = micros();
  contador_flaco++;
}

void CTRL_VUELO() {

  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  digitalWrite(pin_motor1, HIGH);  // MOTOR 1
  digitalWrite(pin_motor2, HIGH);  // MOTOR 2
  digitalWrite(pin_motor3, HIGH);  // MOTOR 3
  digitalWrite(pin_motor4, HIGH);  // MOTOR 4
  tiempo_motores_start = micros();

  // La consigna throttle del mando pasa directamente a los motores
  ESC1_us = Mando_Throttle;
  ESC2_us = Mando_Throttle;
  ESC3_us = Mando_Throttle;
  ESC4_us = Mando_Throttle;

  // Cuando se cumpa el tiempo de PWM definido en ESCx_us, se pasa cada señal a 0 (LOW) para terminar el ciclo PWM.
  // Más detalles en https://arduproject.es/control-de-estabilidad-y-pid/
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tiempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW); // MOTOR 1
    if (tiempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW); // MOTOR 2
    if (tiempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW); // MOTOR 3
    if (tiempo_motores_start + ESC4_us <= micros()) digitalWrite(pin_motor4, LOW); // MOTOR 4
  }
}

void interrupcion_BOTON() {
  // Para reducir la complejidad del montaje del drone, he decidido NO incluir un interruptor
  // para cortar la alimentación de batería. De esta forma, para desconectar la batería tenemos que
  // soltar la manualmente el conector XT60, lo que conlleva tener que agarra el drone con las manos
  // y estirar.
  // Si los motores están girando, y tengo que agarrar el drone para soltar el conector XT60, hay RIESGO de
  // cortarnos con las hélices.
  // Para ganar en seguridad, he asignado esta interrupción al botón del PCB, de forma que si lo pulsamos
  // paramos motores y quedamos en un loop sin hacer nada más. Si el SW queda por lo que sea atascado en
  // algún punto, con este botón podemos detener los motores de forma segura.
  digitalWrite(pin_motor1, LOW);  // MOTOR 1 LOW
  digitalWrite(pin_motor2, LOW);  // MOTOR 2 LOW
  digitalWrite(pin_motor3, LOW);  // MOTOR 3 LOW
  digitalWrite(pin_motor4, LOW);  // MOTOR 4 LOW

  // While infinito. Necesario reinicial la eletrónica.
  while (1);
}
