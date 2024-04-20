#include "HX711.h"

#define calibF -260000.0

/* 
BAUDRATE = 57600
Literal X: Lectura de Barrido
Literal Y: Salir de cualquier modo
Literal r: Empezar 
Literal s: Parar
Literal t: Reiniciar las marcas temporales
Literal n: Cambia la configuración de Throttle del ESC. Uso exclusivo de Sampler
*/

const byte pinData0 = 4;    // Asignación de pines para los HX711
const byte pinData1 = 6;
const byte pinData2 = 8;
const byte pinClk0 = 3;
const byte pinClk1 = 5;
const byte pinClk2 = 7;
const byte PWM = 9;         // Asignación de pines para control de PWM y la sonda de RPM
const byte RPM = 2;


unsigned long rpmTimer = 0;                         // Variable encargada de definir dinamicamente cada cuantos milisegundos se debe enviar
                                                    // un promedio de las muestras de la sonda de RPM
                                                    
const byte numBlades = 2;                           // Número de palas de la helice

byte firstBlade = 0;                                // Flag encargada de comunicar que la sonda ha detectado la primer pala en pasar sobre el sensor

volatile unsigned long rpmTimeOld = 0;              // Variable encargada de guardar la marca temporal en microsegundos del instante en que
                                                    // una pala es detectada por la sonda de RPM
                                                    
volatile unsigned long rpmTimeDelta[200] = {0};     // Vector encargado de guardar los periodos de tiempo entre cada detección de la sonda

volatile byte rpmState = HIGH;                      // Variable que guarda el estado actual de la sonda de RPM (si detecto una pala o no)
                                                    // por default, HIGH = Nada, LOW = Pala detectada
                                                    
volatile int timeIndex = 0;                         // Variable encargada de recordar cuantos periodos del vector rpmTimeDelta han sido medidos
                                                    // antes de enviar un promedio de estos

union period {            // Creación de un nuevo tipo de variable que permite crear variables cuyos valores pueden ser asignados en formato byte o long
  unsigned long t=0;
  byte b[4];
};


HX711 loadCell;          // Asignación de las celdas a la clase HX711
HX711 loadCellM1;
HX711 loadCellM2;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(RPM, INPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(57600);
  Serial.setTimeout(1);
  Serial.print("[HX711: OK]");
  
  loadCell.begin(pinData0, pinClk0);
  loadCellM1.begin(pinData1, pinClk1);
  loadCellM2.begin(pinData2, pinClk2);

  loadCell.set_scale(calibF);
  loadCellM1.set_scale(calibF);
  loadCellM2.set_scale(calibF);

  loadCell.tare();
  loadCellM1.tare();
  loadCellM2.tare();
  
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);              // Configuración de TIMER1 para servir de señal PWM controladora del ESC. Ver Anexo A
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12) | _BV(WGM13);
  ICR1H = B00010011;
  ICR1L = B10000111;

  attachInterrupt(0, pin_ISR, CHANGE);                                  // Creación de un vector de interrupción activado cuando la sonda detecta una pala. La condición para
                                                                        // activar el ISR es un cambio de estado en el pin designado.
                                                                        
  memset(rpmTimeDelta, 1000000*(60/numBlades), sizeof(rpmTimeDelta));   // Asigna a todas las entradas del vector rpmTimeDelta el valor de periodo temporal
                                                                        // correspondiente a una frecuencia de 1 Hz o 60 rpm. Como valor por default
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {       // Bucle de operación principal

  if (Serial.available() > 0) {     // Inicia la operación solo si el puerto COM se encuentra disponible para operar
    
    delay(4);
    unsigned long tp=0;   // Variable encargada de guardar la marca temporal del instante en el que inició un ciclo de muestreo
    unsigned long tk=0;   // Variable encargada de guardar la marca temporal (relativa al inicio del ciclo) 
                          // del instante en el que se pausó, mas no canceló, el ciclo de muestreo
                          
    byte i=1;             // Flag encargada de comunicar si el programa esta en pausa o se reinicio la marca temporal (1 = en pausa, 2 = en ejecución) 
                          // * El microcontrolador continua leyendo datos de los sensores independientemente de si se le ha ordenado pausar.
                          // * Unicamente la exportación de lecturas es pausada

                          
    char M;               // Variable encargada de guardar las ordenes, en formato de literales, del usuario o de Sampler. Las ordenes y sus equivalentes 
                          // literales se encuentran en el comentario al inicio de este script. Se repiten a continuación:
                          /* 
BAUDRATE = 57600
Literal X: Iniciar modo Lectura de Barrido
Literal Y: Salir de cualquier modo
Literal r: Empezar lectura
Literal s: Parar lectura
Literal t: Reiniciar las marcas temporales
Literal n: Cambia la configuración de Throttle del ESC. Uso exclusivo de Sampler
*/             
    
    M = Serial.read();    // Lee el buffer del puerto COM en busca de ordenes
    delay(1);
    if (M == 'X') {       // En caso de recibir literal X, iniciar el modo Lectura de Barrido
      digitalWrite(LED_BUILTIN, LOW);
      while (int j=1 > 0) {           // Bloquear el programa en un bucle infinito hasta recibir la orden de salir del modo Lectura de Barrido
        delay(1);
        M = checkCOMforMode(M);           // Actualiza constantemente M en busca de ordenes. Vease la función "char checkComforMode()" 
        if (M == 'r') {                   // En caso de recibir la literal "r": 
          rpmState = digitalRead(RPM);        // Revisar el estado de la sonda de RPM
          if (i == 1) {                       // En caso de que el programa este pausado previo a la orden:
            tp = millis();                          // Guardar la marca de temporal de recepción de la orden
            rpmTimer += 100;                        // Incrementar 100 milisegundos a rpmTimer para que la primera exportación de datos de la sonda se efectue concluido ese periodo
            rpmTimeOld = micros();                  //  <-- Buscar una solución a lo que implica esta linea
            i = 2;                                  // Cambiar la Flag a indicar que el programa esta en ejecución
          }

          if (digitalRead(pinData0) == LOW) {                 // Si loadCell se encuentra lista para la lectura:
            sendSampleData(loadCell, "[HX7T]", "Kg");               // Exportar lectura de fuerza
            sendSampleTime(tp, tk);                                 // Exportar marca temporal de la lectura
          }
          if (digitalRead(pinData1) == LOW) {                 // Si loadCellM1 se encuentra lista para la lectura:
            sendSampleData(loadCellM1, "[HX7M1]", "Kg.m");          // Exportar lecturas de fuerza y marcas temporales
            sendSampleTime(tp, tk);
            sendSampleData(loadCellM2, "[HX7M2]", "Kg.m");
            sendSampleTime(tp, tk);
          }
          checkRPM(i, tp, tk);                                // Revisar si se debe exportar la lectura de la sonda de RPM
          digitalWrite(LED_BUILTIN, LOW);
             
        } else if (M == 'Y') {            // En caso de recibir la literal "Y":
          digitalWrite(LED_BUILTIN, LOW);
          j = sendEndLine();                    // Exporta un fin de linea al COM y asigna 0 a j para salir de bucle infinito del modo Lectura de Barrido
          break;                                // Rompe el ciclo contenedor del condicional anidado. En este caso, "while(int j=1 > 0)"
        } else if (M == 'n') {            // En caso de recibir la literal "n":
          checkCOMforPWM();                     // Recibir la nueva configuración para Throttle del ESC. Vease la función "void checkCOMforPWM()"
        }
        
        if (M == 's' && i == 2) {         // En caso de recibir la literal "s":
          tk = millis() - tp + tk;              // Guardar la marca temporal del momento en que se recibio la orden
          i = 1;                                // Cambiar la Flag a pausado
        } else if (M == 't') {            // En caso de recibir literal "t":
          tk = 0;                               // Reiniciar la marca temporal de pausa
          i = 1;                                // Cambiar la Flag a marca temporal reiniciada
          delay(80);
        }
      }
    }
  }
}

void pin_ISR() {                                          // Función del vector de interrupción (ISR, Interrupt Service Routine)
  volatile byte newState = digitalRead(RPM);              // Variable encargada de reflejar el estado de la salida de la sonda. El ISR es llamado por automático cuando se
                                                          // detecta un cambio de estado en el pin RPM, más no determina el estado. Esta variable sirve para determinar ese estado
                                                          
  if (rpmState == HIGH && newState == LOW) {              // En caso de que la sonda pase de un estado donde no hay pala a uno donde hay detección:
    if (firstBlade == 0) {                                       // En caso de que sea la primer detección de una pala:
      rpmTimeOld = micros();                                           // Asignar la marca temporal del instante en el que se detecto la primer pala
      firstBlade++;                                                   // Activar la Flag de que la primer pala ha sido detectada
    } else {
    rpmTimeDelta[timeIndex] = micros() - rpmTimeOld;          // Guardar el periodo de tiempo transcurrido entre el paso de la última pala y la actual
    timeIndex++;                                              // Aumentar el registro de cuenta de periodos guardados
    rpmTimeOld = micros();                                    // Guardar la marca temporal del instante en el que se detecto la pala actual para obtener el siguiente periodo
    digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  rpmState = newState;                                    // Actualizar el estado guardado de la sonda
}


void sendSampleData(HX711 loadCell, String type, String Units) {    // Función para exportar la lectura de una celda, especificando el tipo de celda y las unidades de medida
  Serial.print(type);                                               // Exportar el tipo de celda. HX7T para tracción, HX7M para torque. Sampler revisa el tipo de celda para actualizar las gráficas
  Serial.print(" Read: ");                                          // Exportar la lectura
  Serial.print(loadCell.get_units()*(-1), 4);
  Serial.print(" ");
  Serial.print(Units);                                              // Exportar las unidades
  Serial.print(" ");
}

void checkRPM(byte i, unsigned long tp, unsigned long tk) {         // Función para revisar si es necesario exportar la lectura de RPM
  if (millis() > rpmTimer && i == 2) {                              // En caso de que el tiempo de ejecución haya superado el límite establecido para exportar la lectura, y que
                                                                    // el programa se encuentre en ejecución, (Flag i = 2):
    double rpm = 0;                                                  
    noInterrupts();                                                       // Deshabilitar temporalmente las interrupciones y el ISR
    rpm = getAverage();                                                   // Obtener el promedio de todos los periodos temporales guardados antes de que "void checkRPM()" haya
                                                                          // sido llamada
    rpm = 60/(rpm*0.000001*numBlades);                                    // Convertir el valor promedio a revoluciones por minuto y compensar para el número de palas
    if (rpm > 16000) {                                                    // Si el valor de rpm supera un valor establecido como imposible de obtener, entonces:
      rpm = rpm/10;                                                             // Dividir el valor entre 10 para corregir la lectura erronea
    }
    timeIndex = 0;                                                        // Reiniciar la cuenta de periodos validos guardados entre el paso de palas
    interrupts();                                                         // Habilitar las interrupciones
    sendRPMData(rpm);                                                     // Exportar las lecturas de rpm
    sendSampleTime(tp, tk);                                               // Exportar la marca temporal del instante en el que se exportó la lectura
    rpmTimer = rpmTimer + 100;                                            // Incrementar el límite de tiempo para exportar nuevamente la lectura de rpm
  }
}

double getAverage() {                     // Función para obtener el valor promedio de los periodos de tiempo entre paso de palas guardados
  unsigned long sum = 0;
  int i;
  noInterrupts();                         // Deshabilitar temporalmente las interrupciones
  for (i=0; i<=timeIndex; i++) {          // Sumatoria de todos los valores indicados hasta timeIndex
    sum = sum + rpmTimeDelta[i];          
  }

  double average = sum/(timeIndex);       // Obtención del valor promedio
  interrupts();                           // Habilitar las interrupciones
  return average;                         
}

void sendRPMData(double rpm) {            // Función análoga a "sendSampleData" para las lecturas de rpm
  
  Serial.print("[RPMp]");
  Serial.print(" Read; ");
  Serial.print(rpm);
  Serial.print(" rpm ");
}

void sendSampleTime(unsigned long tp, unsigned long tk) {   // Función análoga a "sendSampleData" para las marcas temporales
  Serial.print(millis()-tp + tk);
  Serial.print(" ms");
  Serial.println();
}

unsigned long sendEndLine() {     // Función análoga a "sendSampleData" para exportar terminos de linea
  Serial.print("  Done");
  Serial.println();
  return 0;
}

char checkCOMforMode(char M) {      // Función para revisar el puerto COM en busca de ordenes
  if (Serial.available() > 0) {     // En caso de que el buffer del puerto COM este listo para ser leido:
    delay(3);                         
    M = Serial.read();                    // Leer el buffer del puerto COM
    delay(1);
  }
  return M;                         // Regresar el valor, cambiado o no, de M
}

void checkCOMforPWM() {                             // Función para revisar el puerto COM en busca de una nueva configuración para el Throttle del ESC
  if (Serial.available() > 0) {                     // En caso de que el buffer del puerto COM este listo para ser leido:
    int throttle, j;                                      // Variables para guardar la configuración de Throttle
    period dataIn;                                        // Variable de tipo "period" que permite ser trabajada como byte y como long
    // dataIn = 0.6     //  <- 60 % de potencia           // Usar esta linea solo cuando Sampler no este disponible y se desee cambiar manualmente la configuración de potencia
                                                          // Lo anterior requiere cambiar el valor de potencia directamente en esta linea del codigo cada vez que se necesite 
                                                          // diferente potencia
    j = Serial.readBytes(dataIn.b, 4);                    // Leer 4 bytes del buffer del puerto COM y guardarlos en dataIn en formato byte. (j es una variable auxiliar requerida por
                                                          // la función .readBytes que devuelve un valor igual al número de bytes aun guardados en el buffer COM
                                                          
    throttle = dataIn.t*2.5 + 240;                        // Convertir la configuración obtenida de Sampler en un valor tipo long necesario para cambiar el ciclo de trabajo
                                                          // de la señal PWM requerida para cambiar la potencia de la ESC
    noInterrupts();                                       // Deshabilitar temporalmente las interrupciones
    OCR1A = throttle;                                     // Alterar el ciclo de trabajo de la señal PWM. Ver Anexo A
    interrupts();
  }
}

  
