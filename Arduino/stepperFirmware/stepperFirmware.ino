#include <AccelStepper.h>

/*
  Robot diferencial con dos motores 28BYJ-48 + ULN2003
  Control por velocidad continua (sin destino)
  Modo no bloqueante: setSpeed() + runSpeed()

  Comandos por Serial desde Raspberry Pi:
    MOV V <lineal> W <angular>
    STOP
    GET
    GOL <vel> <pos> -> Go Left <velocidad> <pasos>. Donde velocidad entre 0.0 y 1.0
    GOR <vel> <pos> -> Go Rigth <velocidad> <pasos>. Donde velocidad entre 0.0 y 1.0
  Donde:
    V = velocidad lineal (entre 0.0 y 1.0)
    W = velocidad angular (entre 0.0 y 1.0)
    velLeft / velRight = velocidad de ruedas (pasos/seg)
*/

// Pines del motor izquierdo
#define L_IN1 2
#define L_IN2 3
#define L_IN3 4
#define L_IN4 5

// Pines del motor derecho
#define R_IN1 11
#define R_IN2 10
#define R_IN3 9
#define R_IN4 8

#define MAX_SPEED 600.0
#define MAX_ACCEL 1000.0
#define MIN_SPEED 10.0

#define INTERVALO_POS_MS 500

// Motores en modo FULL4WIRE
#define L 0  //Left
#define R 1  //Rigth
AccelStepper motores[] = {
  AccelStepper(AccelStepper::FULL4WIRE, L_IN1, L_IN3, L_IN2, L_IN4, false),
  AccelStepper(AccelStepper::FULL4WIRE, R_IN1, R_IN3, R_IN2, R_IN4, false)
};

const float k_linear = MAX_SPEED;   // pasos/segundo por unidad de velocidad lineal
const float k_angular = MAX_SPEED;  // pasos/segundo por unidad de velocidad angular

// Velocidades actuales de cada rueda (pasos/s)
float vL = 0;
float vR = 0;

String buffer = "";

void setup() {
  Serial.begin(115200);

  motores[L].setMaxSpeed(MAX_SPEED);
  motores[R].setMaxSpeed(MAX_SPEED);
  motores[L].setAcceleration(MAX_ACCEL);
  motores[R].setAcceleration(MAX_ACCEL);

  Serial.println("ARDUINO READY");
}

void loop() {
  static unsigned long lastTime = 0;
  static unsigned long now = 0;

  // Ejecuta movimiento continuo (NO BLOQUEANTE)
  motores[L].run();
  motores[R].run();

  // Procesar comandos entrantes
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        handleCommand(buffer);
        buffer = "";
      }
    } else {
      buffer += c;
    }
  }

  now = millis();
  if (now - lastTime >= INTERVALO_POS_MS) {  // han pasado 200 ms
    lastTime = now;
    Serial.print(now);
    Serial.print(" L ");
    Serial.print(motores[L].isRunning());
    Serial.print(" ");
    Serial.print(motores[L].currentPosition());
    Serial.print(" R ");
    Serial.print(motores[R].isRunning());
    Serial.print(" ");
    Serial.println(motores[R].currentPosition());        
  }
}

//Establece velocidades: V=Velocidad W=Giro
void setWheelSpeeds(float V, float W) {
  //Correcion en modulo. para que la velocidad no se haga mayor que max_velicidad.
  float VW = abs(V) + abs(W);
  if (VW > 1) {
    V = V / (VW);
    W = W / (VW);
  }
  // Conversión a velocidades de rueda:
  float vLR[2];  //velocidad Left & Right
  vLR[0] = (V * k_linear) - (W * k_angular);
  vLR[1] = (V * k_linear) + (W * k_angular);

  for (int lr = 0; lr < 2; lr++) {
    if (!motores[lr].isRunning())
      motores[lr].enableOutputs();
    motores[lr].setMaxSpeed(abs(vLR[lr]));
    if (vLR[lr] >= MIN_SPEED)
      motores[lr].moveTo(0x3FFFFFFF);  // 0x3FFFFFFF = MaxInt32 / 2
    else if ((vLR[lr] <= -MIN_SPEED))
      motores[lr].moveTo(0xBFFFFFFF);  // 0xBFFFFFFF = -MaxInt32 / 2
    else {
      motores[lr].stop();
      motores[lr].disableOutputs();
      vLR[lr] = 0.0;
    }
  }
}

void setWheelTo(char m, float v, int p) {
  AccelStepper* motor;
  if (m == 'L')
    motor = &motores[L];
  else
    motor = &motores[R];

  // Conversión a velocidades de rueda:
  v = (v * k_linear);
  if (v < 0) p = -p;

  if (motor->isRunning()) {
    motor->setMaxSpeed(abs(v));
  } else {
    motor->enableOutputs();
    motor->setCurrentPosition(0);
    motor->setMaxSpeed(abs(v));
  }
  motor->move(p);  //motor->moveTo(motor->currentPosition() + p);
}

void handleCommand(String cmd) {
  cmd.trim();

  // Separar tokens
  char buf[cmd.length() + 1];
  cmd.toCharArray(buf, sizeof(buf));
  char* t = strtok(buf, " ");

  if (!t) return;

  String s = String(t);

  // ---------------------------
  // SET velocidades diferenciales
  // Formato: MOV V <lineal> W <angular>
  // ---------------------------
  if (s == "MOV") {
    float V = 0;
    float W = 0;

    char* p = strtok(NULL, " ");
    while (p) {
      String key = String(p);
      char* val = strtok(NULL, " ");
      if (!val) break;

      if (key == "V") V = atof(val);
      if (key == "W") W = atof(val);

      p = strtok(NULL, " ");
    }

    setWheelSpeeds(V, W);
    return;
  }

  // STOP
  if (s == "STOP") {
    motores[L].stop();
    motores[R].stop();
    motores[L].disableOutputs();
    motores[R].disableOutputs();
    //motores[L].setCurrentPosition(0);
    //motores[R].setCurrentPosition(0);
    return;
  }

  // GOL <vel> <pos> -> velocidad directa rueda izquierda
  // GOR <vel> <pos> -> rueda derecha
  if (s == "GOL") {
    char* v = strtok(NULL, " ");
    char* p = strtok(NULL, " ");
    if (p) setWheelTo('L', atof(v), atoi(p));
    return;
  }
  if (s == "GOR") {
    char* v = strtok(NULL, " ");
    char* p = strtok(NULL, " ");
    if (p) setWheelTo('R', atof(v), atoi(p));
    return;
  }
}
