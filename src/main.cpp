#include <MPU6050.h>
#include <Servo.h>
#include <Wire.h>

MPU6050 mpu6050;
Servo servo_gauche;
Servo servo_droite;

const int taille_liste_angles = 10;      // Taille du tampon pour la moyenne mobile
float liste_angles[taille_liste_angles]; // Tampon pour stocker les valeurs d'angle
int index = 0;                           // Index actuel dans le tampon

double input, output;

// Coefficients PID pour l'angle
double Kp_angle = 2.5;
double Ki_angle = 0;
double Kd_angle = 1;

// Coefficients PID pour la vitesse
double Kp_vitesse = 2;
double Ki_vitesse = 0;
double Kd_vitesse = 0;

float a = 2;

// Limites de l'intégrale pour prévenir le "windup"
double integral_limit = 1000;

double previous_error = 0.01;
double integral = 0;
int TARGET_ANGLE = 100;
const int TARGET_ACCELERATION = 3;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  mpu6050.initialize();
  servo_gauche.attach(10);
  servo_droite.attach(11);
}

float sigmoid(float a, float x)
{
  return 1 / (1 + exp(-a * (x - 105)));
}

float sigmoid_inverse(float a, float x)
{
  return 1 / (1 + exp(a * (x - 75)));
}

float calculer_angle()
{
  int16_t angle_x, angle_y, angle_z, gyro_x, gyro_y, gyro_z;
  mpu6050.getMotion6(&angle_x, &angle_y, &angle_z, &gyro_x, &gyro_y, &gyro_z);
  float angle = atan2((double)angle_y / 16384.0, (double)angle_z / 16384.0) * 180 / PI;
  return angle;
}

float moyenne_mobile(float nouvel_angle)
{
  liste_angles[index] = nouvel_angle;
  index = (index + 1) % taille_liste_angles; // Avance l'index du tampon, en bouclant si nécessaire

  float somme_liste = 0;
  for (int i = 0; i < taille_liste_angles; i++)
  {
    somme_liste += liste_angles[i];
  }

  float moyenne_angles = somme_liste / taille_liste_angles;

  return moyenne_angles;
}

double calculer_pid(double Kp, double Ki, double Kd, double input, double target)
{
  double error = target - input;

  integral += error;
  // Limiter l'intégrale pour éviter le "windup"
  if (integral > integral_limit)
    integral = integral_limit;
  if (integral < -integral_limit)
    integral = -integral_limit;

  double derivative = (error - previous_error);
  previous_error = error;

  double output = (Kp * error + Ki * integral + Kd * derivative);

  // Limiter la sortie
  if (output > 90)
    output = 90;
  if (output < -90)
    output = -90;

  return output;
}

void rotate_servos(double servo_value)
{
  if (servo_value < 10 && servo_value > -10)
  {
    servo_value = 0;
  }
  double value_servo_droite = 90 + servo_value;
  double value_servo_gauche = 90 - servo_value;

  servo_gauche.write(value_servo_gauche);
  servo_droite.write(value_servo_droite);
}

void stabiliser_angle(int target_angle)
{
  float angle = calculer_angle();
  angle = moyenne_mobile(angle); // Lissage des valeurs de l'angle

  double output = calculer_pid(Kp_angle, Ki_angle, Kd_angle, angle, target_angle);

  Serial.println("Angle : " + String(angle) + "\t Output moteurs : " + String(output));

  rotate_servos(output);
}

void loop()
{
  stabiliser_angle(TARGET_ANGLE);
  delay(3); // Gestion simple du temps d'échantillonnage
}
