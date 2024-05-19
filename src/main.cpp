#include <MPU6050.h>
#include <Servo.h>
#include <Wire.h>

MPU6050 mpu6050;
Servo servo_gauche;
Servo servo_droite;

const int taille_liste_angles = 10;      // Taille du tampon pour la moyenne mobile
float liste_angles[taille_liste_angles]; // Tampon pour stocker les valeurs d'angle
int index = 0;

int compteur = 0; // Compteur pour gérer l'alternance des angles

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

// Limites de l'intégrale
double integral_limit = 1000;

double previous_error = 0.01;
double integral = 0;
float TARGET_ANGLE = 96;
float TARGET_ANGLE_AVANCER = 98;
float TARGET_ACCELERATION = 3;

// Variables pour interpolation douce
float current_target_angle = TARGET_ANGLE;
float angle_step = 0.1; // Incrément pour une transition douce

unsigned long previous_time = 0; // Temps précédent pour gérer l'alternance

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

double calculer_pid(double Kp, double Ki, double Kd, double input, float target)
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
  if (servo_value < 6 && servo_value > -6)
  {
    servo_value = 0;
  }
  double value_servo_droite = 90 + servo_value;
  double value_servo_gauche = 90 - servo_value;

  servo_gauche.write(value_servo_gauche);
  servo_droite.write(value_servo_droite);
}

void stabiliser_angle(float target_angle)
{
  float angle = calculer_angle();
  angle = moyenne_mobile(angle); // Lissage des valeurs de l'angle

  double output = calculer_pid(Kp_angle, Ki_angle, Kd_angle, angle, target_angle);

  Serial.print(">angle:");
  Serial.println(angle);
  // Serial.println("Angle : " + String(angle) + "\t Output moteurs : " + String(output));

  rotate_servos(output);
}

float transition_avancer_stabiliser(float current_target_angle) {
  // Incrémenter le compteur
  compteur++;

  // Vérifier si le compteur est à une seconde
  if (compteur % 100 == 0) // Étant donné que nous avons un delay(10) à la fin, 100 itérations équivalent à 1 seconde
  {
    // Alterner entre les deux angles cibles
    if (TARGET_ANGLE == 96)
    {
      TARGET_ANGLE = TARGET_ANGLE_AVANCER;
    }
    else
    {
      TARGET_ANGLE = 96;
    }
  }

  // Interpoler en douceur entre les angles cibles
  if (current_target_angle < TARGET_ANGLE)
  {
    current_target_angle += angle_step;
    if (current_target_angle > TARGET_ANGLE)
    {
      current_target_angle = TARGET_ANGLE;
    }
  }
  else if (current_target_angle > TARGET_ANGLE)
  {
    current_target_angle -= angle_step;
    if (current_target_angle < TARGET_ANGLE)
    {
      current_target_angle = TARGET_ANGLE;
    }
  }

  Serial.println("Current Target Angle: " + String(current_target_angle));

  return current_target_angle;
}

void loop()
{
  current_target_angle = transition_avancer_stabiliser(current_target_angle);

  stabiliser_angle(current_target_angle);
  // delay(10); // Gestion simple du temps d'échantillonnage
}
