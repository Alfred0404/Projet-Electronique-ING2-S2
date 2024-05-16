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

// 2.5 | 0 | 0.6 | a=1    pas mal mais trop d'oscillations
// 2.5 | 0 | 1 | a=1      insane
// 2.5 | 0 | 1 | a=2      goatesque
// 2.8 | 0 | 1 | a=3      très bon aussi

// valeurs du PID
double Kp = 2.5;
double Ki = 0.01;
double Kd = 1.8;

// coef a pour la sigmoid
double a = 0.8;

int vitesse_x = 0;
int vitesse_y = 0;

double previous_error = 0.01;
double integral = 0;
const int TARGET = 96;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  mpu6050.initialize();
  servo_gauche.attach(10);
  servo_droite.attach(11);
}

/*
 * Fonction pour la fonction sigmoïde
 * @param x: valeur de l'angle
 * @param a: coefficient de la fonction sigmoïde
 * @return sigmoid de l'entrée
 */
float sigmoid(float x, float a)
{
  return 1 / (1 + exp(-a * (x - 94))); // changer pour 100
}

/*
 * Fonction pour la fonction sigmoïde inversée
 * @param x: valeur de l'angle
 * @param a: coefficient de la fonction sigmoïde
 * @return : sigmoid inverse de l'entrée
 */
float sigmoidInverse(float x, float a)
{
  return 1 / (1 + exp(a * (x - 86))); //  changer pour 80
}


//! frauduleux de ouf
// float calculer_vitesse(float acc_x) {
//   vitesse_x += acc_x;
//   return vitesse_x;
// }


/*
  * Fonction pour obtenir l'accélération de l'IMU
  * @return : l'accélération mesurée
*/
int get_acceleration() {
  int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  mpu6050.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);

  Serial.println("Acceleration x : " + String(acc_x));

  return acc_x;
}


/*
 * Fonction pour calculer l'angle de l'IMU
 */
float calculer_angle()
{
  int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  mpu6050.getMotion6(&acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
  float angle = atan2((double)acc_y / 16384.0, (double)acc_z / 16384.0) * 180 / PI;
  return angle;
}

/*
 * Fonction pour calculer la moyenne mobile
 * @param nouvel_angle: nouvel angle de l'IMU
 */
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

/*
 * Fonction pour calculer le PID
 * @param angle: angle de l'IMU
 */
double calculer_pid(float angle)
{
  double error = TARGET - input;

  integral += error;
  double derivative = (error - previous_error);
  previous_error = error;

  output = Kp * error + Ki * integral + Kd * derivative;
  output = output;

  if (output > 90)
  {
    output = 90;
  }

  else if (output < -90)
  {
    output = -90;
  }

  if (output < 2 && output > -2)
  {
    output = 0;
  }

  return output;
}

/*
 * Fonction pour faire tourner les servos
 * @param servo_value: valeur de l'angle pour les servos
 */
void rotate_servos(double servo_value)
{
  double value_servo_droite = 90 + output;
  double value_servo_gauche = 90 - output;

  servo_gauche.write(value_servo_gauche);
  servo_droite.write(value_servo_droite);
}

void loop()
{
  input = calculer_angle();
  input = moyenne_mobile(input); // Lissage des valeurs de l'angle

  double output = calculer_pid(input);

  // Utiliser la fonction sigmoïde pour la plage 90 à 180
  if (input >= 90)
  {
    output = 90 * sigmoid(input, a);
  }
  // Utiliser la fonction sigmoïde inversée pour la plage 0 à 90
  else
  {
    output = 90 * sigmoidInverse(input, a);
  }

  // if (output < 2 && output > -2) {
  //   output = 0;
  // }

  rotate_servos(output);

  Serial.println("input : " + String(input) + "\t output : " + String(output));

  delay(10); // Gestion simple du temps d'échantillonnage
}

// TODO
//  ET2: Vous devrez faire une acquisition temporelle de l’angle sur une période de 10 secondes.

// ET2: Vous devrez faire une acquisition temporelle de l’angle pour différentes valeurs de pid
//      pour mettre en évidence l'effet de chaque coefficient.

// ET4. Tout en restant à la verticale, le robot doit pouvoir se déplacer en ligne droite sur au
//      moins 80 cm, dans un couloir de 25 cm de large.

// ET5. Tout en restant à la verticale et en avançant en ligne droite, le robot doit pouvoir se
//      déplacer à une vitesse rectiligne constante pendant 10 secondes, avec au maximum ±��% de
//      fluctuation