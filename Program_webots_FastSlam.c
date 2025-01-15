
// Algoritma FastSlam

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SENSOR_NUMBER 16
#define RANGE (1024 / 2)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

#define NUM_PARTICLES 100

// Partikel untuk FastSLAM
typedef struct {
  double x, y, theta;  // Posisi robot
  double weight;        // Bobot partikel
  double map[MAX_SENSOR_NUMBER][2];  // Peta yang dibangun oleh partikel
} Particle;

// Variabel untuk sensor, motor, dan kamera
static WbDeviceTag sensors[MAX_SENSOR_NUMBER], camera, left_motor, right_motor;
static double matrix[MAX_SENSOR_NUMBER][2];
static int num_sensors;
static double range;
static int time_step = 0;
static double max_speed = 0.0;
static double speed_unit = 4.0;
static int camera_enabled;

// Array partikel untuk FastSLAM
Particle particles[NUM_PARTICLES];

// Fungsi untuk menginisialisasi Webots dan perangkat robot
void initialize() {
  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();

  const char *robot_name = wb_robot_get_name();
  const char e_puck_name[] = "ps0";
  const char khepera_name[] = "ds0";
  const char koala_name[] = "ds0";
  const char pioneer2_name[] = "ds0";

  char sensors_name[5];
  const double (*temp_matrix)[2];

  camera_enabled = 0;
  range = RANGE;

  const double e_puck_matrix[8][2] = {{150, -35}, {100, -15}, {80, -10},  {-10, -10},
                                      {-10, -10}, {-10, 80},  {-30, 100}, {-20, 150}};
  const double khepera3_matrix[9][2] = {{-5000, -5000},  {-20000, 40000}, {-30000, 50000}, {-70000, 70000}, {70000, -60000},
                                        {50000, -40000}, {40000, -20000}, {-5000, -5000},  {-10000, -10000}};
  const double khepera_matrix[8][2] = {{-2, 4}, {-3, 5}, {-7, 7}, {7, -6}, {5, -4}, {4, -2}, {-0.5, -0.5}, {-0.5, -0.5}};
  const double pioneer2_matrix[16][2] = {{-1, 15}, {-3, 13}, {-3, 8},  {-2, 7}, {-3, -4}, {-4, -2}, {-3, -2}, {-1, -1},
                                         {-1, -1}, {-2, -3}, {-2, -4}, {4, -3}, {7, -5},  {7, -3},  {10, -2}, {11, -1}};
  const double koala_matrix[16][2] = {{17, -1}, {8, -2},  {4, -3},  {9, -2}, {5, -3}, {-4, -2}, {-4, -2}, {-2, -2},
                                      {-2, -2}, {-2, -4}, {-2, -4}, {-4, 5}, {-3, 8}, {-3, 5},  {-2, 10}, {-1, 15}};

  if (strncmp(robot_name, "e-puck", 6) == 0) {
    num_sensors = 8;
    sprintf(sensors_name, "%s", e_puck_name);
    temp_matrix = e_puck_matrix;
    max_speed = 6.28;
    speed_unit = 0.00628;
  } else if (strncmp(robot_name, "Khepera III", 8) == 0) {
    num_sensors = 9;
    sprintf(sensors_name, "%s", khepera_name);
    temp_matrix = khepera3_matrix;
    range = 2000;
    max_speed = 19.1;
    speed_unit = 0.00053429;
  } else if (strncmp(robot_name, "khepera", 7) == 0) {
    num_sensors = 8;
    sprintf(sensors_name, "%s", khepera_name);
    temp_matrix = khepera_matrix;
    max_speed = 1.0;
    speed_unit = 1.0;
  } else if (strcmp(robot_name, "koala") == 0) {
    num_sensors = 16;
    sprintf(sensors_name, "%s", koala_name);
    temp_matrix = koala_matrix;
    max_speed = 10.0;
    speed_unit = 0.1;
  } else if (strcmp(robot_name, "pioneer2") == 0) {
    num_sensors = 16;
    sprintf(sensors_name, "%s", pioneer2_name);
    temp_matrix = pioneer2_matrix;
    max_speed = 10.0;
    speed_unit = 0.1;
  } else {
    fprintf(stderr, "This controller doesn't support this robot\n");
    exit(EXIT_FAILURE);
  }

  for (int i = 0; i < num_sensors; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);
    if ((i + 1) >= 10) {
      sensors_name[2] = '1';
      sensors_name[3]++;
      if ((i + 1) == 10) {
        sensors_name[3] = '0';
        sensors_name[4] = '\0';
      }
    } else
      sensors_name[2]++;
    for (int j = 0; j < 2; j++) {
      matrix[i][j] = temp_matrix[i][j];
    }
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  if (camera_enabled == 1) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, time_step);
  }

  printf("The %s robot is initialized, it uses %d distance sensors\n", robot_name, num_sensors);
}

// Fungsi untuk menginisialisasi partikel
void initialize_particles() {
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x = rand() % 1000 - 500;  // Posisi acak
    particles[i].y = rand() % 1000 - 500;
    particles[i].theta = rand() % 360;    // Sudut acak
    particles[i].weight = 1.0 / NUM_PARTICLES;  // Bobot partikel awal sama
  }
}

// Fungsi untuk memperbarui posisi partikel berdasarkan gerakan robot
void update_particle_positions(double v, double omega, double delta_t) {
  for (int i = 0; i < NUM_PARTICLES; i++) {
    particles[i].x += v * cos(particles[i].theta) * delta_t;
    particles[i].y += v * sin(particles[i].theta) * delta_t;
    particles[i].theta += omega * delta_t;
  }
}

// Fungsi untuk memperbarui bobot partikel berdasarkan pembacaan sensor
void update_particle_weights(double *sensors_value) {
  for (int i = 0; i < NUM_PARTICLES; i++) {
    double weight = 1.0;

    for (int j = 0; j < num_sensors; j++) {
      double predicted_distance = 0.0;  // Misalnya, gunakan model untuk menghitung jarak prediksi
      weight *= exp(-0.5 * pow(sensors_value[j] - predicted_distance, 2));
    }

    particles[i].weight = weight;
  }
}

// Fungsi untuk melakukan resampling partikel
void resample_particles() {
  Particle new_particles[NUM_PARTICLES];

  for (int i = 0; i < NUM_PARTICLES; i++) {
    int idx = rand() % NUM_PARTICLES;  // Pilih partikel berdasarkan bobot
    new_particles[i] = particles[idx];
  }

  memcpy(particles, new_particles, sizeof(particles));
}

int main() {
  initialize();
  initialize_particles();  // Inisialisasi partikel

  while (wb_robot_step(time_step) != -1) {
    int i;
    double sensors_value[MAX_SENSOR_NUMBER];
    double speed[2];
    double v = 0.0, omega = 0.0;

    // Pembacaan sensor
    for (i = 0; i < num_sensors; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    // Algoritma Braitenberg untuk kecepatan motor
    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
      for (int j = 0; j < num_sensors; j++) {
        speed[i] += speed_unit * matrix[j][i] * (1.0 - (sensors_value[j] / range));
      }
      speed[i] = BOUND(speed[i], -max_speed, max_speed);
    }

    v = speed[0];  // Kecepatan linier
    omega = (speed[1] - speed[0]) / 2.0;  // Kecepatan angular
    update_particle_positions(v, omega, time_step);  // Update posisi partikel

    // Update bobot partikel
    update_particle_weights(sensors_value);

    // Lakukan resampling
    resample_particles();

    // Set kecepatan motor
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  return 0;
}


//end program
