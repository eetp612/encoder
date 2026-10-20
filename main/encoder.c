#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stddef.h>

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"

#include "driver/gpio.h"

#include "uros_network_interfaces.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>

#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// ============================================================
// Configuracion micro-ROS
// ============================================================

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 8192
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

// ============================================================
// Configuracion de encoders
// ============================================================
//
// Este programa asume encoders incrementales en cuadratura,
// con dos canales (A y B) por rueda.
//
// Los cuatro GPIO se configuran como entradas con pull-up.
// Cambia los pines si tu placa ya los utiliza para otros
// perifericos. Evita GPIO 19/20 si usas el USB nativo del S3.
//
// El conteo es x4: se contabilizan los flancos de A y B.
// ENCODER_*_COUNTS_PER_REV debe indicar las cuentas x4 por
// revolucion de la RUEDA, despues de considerar la reductora.
//
// ============================================================

#define ENCODER_LEFT_A_GPIO       GPIO_NUM_4
#define ENCODER_LEFT_B_GPIO       GPIO_NUM_5
#define ENCODER_RIGHT_A_GPIO      GPIO_NUM_6
#define ENCODER_RIGHT_B_GPIO      GPIO_NUM_7

// EJEMPLOS; reemplazar por valores reales de cada encoder/rueda.
#define ENCODER_LEFT_COUNTS_PER_REV   600.0
#define ENCODER_RIGHT_COUNTS_PER_REV  600.0

// EJEMPLOS en metros; reemplazar por las medidas reales.
#define WHEEL_RADIUS_LEFT_M       0.033
#define WHEEL_RADIUS_RIGHT_M      0.033
#define WHEEL_SEPARATION_M        0.160

// Frecuencia de publicacion de /odom.
#define ODOM_PUBLISH_PERIOD_MS    50   // 20 Hz
#define ODOM_PI                  3.14159265358979323846

// Si las salidas del encoder son open-collector, los pull-ups
// internos pueden servir. Si el modulo ya entrega niveles
// push-pull, tambien suelen ser compatibles; verificar hardware.
#define ENCODER_PULLUP            GPIO_PULLUP_ENABLE

// ============================================================
// Estado compartido con las ISR
// ============================================================

typedef struct
{
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    volatile int64_t count;
    volatile uint8_t previous_state;
    int id;
} encoder_t;

static encoder_t encoder_left = {
    .pin_a = ENCODER_LEFT_A_GPIO,
    .pin_b = ENCODER_LEFT_B_GPIO,
    .count = 0,
    .previous_state = 0,
    .id = 0
};

static encoder_t encoder_right = {
    .pin_a = ENCODER_RIGHT_A_GPIO,
    .pin_b = ENCODER_RIGHT_B_GPIO,
    .count = 0,
    .previous_state = 0,
    .id = 1
};

static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

// Tabla de decodificacion de cuadratura.
// Estado = (A << 1) | B.
// La secuencia 00 -> 01 -> 11 -> 10 -> 00 cuenta +1.
// Si el cableado/sentido mecanico queda invertido, intercambiar
// A y B de esa rueda o invertir su signo en la configuracion.
DRAM_ATTR static const int8_t quadrature_table[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

static inline uint8_t IRAM_ATTR encoder_read_state(const encoder_t *encoder)
{
    const uint8_t a = (uint8_t)gpio_get_level(encoder->pin_a);
    const uint8_t b = (uint8_t)gpio_get_level(encoder->pin_b);
    return (uint8_t)((a << 1) | b);
}

static void IRAM_ATTR encoder_gpio_isr(void *arg)
{
    encoder_t *encoder = (encoder_t *)arg;

    portENTER_CRITICAL_ISR(&encoder_mux);

    // Leer el estado dentro de la seccion critica para evitar
    // que ISR de A y B se pisen entre si.
    const uint8_t current_state = encoder_read_state(encoder);

    const uint8_t index =
        (uint8_t)((encoder->previous_state << 2) | current_state);

    // Los estados iguales no suman. Transiciones ilegales de dos
    // bits consecutivos producen 0 en la tabla y se descartan.
    encoder->count += quadrature_table[index];
    encoder->previous_state = current_state;

    portEXIT_CRITICAL_ISR(&encoder_mux);
}

// ============================================================
// Inicializacion GPIO e interrupciones de encoders
// ============================================================

static void encoder_gpio_init(void)
{
    const uint64_t pin_mask =
        (1ULL << ENCODER_LEFT_A_GPIO)  |
        (1ULL << ENCODER_LEFT_B_GPIO)  |
        (1ULL << ENCODER_RIGHT_A_GPIO) |
        (1ULL << ENCODER_RIGHT_B_GPIO);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = ENCODER_PULLUP,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Capturar los estados iniciales antes de activar las ISR.
    encoder_left.previous_state = encoder_read_state(&encoder_left);
    encoder_right.previous_state = encoder_read_state(&encoder_right);

    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(
        ENCODER_LEFT_A_GPIO, encoder_gpio_isr, &encoder_left));
    ESP_ERROR_CHECK(gpio_isr_handler_add(
        ENCODER_LEFT_B_GPIO, encoder_gpio_isr, &encoder_left));
    ESP_ERROR_CHECK(gpio_isr_handler_add(
        ENCODER_RIGHT_A_GPIO, encoder_gpio_isr, &encoder_right));
    ESP_ERROR_CHECK(gpio_isr_handler_add(
        ENCODER_RIGHT_B_GPIO, encoder_gpio_isr, &encoder_right));

    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_LEFT_A_GPIO, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_LEFT_B_GPIO, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_RIGHT_A_GPIO, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENCODER_RIGHT_B_GPIO, GPIO_INTR_ANYEDGE));

    ESP_LOGI("ENCODER", "GPIO e interrupciones inicializados");
}

// ============================================================
// Lectura atomica de los contadores
// ============================================================

static void encoder_get_counts(int64_t *left_count, int64_t *right_count)
{
    portENTER_CRITICAL(&encoder_mux);
    *left_count = encoder_left.count;
    *right_count = encoder_right.count;
    portEXIT_CRITICAL(&encoder_mux);
}

// ============================================================
// Odometry y micro-ROS
// ============================================================

static rcl_publisher_t odom_publisher;
static nav_msgs__msg__Odometry odom_msg;

static double odom_x = 0.0;
static double odom_y = 0.0;
static double odom_yaw = 0.0;

static int64_t previous_left_count = 0;
static int64_t previous_right_count = 0;
static int64_t previous_time_us = 0;

#define RCCHECK(fn) do {                                                \
    rcl_ret_t rc_ = (fn);                                               \
    if (rc_ != RCL_RET_OK) {                                            \
        printf("micro-ROS error en linea %d: %d\n", __LINE__, (int)rc_); \
        vTaskDelete(NULL);                                              \
        return;                                                         \
    }                                                                   \
} while (0)

#define RCSOFTCHECK(fn) do {                                            \
    rcl_ret_t rc_ = (fn);                                               \
    if (rc_ != RCL_RET_OK) {                                            \
        printf("micro-ROS aviso en linea %d: %d\n", __LINE__, (int)rc_); \
    }                                                                   \
} while (0)

static void odometry_message_init(void)
{
    if (!nav_msgs__msg__Odometry__init(&odom_msg)) {
        ESP_LOGE("ODOM", "No se pudo inicializar nav_msgs/Odometry");
        abort();
    }

    if (!rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom") ||
        !rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link")) {
        ESP_LOGE("ODOM", "No se pudieron reservar los frame_id");
        abort();
    }

    // Covarianzas diagonales iniciales orientativas. Ajustarlas
    // de acuerdo con la precision real del robot.
    for (size_t i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0]  = 0.02;  // x
    odom_msg.pose.covariance[7]  = 0.02;  // y
    odom_msg.pose.covariance[35] = 0.05;  // yaw
    odom_msg.twist.covariance[0]  = 0.05; // vx
    odom_msg.twist.covariance[35] = 0.05; // wz
}

static void odometry_update_and_publish(void)
{
    int64_t left_count;
    int64_t right_count;
    encoder_get_counts(&left_count, &right_count);

    const int64_t now_us = esp_timer_get_time();

    if (previous_time_us == 0) {
        previous_left_count = left_count;
        previous_right_count = right_count;
        previous_time_us = now_us;
        return;
    }

    const double dt = (double)(now_us - previous_time_us) / 1000000.0;
    if (dt <= 0.0) {
        return;
    }

    const int64_t delta_left_count = left_count - previous_left_count;
    const int64_t delta_right_count = right_count - previous_right_count;

    previous_left_count = left_count;
    previous_right_count = right_count;
    previous_time_us = now_us;

    const double meters_per_count_left =
        (2.0 * ODOM_PI * WHEEL_RADIUS_LEFT_M) / ENCODER_LEFT_COUNTS_PER_REV;
    const double meters_per_count_right =
        (2.0 * ODOM_PI * WHEEL_RADIUS_RIGHT_M) / ENCODER_RIGHT_COUNTS_PER_REV;

    const double delta_left_m = (double)delta_left_count * meters_per_count_left;
    const double delta_right_m = (double)delta_right_count * meters_per_count_right;

    // Cinematica diferencial.
    const double delta_s = (delta_right_m + delta_left_m) / 2.0;
    const double delta_yaw = (delta_right_m - delta_left_m) / WHEEL_SEPARATION_M;

    // Integracion usando el rumbo medio del intervalo.
    const double yaw_mid = odom_yaw + (delta_yaw / 2.0);
    odom_x += delta_s * cos(yaw_mid);
    odom_y += delta_s * sin(yaw_mid);
    odom_yaw += delta_yaw;

    const double linear_velocity = delta_s / dt;
    const double angular_velocity = delta_yaw / dt;

    // Quaternion planar (rotacion solamente sobre Z).
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(odom_yaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(odom_yaw / 2.0);

    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;

    // Si se sincronizo el reloj con el agente, epoch_nanos()
    // entrega el tiempo ROS. De lo contrario quedara un tiempo
    // local/no sincronizado y conviene revisar la sincronizacion.
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    const int64_t epoch_ns = rmw_uros_epoch_nanos();
    if (epoch_ns > 0) {
        odom_msg.header.stamp.sec = (int32_t)(epoch_ns / 1000000000LL);
        odom_msg.header.stamp.nanosec = (uint32_t)(epoch_ns % 1000000000LL);
    } else
#endif
    {
        odom_msg.header.stamp.sec = 0;
        odom_msg.header.stamp.nanosec = 0;
    }

    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

// ============================================================
// Tarea micro-ROS: publica nav_msgs/msg/Odometry en /odom
// ============================================================

static void micro_ros_task(void *arg)
{
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);

    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(
        &support, 0, NULL, &init_options, &allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    // Solicitar sincronizacion de reloj con el agente para sellos ROS.
    (void)rmw_uros_sync_session(1000);
#endif

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(
        &node, "esp32_encoder_odometry", "", &support));

    odometry_message_init();

    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"));

    ESP_LOGI("ODOM", "Publicando nav_msgs/msg/Odometry en /odom");

    while (1) {
        odometry_update_and_publish();
        vTaskDelay(pdMS_TO_TICKS(ODOM_PUBLISH_PERIOD_MS));
    }

    // El bucle es intencionalmente permanente.
    // Si se agrega una salida, finalizar publisher, nodo y support.
}

// ============================================================
// app_main
// ============================================================

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || \
    defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    encoder_gpio_init();

    BaseType_t task_result = xTaskCreate(
        micro_ros_task,
        "uros_odom_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);

    if (task_result != pdPASS) {
        ESP_LOGE("MAIN", "No se pudo crear la tarea micro-ROS");
    }
}
