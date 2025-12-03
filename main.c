#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/display/mb_display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h> //permite generar el sonido del buzzer
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <math.h>
#include <stdio.h> 

//configura que nodo esta conectado al buzzer, para esto, crea una variable que contiene toda la información de configuración de los pines para el buzzer jalando esos datos directamente del device tree, sin esa línea, el programa debe hacerlo, pero no sabe en qué pin está conectado el buzzer.
static const struct pwm_dt_spec buzzer = PWM_DT_SPEC_GET(DT_NODELABEL(buzzer_pwm_custom));

// SOn frecuenciaqs aleatorias para que suene una melodia al completar la meta
#define NOTE_C4  262
#define NOTE_E4  330
#define NOTE_G4  392
#define NOTE_C5  523

//lee el acelerometro desde device tree
#define ACCEL_NODE DT_ALIAS(accel0)

// define el boton
#define BUTTON_A_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec buttonA = GPIO_DT_SPEC_GET(BUTTON_A_NODE, gpios);

const struct device *accel;
struct mb_display *disp;

//al iniciar el bluetooth, si hay error lo marca, sino, inicia
static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth fallo (%d)\n", err);
    } else {
        printk("Bluetooth listo\n");
    }
}
//permite que se lean los datos del bluetooth, una parte es ¿que se manda? la otra contesta esto
static ssize_t read_data(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len,
                         uint16_t offset)
{
    const char *value = (const char *)attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

char bt_buffer[60] = "Iniciando...\n";

BT_GATT_SERVICE_DEFINE(podo_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0x12,0x34,0x56,0x78,
        0x12,0x34,
        0x12,0x34,
        0x12,0x34,
        0x12,0x34,0x56,0x78,0x90,0xAB)),

    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xAB,0x90,0x78,0x56,
        0x34,0x12,
        0x34,0x12,
        0x56,0x78,
        0x90,0x12,0x34,0x56,0x78,0x12),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, 
        BT_GATT_PERM_READ,
        read_data, NULL, bt_buffer)
);

//guarda valores de pitch, roll & yaw
struct sensor_value ax, ay, az;
double Ax, Ay, Az;
double mag, diff;

double gravity = 9.8;
//
double umbral = 1.2;

int pasos = 0;
//esta longitud corresponde a una persona de mas o menos 170 cm, se calcula al hacer una multiplicacion de la estatura por 41.4 para obtener la distancia que es recorrida por paso en promedio
double longitud_paso = 0.70;
double distancia = 0.0;

//la meta aumenta por el boton a cada 100m 
int meta_m = 0;
int last_button_state = 1;
int meta_cumplida = 0;
int primera_vuelta = 1;

//palomita 
static const struct mb_image meta_img =
    MB_IMAGE({
        {0,0,0,0,1},
        {0,0,0,1,0},
        {1,0,1,0,0},
        {1,1,0,0,0},
        {1,0,0,0,0},
    });

//limpíar el display
static const struct mb_image blank_img =
    MB_IMAGE({
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
    });

//cronometros
uint32_t t0 = 0;
uint32_t t1 = 0;

//genera el pwm, esta es la función encargada de hacer sonar una nota musical por un tiempo específico.
void play_tone(uint32_t frequency, uint32_t duration_ms)
{
    if (frequency == 0) {
        pwm_set_dt(&buzzer, PWM_HZ(100), 0);
    } else {
        uint32_t period = 1000000000U / frequency;
        pwm_set_dt(&buzzer, period, period / 2U);
    }
    k_sleep(K_MSEC(duration_ms));
    pwm_set_dt(&buzzer, PWM_HZ(100), 0); // Silencio
    k_sleep(K_MSEC(50)); // pausa entre notas
}

void play_victory_melody()
{
    play_tone(NOTE_C4, 150); //chatgptazo
    play_tone(NOTE_E4, 150);
    play_tone(NOTE_G4, 150);
    play_tone(NOTE_C5, 400);
}


int main(void)
{
    accel = DEVICE_DT_GET(ACCEL_NODE);
    disp  = mb_display_get();

    /* 1. Inicializar dispositivos */
    if (!device_is_ready(accel)) {
        printk("Acelerómetro no listo\n");
        return 0;
    }

    if (!disp) {
        printk("Display no listo\n");
        return 0;
    }

    if (!device_is_ready(buttonA.port)) {
        printk("Botón A no listo\n");
        return 0;
    }

    // inicia el buzzer
    if (!pwm_is_ready_dt(&buzzer)) {
        printk("Error: El Buzzer no está listo\n");
        return 0;
    }

    gpio_pin_configure_dt(&buttonA, GPIO_INPUT);

    //inicia bluetooth
    bt_enable(bt_ready);

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, "PODOMETRO", 9),
    };

    bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);

    printk("Podómetro iniciado con Bluetooth y Sonido\n");

    t0 = k_uptime_get_32();

    while (1) {

        //limpiar el display
        if (primera_vuelta) {
            mb_display_stop(disp);
            k_msleep(50);
            mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, 100, &blank_img, 1);
            k_msleep(50);
            primera_vuelta = 0;
        }

        //boton que configura la meta
        int val = gpio_pin_get_dt(&buttonA);

        if (val == 0 && last_button_state == 1) {
            meta_m += 100; // Incrementa meta
            meta_cumplida = 0; // reinicia estado de victoria

            // confirmacion al presionar boton
            play_tone(NOTE_C5, 50);

            char msg[25];
            sprintf(msg, "M:%d", meta_m);

            mb_display_stop(disp);
            k_msleep(50);
            mb_display_print(disp, MB_DISPLAY_MODE_SCROLL, 300, msg);
            printk("Meta actual: %d m\n", meta_m);
        }
        last_button_state = val;

        //lectura del acelerometro
        sensor_sample_fetch(accel);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &ax);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &ay);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &az);

        Ax = sensor_value_to_double(&ax);
        Ay = sensor_value_to_double(&ay);
        Az = sensor_value_to_double(&az);

        mag  = sqrt(Ax * Ax + Ay * Ay + Az * Az);
        diff = fabs(mag - gravity);

        t1 = k_uptime_get_32();

        //deteccion de los pasos
        if (diff > umbral && (t1 - t0) > 350) { 

            pasos++;
            t0 = t1;
            distancia = pasos * longitud_paso;

            double progreso = 0.0;
            if (meta_m > 0) {
                progreso = (distancia / meta_m) * 100.0;
            }

            printk("Pasos:%d Dist:%.2f Meta:%d Progreso:%.1f%%\n",
                   pasos, distancia, meta_m, progreso);

            sprintf(bt_buffer, "P:%d D:%.1f M:%d %%.2f\n", 
                    pasos, distancia, meta_m, progreso);

            if (!meta_cumplida) {
                char txt[15];
                sprintf(txt, "%d %.2f%%", pasos, progreso);
                mb_display_stop(disp); 
                mb_display_print(disp, MB_DISPLAY_MODE_SCROLL, 2000, txt);
            }
        }

        //si se cumple la meta suena el buzzer
        if (meta_m > 0 && distancia >= meta_m && meta_cumplida == 0) {

            meta_cumplida = 1;
            printk("¡¡ META CUMPLIDA !!\n");
            
            sprintf(bt_buffer, "META CUMPLIDA! %d pasos", pasos);

            mb_display_stop(disp);
            
            play_victory_melody();
           

            k_msleep(100);

            // Parpadeo visual
            for (int i = 0; i < 3; i++) {
                mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, 200, &meta_img, 1);
                k_msleep(200);
                mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, 200, &blank_img, 1);
                k_msleep(200);
            }
           // imagen final
            mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, SYS_FOREVER_MS, &meta_img, 1);
        }

        k_msleep(50);
    }
    return 0;
}
