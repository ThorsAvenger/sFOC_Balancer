#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include "esp_wifi.h"
#include <vector>

typedef struct struct_velocity {
    float vel_L;
    float vel_R;
} struct_velocity;

typedef struct struct_mot_data {
    int m_id;
    float pitch;
    float voltage;
} struct_mot_data;

float getPitchMsg();
struct_velocity getMsg();
void sendData(struct_velocity velo);
void sendMotorData(struct_mot_data motor);
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len); 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void espNowInit();
int msgData();