#include "ESP_NOW_helper.h"


esp_now_peer_info_t peerInfo;

int msgFlag = 0;

uint8_t broadcastAddress[] = {0xE0,0xE2,0xE6,0x70,0x74,0xF0}; //{0xAC, 0x67, 0xB2, 0x59, 0xAB, 0xB8};

struct_velocity data_in;

// put function declarations here:
// void espNowInit();
float getPitchMsg() {
    msgFlag = 0;
    return data_in.vel_L;
  }

  struct_velocity getMsg() {
    msgFlag = 0;
    return data_in;
  }

int msgData() {
    return msgFlag;
  }

void sendData(struct_velocity velo) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&velo, sizeof(velo));
    // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outputPosition, sizeof(outputPosition));
  }
  
  void sendMotorData(struct_mot_data motor) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&motor, sizeof(motor));
    
    // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outputPosition, sizeof(outputPosition));
  } 

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&data_in, incomingData, sizeof(data_in));
    msgFlag = 1;
    //Serial.print("Bytes received: ");
    //Serial.println(len);
    // Serial.printf("Value1:%.2f, Value2:%.2f\r\n", data_in.vel_L, data_in.vel_R);
    // Serial.printf("Value1, %.2f\r\n", data_in.vel_L);
  
    // motorAngle = inputPosition.value;
  }
  
  // Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    /*if (status == 0){
      String success = "Delivery Success :)";
    }
    else{
      String success = "Delivery Fail :(";
    }*/
  }
  
  void espNowInit() {
    // Init ESP-NOW
    // WiFi.mode(WIFI_STA);
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
  
    esp_now_register_recv_cb(OnDataRecv);
  
    esp_now_register_send_cb(OnDataSent);
  
    // Register peer
    //esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

  
    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }