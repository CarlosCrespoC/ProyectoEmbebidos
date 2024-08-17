  #include <Arduino.h>
  #include <ESP32Servo.h>
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  #include <BluetoothSerial.h>


  #define I2C_SDA 21
  #define I2C_SCL 22

  BluetoothSerial SerialBT;

  Servo servo1;
  Servo servo2;
  int joyX = 36;
  int joyY = 39;

  int servoVal1;
  int servoVal2;

  LiquidCrystal_I2C lcd(0x27, 16, 2);

  int ledEncendido = 32;
  int ledApagado = 33;

  int pulsadorEncendido = 23;
  int pulsadorApagado = 19;
  int pulsadorVictoria = 18;

  int estado = 1;
  int punt = 0;
  int dificultad = 1;
  bool victoria = false;
  unsigned long startMillis = 0;
  unsigned long elapsedMillis = 0;
  bool cronometroActivo = false;

  unsigned long hours = 0;
  unsigned long minutes = 0;
  unsigned long seconds = 0;

  volatile const char* puntuacion;

  byte arrowUp[8] = { 0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000 };
  byte arrowDown[8] = { 0b00100, 0b00100, 0b00100, 0b00100, 0b10101, 0b01110, 0b00100, 0b00000 };
  byte arrowLeft[8] = { 0b00100, 0b00110, 0b00111, 0b11111, 0b00111, 0b00110, 0b00100, 0b00000 };
  byte arrowRight[8] = { 0b00100, 0b01100, 0b11100, 0b11111, 0b11100, 0b01100, 0b00100, 0b00000 };

  void IRAM_ATTR ISR_encender() {
    estado = 1;
    victoria = false;
    if (!cronometroActivo) {
      startMillis = millis() - elapsedMillis;
      cronometroActivo = true;
    }
  }

  void IRAM_ATTR ISR_apagar() {
    estado = 2;
    cronometroActivo = false;
  }
  void IRAM_ATTR ISR_Victoria() {
    cronometroActivo = false;
    victoria = true;
    estado = 4;
    hours = 0;
    minutes = 0;
    seconds = 0;
    unsigned long totalSeconds = (elapsedMillis / 1000);
    unsigned long totalminutes = totalSeconds / 60;
    unsigned long totalHours = totalSeconds % 60;
    
        if (totalminutes >= 10) {
          punt = 1;
        } else if (totalminutes >= 5 && totalminutes < 10) {
          punt = 2;
        } else {
          punt = 3;
        }

    elapsedMillis = 0;

  }

  int ajustarServo(int valor) {
    if (valor < 70) {
      return max(valor - 5, 0);  // Restar 10 grados, asegurando que no sea menor que 0
    } else if (valor > 80) {
      return min(valor + 5, 180);  // Sumar 10 grados, asegurando que no sea mayor que 180
    } else {
      return 90;  // Si está entre 100 y 120, fijar en 90 grados
    }
  }

  int ajustarServo2(int valor) {
    if (valor < 85) {
        return 75;  // Limitar el valor mínimo a 75
    } else if (valor > 105) {
        return 105;  // Limitar el valor máximo a 105
    } else {
        return valor;  // Si está entre 75 y 105, devolver el valor tal como está
    }
}

  void controlServosTask(void *parameter);
  void updateLCDTask(void *parameter);
  void cronometroTask(void *parameter);

  void setup() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.begin(115200);
    SerialBT.begin("ESP32_BT_CC");

    pinMode(ledEncendido, OUTPUT);
    pinMode(ledApagado, OUTPUT);
    pinMode(pulsadorEncendido, INPUT_PULLDOWN);
    pinMode(pulsadorApagado, INPUT_PULLDOWN);
    pinMode(pulsadorVictoria, INPUT_PULLDOWN);
    servo1.attach(25);
    servo2.attach(26);

    lcd.createChar(0, arrowUp);
    lcd.createChar(1, arrowDown);
    lcd.createChar(2, arrowLeft);
    lcd.createChar(3, arrowRight);

    lcd.begin(16,2);
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("Bienvenidos ");
    lcd.setCursor(0, 1);
    lcd.print("al laberinto!");
    delay(2000);
    lcd.clear();

    attachInterrupt(digitalPinToInterrupt(pulsadorEncendido), ISR_encender, RISING);
    attachInterrupt(digitalPinToInterrupt(pulsadorApagado), ISR_apagar, RISING);
    attachInterrupt(digitalPinToInterrupt(pulsadorVictoria), ISR_Victoria, RISING);
    
    
    xTaskCreatePinnedToCore(controlServosTask, "Control Servos", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(updateLCDTask, "Update LCD", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(cronometroTask, "Cronometro", 2048, NULL, 1, NULL, 1);

    estado = 3;
  }

  void loop() {
  }

  void controlServosTask(void *parameter) {
    while (1) {
        if (SerialBT.available()) {
          char incomingChar = SerialBT.read();
          Serial.print("Valor de dificultad: ");
          Serial.println(incomingChar);
          if (incomingChar == '3') { // Dificil dificultad
              Serial.println("Dificultad Dificil");
              dificultad = 3;
          } else if (incomingChar == '2') { // Mediana dificultad
              Serial.println("Dificultad Media");
              dificultad = 2;
          } else {
              Serial.println("Dificultad Fácil");
              dificultad = 1;
          }
          
      }
        servoVal1 = analogRead(joyX);
        servoVal1 = map(servoVal1, 0, 4095, 60, 105);
        //servoVal1 = ajustarServo2(servoVal1);
        servo1.write(servoVal1);

        // Leer y mapear el valor del joystick Y para el servo2
        servoVal2 = analogRead(joyY);
        servoVal2 = map(servoVal2, 0, 4095, 180, 0);
        servoVal2 = ajustarServo(servoVal2);
        servo2.write(servoVal2);

        // Debug para verificar valores en el Serial Monitor
        Serial.print("JoyX: ");
        Serial.println(servoVal1);
        Serial.print("JoyY: ");
        Serial.println(servoVal2);
        Serial.println();

        if (dificultad == 3) { // Dificil dificultad
            vTaskDelay(1000 / portTICK_PERIOD_MS);    
          } else if (dificultad == 2) { // Mediana dificultad
            vTaskDelay(750 / portTICK_PERIOD_MS);
          }
    }
  }

  void updateLCDTask(void *parameter) {
    while (1) {
      if (estado == 1) {
        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, LOW);
        lcd.setCursor(0, 0);
        lcd.print("Estado: ON        ");
        lcd.setCursor(0, 1);
        lcd.print("                 ");
        lcd.setCursor(0, 1);
        lcd.print(hours < 10 ? "0" : ""); lcd.print(hours);
        lcd.print(":");
        lcd.print(minutes < 10 ? "0" : ""); lcd.print(minutes);
        lcd.print(":");
        lcd.print(seconds < 10 ? "0" : ""); lcd.print(seconds);

      } else if (estado == 2) {
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, HIGH);
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 0);
        lcd.print("Estado: OFF    ");
        lcd.setCursor(0, 1);
        lcd.print("Pausa!");

      } else if (estado == 3) {
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, LOW);
        lcd.setCursor(0, 0);
        lcd.print("Presiona el: ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        lcd.setCursor(0, 1);
        lcd.print("Boton de inicio");


      } else if (estado == 4) { 

        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, LOW);

        lcd.setCursor(0, 0);
        lcd.print("Felicitaciones");
        lcd.setCursor(0, 1);
        lcd.print("Haz ganado!");
        lcd.setCursor(15, 0);
        lcd.print(" ");
        lcd.setCursor(15, 1);
        lcd.print(" ");

        // Imprimir mensaje de victoria
        lcd.setCursor(0, 0);
        lcd.print("Felicitaciones!");
        lcd.setCursor(0, 1);
        lcd.print("Punt: ");
        
        if(punt == 1){
        lcd.print("Normal");
        } else if(punt == 2){
        lcd.print("Crack");
        } else {
        lcd.print("Legendario");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);

        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, HIGH);
        digitalWrite(ledApagado, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(ledEncendido, LOW);
        digitalWrite(ledApagado, LOW);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Disfruta tu dia!");
        lcd.setCursor(0, 1);
        lcd.print("Sigue asi!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    }
  }

  void cronometroTask(void *parameter) {
    while (1) {
      if (cronometroActivo) {
        unsigned long currentMillis = millis();
        elapsedMillis = currentMillis - startMillis;

        seconds = (elapsedMillis / 1000) % 60;
        minutes = (elapsedMillis / (1000 * 60)) % 60;
        hours = (elapsedMillis / (1000 * 60 * 60)) % 24;
      }

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  
