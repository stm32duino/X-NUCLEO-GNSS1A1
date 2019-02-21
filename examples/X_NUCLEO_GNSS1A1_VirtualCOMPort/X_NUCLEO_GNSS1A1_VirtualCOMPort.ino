volatile uint8_t fromPC[16];
volatile uint8_t fromGNSS[16];
volatile unsigned long idxPC = 0;
volatile unsigned long idxGNSS = 0;

#ifdef ARDUINO_ARCH_STM32
HardwareSerial Serial1(PA10, PA9);
#endif

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  delay(1000);
  digitalWrite(7,HIGH);
  delay(50);
}


void loop() {
  if(Serial.available()){
    fromPC[idxPC]= Serial.read();
    Serial1.write(fromPC[idxPC]);
    idxPC++;
    idxPC %= 16;
    }
  if(Serial1.available()){
    fromGNSS[idxGNSS]= Serial1.read();
    Serial.write(fromGNSS[idxGNSS]);
    idxGNSS++;
    idxGNSS %= 16;
    }
}
