//参考にしたサイト
//https://qiita.com/MedakanoGakko/items/e9624c3fa45e40b0425e

// VCC：3.3V
#define AIN1 2
#define AIN2 3
#define BIN1 8
#define BIN2 6
#define PWMA 10
#define PWMB 5
#define STBY 4

void setup() {
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    digitalWrite(STBY, HIGH); // スタンバイ
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
}
void loop() {
//    // 左回り
//    digitalWrite(AIN1, HIGH);
//    digitalWrite(AIN2, LOW);
//    digitalWrite(BIN1, HIGH);
//    digitalWrite(BIN2, LOW);
//    delay(1000);
//    
//    // 右回り
//    digitalWrite(AIN1, LOW);
//    digitalWrite(AIN2, HIGH);
//    digitalWrite(BIN1, LOW);
//    digitalWrite(BIN2, HIGH);
//    delay(1000);
//    
    // 後進
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    delay(1000);

    // 前進
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    delay(1000);
    
    // 回転速度を設定（0～255）まで
    analogWrite(PWMA, 50);
    analogWrite(PWMB, 50);
}
