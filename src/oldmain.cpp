#define MCU_MODE 1
#define IR_SEND_PIN 10
#define IR_RECEIVE_PIN 6
#define rxPin 4
#define txPin 5
#define RST_PIN         9           // Configurable, see typical pin layout above
#define SS_PIN          10          // Configurable, see typical pin layout above
#define lm35Pin                   //LM35 PIN
#define Flamepin 16                  //FLAME PIN
#define gasPin 17                    //GAS PIN
#define buzzer 20                   //buzzer bin
#define red_led 19                  //svn PIN
#define green_led 18                //svn PIN
#undef F_CPU
#define F_CPU 1000000UL
#include <Arduino.h>
#include <IRremote.hpp>
#include <SoftwareSerial.h>
#include <Keypad.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>
#include <avr/io.h>
#include <util/delay.h>
volatile byte passcount=9;
LiquidCrystal_I2C lcd(0x27,16,2);
#if MCU_MODE == 1
volatile bool admin_authorization=false;
volatile bool pass_enter=false;
String buff,pass;
SoftwareSerial sim800L(rxPin,txPin); 
MFRC522 mfrc522(SS_PIN, RST_PIN);
void waitForResponse();
void send_sms();
void make_call();
byte measureTemperature();
void FlameDetected();
void gasDetected();
void svn(byte data);
void terminate();
void alarm_off();
void alarm_on();
void password();
bool rfID();
void lcdprint();
volatile bool flame= false;
volatile bool gas = false;
volatile bool danger = false;
void ADC_init() {
    ADCSRA=0b10000011;
}

uint16_t ADC_read() {
    ADCSRA|=64;
    while(ADCSRA&64);
}
int main() {
    MCUCSR|=1<<JTD;
    MCUCSR|=1<<JTD;
    
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    // Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    //RFID
    SPI.begin();                                                  // Init SPI bus
    mfrc522.PCD_Init();                                           // Init MFRC522 card
    // Serial.println(F("Read personal data on a MIFARE PICC:"));    //shows in serial that it is ready to read
    //Flame
    
    // pinMode(Flamepin, INPUT);
    // attachInterrupt(digitalPinToInterrupt(Flamepin), FlameDetected, RISING);
    // //GAS HW408
    // pinMode(gasPin, INPUT);
    // attachInterrupt(digitalPinToInterrupt(gasPin), gasDetected, RISING);
    GICR |= (1<<6) | (1<<7);   
    MCUCR = 15;                
    sei();  
    DDRD |= (1 << GREEN_LED); 
    DDRD |= (1 << RED_LED);   
    DDRD |= (1 << BUZZER);    
    PORTD |= (1 << GREEN_LED); 
    digitalWrite(green_led,HIGH);
    lcd.init();
    lcd.backlight();
    while(1) {
    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            //Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
            //IrReceiver.printIRResultRawFormatted(&Serial, true);
            IrReceiver.resume();
        } else {
            IrReceiver.resume();
            //IrReceiver.printIRResultShort(&Serial);
            //IrReceiver.printIRSendUsage(&Serial);
        }
        //Serial.println();
        switch(IrReceiver.decodedIRData.command){
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
          case '*':
          case '#':
            if(pass_enter){
              pass+=(char)IrReceiver.decodedIRData.command;
              //Serial.println((String)pass);
              passcount++;
            }
            else{
              Serial.println("access denied");
              Serial.println(buff);
            }
            break;
          case 'A':
            if(admin_authorization){
            terminate();
            alarm_off();
            gas=false;
            flame=false;
            }
            break;
          case 'B':
            if(admin_authorization){
              alarm_on();
              make_call();
            }
            break;
          case 'C':
            admin_authorization=false;
            pass_enter=false;
            buff = "";
            pass="";
            break;
          case 'D':
            while(!rfID());
            if(!pass_enter&&(buff == "Moslem Sayed "||buff == "Mina Magdy")){
              pass_enter=true;
              pass="";
              lcd.clear();
              lcd.print(buff);
              lcd.setCursor(0,1);
              lcd.print("accepted");
              delay(2000);
              passcount=0;
              delay(2000);
            }
            else{
              lcd.clear();
              lcd.print(buff);
              lcd.setCursor(0,1);
              lcd.print("rejected");
              delay(1000);
            }
            break;
          default:
            break;
        }
    }
    if(pass=="123456789"&&!admin_authorization){
      admin_authorization=true;
      //Serial.println("autinticated");
      lcd.clear();
      lcd.print("autinticated");
      _delay_ms(1000);
      pass_enter=false;
    }
    else if(pass_enter&&passcount==9){
      lcd.clear();
      lcd.print("wrong");
      pass="";
      passcount=0;
    }
    lcdprint();
    _delay_ms(200);
}
    //pinMode(A3,OUTPUT);
}
void send_sms(){
  //sim800L.print("AT+CMGS=\"+201027457807\"\r");
  //waitForResponse();
  
  //sim800L.print("Hello from SIM800L");
  //sim800L.write(0x1A);
  //waitForResponse();
}
void terminate(){
  //sim800L.print("ATH");
  //waitForResponse();
}

void make_call(){
  //sim800L.println("ATD+201551228625;");
  //waitForResponse();
}

void waitForResponse(){
  delay(1000);
//   while(sim800L.available()){
//     Serial.println(sim800L.readString());
//   }
//   sim800L.read();
 }
/////////////////////////////Temperature function////////////////////
byte measureTemperature(){
  return (*500)/1023;
}

void FlameDetected(){ 
  alarm_on();
  delay(1000);
  // delay(1000);
  // lcd.clear();
  // lcd.print("danger!!");
  // lcd.setCursor(0,1);
  // lcd.print("call firefighter");
  make_call();
  digitalWrite(A3,HIGH);
  flame=true;
}

void gasDetected() {
  alarm_on();
  delay(1000);
  make_call();
  digitalWrite(A3,HIGH);
  gas=true;
}

bool rfID(){
  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
  //some variables we need
  byte block;
  byte len;
  MFRC522::StatusCode status;
  //-------------------------------------------
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return 0;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return 0;
  }
  Serial.println(F("*Card Detected:*"));
  //-------------------------------------------
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); //dump some details about the card
  //mfrc522.PICC_DumpToSerial(&(mfrc522.uid));      //uncomment this to see all blocks in hex
  //-------------------------------------------
  Serial.print(F("Name: "));

  byte buffer1[18];

  block = 4;
  len = 18;

  //------------------------------------------- GET FIRST NAME
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 4, &key, &(mfrc522.uid)); //line 834 of MFRC522.cpp file
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    digitalWrite(A3,HIGH);
    return 0;
  }

  status = mfrc522.MIFARE_Read(block, buffer1, &len);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    digitalWrite(A3,HIGH);
    return 0;
  }
  //PRINT FIRST NAME
  for (uint8_t i = 1; i < 16; i++)
  {
    if (buffer1[i] != 32)
    {
      Serial.write(buffer1[i]);
      buff+=(char)buffer1[i];
    }
  }
  Serial.print(" ");
  buff +=" ";
  //---------------------------------------- GET LAST NAME
  byte buffer2[18];
  block = 1;
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(mfrc522.uid)); //line 834
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    digitalWrite(A3,HIGH);
    return 0;
  }
  status = mfrc522.MIFARE_Read(block, buffer2, &len);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
    digitalWrite(A3,HIGH);
    return 0;
  }
  //PRINT LAST NAME
  for (uint8_t i = 1; i < 16; i++) {
    if (buffer1[i] != 32){
      Serial.write(buffer2[i]);
      buff+=(char)buffer2[i];
    }
  }
  //----------------------------------------
  Serial.println(F("\n*End Reading*\n"));
  delay(1000); //change value if you want to read cards faster
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  return 1;
}
void alarm_on(){
  digitalWrite(red_led,HIGH);
  digitalWrite(buzzer,HIGH);
  digitalWrite(green_led,LOW);
  // delay(1000);
  // lcd.clear();
  // lcd.print("danger!!");
  // lcd.setCursor(0,1);
  // lcd.print("call firefighter");
}
void alarm_off(){
  digitalWrite(red_led,LOW);
  digitalWrite(buzzer,LOW);
  digitalWrite(green_led,HIGH);
}
void lcdprint(){
  if(flame||gas)
  {
  lcd.clear();
  lcd.print("danger!!");
  lcd.setCursor(0,1);
  lcd.print("call firefighter");
  return;
  }
  if(passcount==9){
    lcd.clear();
    lcd.print("temprature:");
    lcd.print(measureTemperature());
    lcd.setCursor(0,1);
    lcd.print("gas:no flame:no");
  }
  else{
    lcd.clear();
    lcd.print("password");
    lcd.setCursor(0,1);
    lcd.print(pass);
  }
}
#else
const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
{'1','2','3','A'},
{'4','5','6','B'},
{'7','8','9','C'},
{'*','0','#','D'}
};
byte rowPins[ROWS] = {5, 4, 3, 2}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {9, 8, 7, 6}; //connect to the column pinouts of the kpd

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.print(F("Send IR signals at pin "));
    Serial.println(IR_SEND_PIN);
    IrSender.begin();
    disableLEDFeedback();
}

String password;
void loop() {
    if (kpd.getKeys())
    {
        for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
        {
            if ( kpd.key[i].stateChanged )   // Only find keys that have changed state.
            {
                if (kpd.key[i].kstate==PRESSED){
                IrSender.sendNEC(0x00,(int)kpd.key[i].kchar,0);
                Serial.println(kpd.key[i].kchar);
                }
            }
        }
    }
    //Serial.flush();
    delay(100);
}
#endif

// void svn(byte data) {
//     pinMode(svnPin1, 1);
//     pinMode(svnPin2, 1);
//     pinMode(svnPin3, 1);
// 	int x = 0, temp;
// 	unsigned char num[10]={63,6,91,79,102,109,125,7,127,111},counter,counter2,mag;
//     while(1)
//     {
//         temp=x;
//         for(counter=0;counter<3;counter++){
//             mag=temp%10;
//             mag=~num[mag];
// 			for(counter2=0;counter2<8;counter2++){
// 				//delay(10);
// 				// PORTC&=~(1<<1);
//                 digitalWrite(svnPin2, LOW);
// 				//delay(10);
// 				// PORTC|=(mag%2)<<1;
//                 digitalWrite(svnPin2, (mag % 2));
// 				//delay(10);
// 				// PORTC&=~1;
//                 digitalWrite(svnPin1, 0);
// 				//delay(10);
// 				// PORTC|=1;
//                 digitalWrite(svnPin1, 1);
// 				//delay(10);
// 				mag/=2;
// 				//delay(10);
// 			}
// 			temp/=10;
// 			//delay(10);
// 			// PORTC&=~(1<<2);
//             digitalWrite(svnPin3, 0);
// 			//delay(10);
// 			// PORTC|=(1<<2);
//             digitalWrite(svnPin3, 1);
// 			//delay(10);
//         }

//         if(x==999)
// 			x=0;
// 		else
//             x++;
// 		// PORTC=0;
//         digitalWrite(svnPin1, 0);
//         digitalWrite(svnPin2, 0);
//         digitalWrite(svnPin3, 0);
// 		delay(50);
//     }
// }