#include <esp_now.h>
#include <WiFi.h>
#include <ModbusRTU.h>
#include <Keypad.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

#define RS485_ENABLE 1

#define EEPROM_SIZE 100
//EEPROM data ADDRESSER
#define EEP_ADDRESS 0
#define EEP_TICKET_TYPE 1
#define EEP_MASTER_CONSOLE 2

#define RS485_DISPLAY_CALL 0xA1

#define RXD2 16
#define TXD2 17
#define RS485_PIN_DIR 4
HardwareSerial rs485(1);
#define RS485_WRITE     1
#define RS485_READ      0

static uint8_t CRC_intex = 0;
static uint16_t CRC_receive = 0xffff;
#define CRC_divisor	 0x8005

//uint8_t broadcastConsole1[] = { 0x30, 0xC6, 0xF7, 0xEA, 0xE5, 0x44 }; //Master 30:C6:F7:EA:E5:44
uint8_t broadcastConsole1[] = { 0x94, 0x3C, 0xC6, 0x82, 0x18, 0xA4 };//Slave
uint8_t broadcastAddress1[] = { 0x0C, 0x8B, 0x95, 0x70, 0xFE, 0xE4 }; //0C:8B:95:70:FE:E4 μαστερ 30:C6:F7:EA:E5:44
uint8_t broadcastAddress2[] = { 0xB8, 0xD6, 0x1A, 0x35, 0x4D, 0x48 };
String success; //10:97:BD:D4:59:C4
esp_err_t result;

String recalls[] = { "000", "000", "000" };
char queue_characters[5] = { 0,0,0,0 };

char key = 0xff;

uint8_t address;
uint8_t master;

// Define variables to store Queue to be sent
uint16_t queue = 0;
String out_queue;
uint8_t out_counter;
String out_category = "A";

// Define variables to store Queue readings
String in_queue;
uint8_t in_counter;
char in_category;

typedef struct struct_message {
    String queue;
    String device;
    String instruction;
    uint8_t counter;
    String category;
} struct_message;

struct_message Queue_senting;
struct_message Queue_receive;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == 0) {
        success = "Delivery Success :)";
    }
    else {
        success = "Delivery Fail :(";
    }
}

const int rs = 5, en = 27, d4 = 26, d5 = 25, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define ROW_NUM     4 // four rows
#define COLUMN_NUM  4 // four columns
const char Sorft_ID[] PROGMEM = "Console_now_V4.1";
const char Company[] PROGMEM = " Rousis Systems ";

char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pin_rows[ROW_NUM]      = {35, 34, 39, 36}; // GIOP19, GIOP18, GIOP5, GIOP17 connect to the row pins
byte pin_column[COLUMN_NUM] = {14, 12, 13, 15};   // GIOP16, GIOP4, GIOP0, GIOP2 connect to the column pins

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM );
String key_state;

// Callback when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&Queue_receive, incomingData, sizeof(Queue_receive));
    Serial.print("Bytes received: ");
    Serial.println(len);
    out_queue = Queue_receive.queue;
    out_counter = Queue_receive.counter;
    out_category = Queue_receive.category;

    Serial.print("Queue Number: ");
    Serial.println(out_queue);
    Serial.print("Counter: ");
    Serial.println(out_counter);
    Serial.print("Category: ");
    Serial.println(out_category);
    Serial.print("Device: ");
    Serial.println(Queue_receive.device);
    Serial.print("Instruction: ");
    Serial.println(Queue_receive.instruction);
    Serial.println("----------------------------------");

    if (Queue_receive.instruction == "CALLED" && !master)
    {
        lcd.setCursor(0, 0);
        lcd.print("Called: ");
        lcd.print(Queue_receive.queue);
    }
    else if (Queue_receive.instruction == "SLAVE_CALL" && master)
    {
        //queue++;
        queue = inc_cueue(queue);
        update_queue("CALL", Queue_receive.counter);
    }
    else if ("SLAVE_RECALL")
    {
        recall(Queue_receive.counter);
    }
}

void setup() {
  Serial.begin(115200);
  rs485.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Project      :  Arduino_ESP32_RS485");
  Serial.print("Info: Intial gpio...");
  pinMode(RS485_PIN_DIR, OUTPUT);
  digitalWrite(RS485_PIN_DIR, RS485_READ);
  Serial.println("done");

  delay(100);
  lcd.begin(16, 2);

  EEPROM.begin(EEPROM_SIZE);
  address = EEPROM.read(EEP_ADDRESS);
  master = EEPROM.read(EEP_MASTER_CONSOLE);

  keypad.setHoldTime(3000);                   // Default is 1000mS

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  } else {
      Serial.println("initialized ESP - NOW");
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer 1");
      return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer 2");
      return;
  }

  //Connect with other console
  memcpy(peerInfo.peer_addr, broadcastConsole1, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer 2");
      return;
  }
  

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println(Sorft_ID);
  Serial.println(Company);

  lcd.clear();
  lcd.print(Sorft_ID);
  lcd.setCursor(0, 1);
  lcd.print(Company);
  delay(2000);
  lcd.clear();
  //uint8_t A = EEPROM.read(EEP_MASTER_CONSOLE)
  if (EEPROM.read(EEP_MASTER_CONSOLE))
  {
      lcd.print("MASTER CONSOLE");
  }
  else 
  {
      lcd.print("SLAVE CONSOLE");
  }
  lcd.setCursor(0, 1);
  lcd.print("Address: ");
  lcd.print(address);
  delay(2000);
  lcd.clear();

  update_queue("UPDATE",0);
}

void loop() {

    if (keypad.getKeys())
    {
        for (int i = 0; i < LIST_MAX; i++)   // Scan the whole key list.
        {
            if (keypad.key[i].stateChanged)   // Only find keys that have changed state.
            {
                switch (keypad.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
                case PRESSED:
                    key_state = "PRESSED";
                    break;
                case HOLD:
                    key_state = "HOLD";
                    break;
                case RELEASED:
                    key_state = "RELEASED";
                    break;
                case IDLE:
                    key_state = "IDLE";
                }
                key = keypad.key[i].kchar;

                if (key == 'D' && key_state == "PRESSED")
                {

                    lcd.setCursor(0, 1);
                    Serial.println("Call Next");
                    lcd.print("Call Next");
                    if (master)
                    {
                        //queue++;
                        queue = inc_cueue(queue);
                        update_queue("CALL",address);
                        lcd.setCursor(0, 1);
                        lcd.print("CALL            ");
                    }
                    else 
                    {
                        Queue_senting.queue = "";
                        Queue_senting.device = "CONSOLE";
                        Queue_senting.instruction = "SLAVE_CALL";
                        Queue_senting.counter = address;
                        Queue_senting.category = "";
                        result = esp_now_send(broadcastConsole1, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
                        lcd.clear();
                        lcd.setCursor(0, 1);
                        if (result == ESP_OK) {
                            Serial.println("Send CALL");
                            lcd.print("Send Call");
                        }
                        else 
                        {
                            Serial.println("Error sending");
                            lcd.print("Error sending...");
                        }
                    }             
                }
                else if (key == 'B' && key_state == "PRESSED")
                {
                    if (master)
                    {
                        set_queue();
                    }                    
                }
                else if (key == 'C' && key_state == "PRESSED")
                {
                    if (master)
                    {
                        recall(address);
                        lcd.setCursor(0, 1);
                        lcd.print("RECALL          ");
                    }
                    else 
                    {
                        Queue_senting.queue = "";
                        Queue_senting.device = "CONSOLE";
                        Queue_senting.instruction = "RECALL";
                        Queue_senting.counter = address;
                        Queue_senting.category = "";
                        result = esp_now_send(broadcastConsole1, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
                        lcd.setCursor(0, 1);
                        if (result == ESP_OK) {
                            Serial.println("Send RECALL");
                            lcd.print("Send reCall");
                        }
                        else
                        {
                            Serial.println("Error sending");
                            lcd.print("Error sending recall");
                        }
                    }
                    
                }
                else if (key == '*' && key_state == "PRESSED" && master)
                {
                    queue--;
                    update_queue("BACK",address);
                    lcd.setCursor(0, 1);
                    lcd.print("BACK            ");
                }
                else if (key == 'A' && key_state == "HOLD")
                {
                    settings();
                }
            }
        }
    }
  //key = keypad.getKey();
}

uint16_t inc_cueue(uint16_t counter) {
    uint8_t type = EEPROM.read(EEP_TICKET_TYPE);
    counter++;
    if (type==1 && counter > 499) {
        return 0;
    }
    else if (type == 0 &&  counter > 9999)
    {
        return 0;
    }
    else
    {
        return counter;
    }
}

void settings(void) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Settings:");

    lcd.setCursor(0, 0);

    uint8_t B = EEPROM.read(EEP_ADDRESS);
    if (B > 9) { B = 1; }
    lcd.setCursor(0, 1);
    lcd.print("Counter: ");
    lcd.print(B);
    lcd.setCursor(9, 1);
    lcd.blink();

    while (key != 'D')
    {
        key = keypad.getKey();
        if (key == 'A')
        {
            lcd.noBlink();
            update_queue("UPDATE",0);
            return;
        }     
        else if (key > 0x30 && key <= 0x39)
        {
            B = (key & 0x0f);
            lcd.print(key);
            lcd.setCursor(9, 1);
        }
    }
    EEPROM.write(EEP_ADDRESS, B);
    EEPROM.commit();
    address = B;
    lcd.noBlink();
    key = 0xFF;

    lcd.setCursor(0, 1);
    B = EEPROM.read(EEP_TICKET_TYPE);
    lcd.print("Ticket letter: ");
    if (B){ lcd.print('Y' ); } else { lcd.print('N'); }
    lcd.setCursor(15, 1);
    while (key != 'D')
    {
        key = keypad.getKey();
        if (key == 'A')
        {
            update_queue("UPDATE",0);
            return;
        }
        else if (key == '#')
        {
            B = 1;
            lcd.print('Y');
            lcd.setCursor(15, 1);
        }
        else if (key == '*')
        {
            B = 0;
            lcd.print('N');
            lcd.setCursor(15, 1);
        }
    }
    EEPROM.write(EEP_TICKET_TYPE, B);
    EEPROM.commit();
    key = 0xFF;
    
    lcd.setCursor(0, 1);
    B = EEPROM.read(EEP_MASTER_CONSOLE);
    lcd.print("Master consle: ");
    if (B) { lcd.print('Y'); } else { lcd.print('N'); }
    lcd.setCursor(15, 1);
    while (key != 'D')
    {
        key = keypad.getKey();
        if (key == 'A')
        {
            update_queue("UPDATE",0);
            return;
        }
        else if (key == '#')
        {
            B = 1;
            lcd.print('Y');
            lcd.setCursor(15, 1);
        }
        else if (key == '*')
        {
            B = 0;
            lcd.print('N');
            lcd.setCursor(15, 1);
        }
    }
    EEPROM.write(EEP_MASTER_CONSOLE, B);
    EEPROM.commit();
    master = B;

    lcd.clear();
    update_queue("UPDATE",0);
}

uint8_t settings_select(char) {

}

void set_queue(void) {
    char characters[6];
    char set_chars[5];
    uint8_t type = EEPROM.read(EEP_TICKET_TYPE);
    itoa(queue, characters, 10);
    uint8_t len = 0;
    size_t i;

    //debug_buf_print(characters);

    while (characters[len]) {
        len++;
    }

    for (i = 0; i < 4; i++)
    {
        set_chars[i] = '0';
    }
    for (i = 0; i < len; i++)
    {
        set_chars[(4-len)+i] = characters[i];
    }
    set_chars[4] = 0;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set Q: ");
    lcd.print(set_chars);
    lcd.setCursor(7, 0);
    lcd.blink();
    i = 0;
    key = 0xff;

    while (key != 'B')
    {
        key = keypad.getKey();

        if (key >= 0x30 && key <= 0x39)
        {
            if (type && i==0){key = 0x30;}
            set_chars[i++] = key;
            lcd.setCursor(7, 0);
            lcd.print(set_chars);
            if (i > 3) { i = 3;}
            lcd.setCursor(7 + i, 0);
            //debug_buf_print(set_chars);
        }
        else if (key == '*' && i > 0)
        {
            i--;
            lcd.setCursor(7 + i, 0);
        }
        else if (key == '#' && i < 3)
        {
            i++;
            lcd.setCursor(7 + i, 0);
        }
        else if (key == 'A')
        {
            lcd.noBlink();
            lcd.clear();
            update_queue("UPDATE",0);
            return;
        }
    }
    //debug_buf_print(set_chars);

    queue = atoi(set_chars);
    if (type && queue >= 500) {queue = 0;}
    lcd.noBlink();
    lcd.clear();
    update_queue("UPDATE",0);
}

void update_queue(String Instr, uint8_t consol_address) {
    lcd.clear();
    char characters[5] = {0,0,0,0,0};
    itoa(queue, characters,  10);
    if (EEPROM.read(EEP_TICKET_TYPE))
    {
        if (characters[1]==0)
        {
            characters[2] = characters[0];
            characters[1] = '0';
            characters[0] = '0';

        }else if (characters[2] == 0){
            characters[2] = characters[1];
            characters[1] = characters[0];
            characters[0] = '0';
        }

        switch (characters[0])
        {
        case '0':
            characters[0] = 'A';
            break;
        case '1':
            characters[0] = 'B';
            break;
        case '2':
            characters[0] = 'C';
            break;
        case '3':
            characters[0] = 'D';
            break;
        case '4':
            characters[0] = 'E';
            break;
        default:
            characters[0] = 'A';
            break;
        }
    }

    for (size_t i = 0; i < sizeof(queue_characters); i++)
    {
        queue_characters[i] = characters[i];
    }

    if (consol_address <= 1)
    {
        lcd.setCursor(0, 0);
        lcd.print("Queue: ");
        lcd.print(characters);
    }
    Queue_senting.queue = String(characters);
    Queue_senting.device = "CONSOLE";
    Queue_senting.instruction = Instr;
    Queue_senting.counter = consol_address;
    Queue_senting.category = out_category;

    if (Instr == "CALL")
    {    
        recalls[consol_address] = Queue_senting.queue;

        result = esp_now_send(broadcastAddress1, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
        lcd.setCursor(0, 1);
        if (result == ESP_OK) {
            Serial.println("Updated dipsplay 1");
        }
        else {
            Serial.println("Error display 1");
            lcd.print("Error display 1");
        }

#if RS485_ENABLE
        if (RS485_display_call(queue_characters, RS485_DISPLAY_CALL, 1))
        {
            Serial.println("Succesfuly RS485 CALL dipsplay 1");
        } else {
            Serial.println("Error on RS485 CALL dipsplay 1...");
        }        
#endif // RS485_ENABLE


        //result = esp_now_send(broadcastAddress2, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
        result = esp_now_send(broadcastAddress2, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
        lcd.setCursor(0, 1);
        if (result == ESP_OK) {
            Serial.println("Updated dipsplay 2");
        }
       else {
            Serial.println("Error display 2");
           lcd.print("Error display 2");
        }

        if (consol_address != 1)
        {
            //Send answer to slave console
            Queue_senting.instruction = "CALLED",
                result = esp_now_send(broadcastConsole1, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
            lcd.setCursor(0, 1);
            if (result == ESP_OK) {
                Serial.println("Updated slave console");
            }
            else {
                Serial.println("Error console 2");
                lcd.print("Error console 2");
            }
        }
    }
}

void recall(uint8_t console_address) {
    Queue_senting.queue = recalls[console_address];
    Queue_senting.device = "CONSOLE";
    Queue_senting.instruction = "CALL";
    Queue_senting.counter = console_address;
    Queue_senting.category = "1";

    result = esp_now_send(broadcastAddress1, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
    lcd.setCursor(0, 1);
    if (result == ESP_OK) {
        Serial.println("Updated dipsplay 1");
    }
    else {
        Serial.println("Error display 1");
        lcd.print("Error display 1");
    }
    //result = esp_now_send(broadcastAddress2, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
    result = esp_now_send(broadcastAddress2, (uint8_t*)&Queue_senting, sizeof(Queue_senting));
    lcd.setCursor(0, 1);
    if (result == ESP_OK) {
        Serial.println("Updated dipsplay 2");
    }
    else {
        Serial.println("Error display 2");
        lcd.print("Error display 2");
    }
}

void debug_buf_print(char *buf) {

    Serial.print("Buffer: ");
    for (size_t i = 0; i < sizeof(buf); i++)
    {
        Serial.print(buf[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println("----------------------------------------");
}

bool RS485_display_call(char* buf, uint8_t instr,uint8_t disp_add) {
    /*Example of sending Values :
    [01, 55, AA] [Address(default 01)] [A1] “1234”[04]*/
    byte get_byte;
    while (rs485.available() > 0) { get_byte = rs485.read(); }
    char header_pckt[] = { 0x1, 0x55, 0xAA, disp_add, instr };
    CRC_receive = 0xffff;
    digitalWrite(RS485_PIN_DIR, RS485_WRITE);

    for (size_t i = 0; i < sizeof(header_pckt); i++)
    {
        rs485.write(header_pckt[i]);
        Put_CRC(header_pckt[i]);
    }

    for (size_t i = 0; i < sizeof(buf); i++)
    {
        rs485.write(buf[i]);
        Put_CRC(buf[i]);
    }

    rs485.write(0x4);
    Put_CRC(4);
    rs485.flush();
    digitalWrite(RS485_PIN_DIR, RS485_READ);

    CRC_receive = 0xffff;
    char reply_pckt[5] = { 0xAA, 0x55, 'O', 'K', '!' };
    while (rs485.available() > 0) { get_byte = rs485.read(); }
    //Serial.println("Start receiving packet...");
    unsigned long startedWaiting = millis();
    while (millis() - startedWaiting <= 100 && rs485.available() < 7) {
        for (size_t i = 0; i < sizeof(reply_pckt); i++)
        {
            get_byte = rs485.read();
            Put_CRC(reply_pckt[i]);
            if (reply_pckt[i] != get_byte) { return false; }
        }
        uint16_t received_CRC = rs485.read() << 8;
        received_CRC = received_CRC | rs485.read();
        if(CRC_receive == received_CRC){ return true; }
    }
    return false;
}

bool RS485_display_check(uint8_t disp_add) {
    /*Example of sending Values :
    [01, 55, AA] [Address(default 01)] [A1] “1234”[04]*/
    byte get_byte;
    while (rs485.available() > 0) { get_byte = rs485.read(); }
    char header_pckt[] = { 0x1, 0x55, 0xAA, disp_add, 0xA3 };
    CRC_receive = 0xffff;
    digitalWrite(RS485_PIN_DIR, RS485_WRITE);

    for (size_t i = 0; i < sizeof(header_pckt); i++)
    {
        rs485.write(header_pckt[i]);
        Put_CRC(header_pckt[i]);
    }
    rs485.flush();
    digitalWrite(RS485_PIN_DIR, RS485_READ);

    CRC_receive = 0xffff;
    char reply_pckt[5] = { 0xAA, 0x55, 'O', 'K', '!' };
    while (rs485.available() > 0) { get_byte = rs485.read(); }
    //Serial.println("Start receiving packet...");
    unsigned long startedWaiting = millis();
    while (millis() - startedWaiting <= 100 && rs485.available() < 7) {
        for (size_t i = 0; i < sizeof(reply_pckt); i++)
        {
            get_byte = rs485.read();
            Put_CRC(reply_pckt[i]);
            if (reply_pckt[i] != get_byte) { return false; }
        }
        uint16_t received_CRC = rs485.read() << 8;
        received_CRC = received_CRC | rs485.read();
        if (CRC_receive == received_CRC) { return true; }
    }
    return false;
}

void Put_CRC(uint8_t Byte) {
    CRC_receive ^= Byte;    // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
        if ((CRC_receive & 0x0001) != 0) {      // If the LSB is set
            CRC_receive >>= 1;                    // Shift right and XOR 0xA001
            CRC_receive ^= CRC_divisor; //0xA001;
        }
        else                            // Else LSB is not set
            CRC_receive >>= 1;                    // Just shift right
    }
}