#include <MCUFRIEND_kbv.h>
#include <Adafruit_GFX.h> // Core graphics library 
 
#define LCD_RESET A4
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define BUMPER 31
#define SNTN 35
#define LOSE 33

// Assign common color values name
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

//Declare global variables
char *init_text1 = "WELCOME";
// char *scoregain = "+";
char *totalscore = "TOTAL SCORE: ";
char *losemsg = "GAME OVER";
char *bumpermsg = "BUMPER! ";
char *sntnmsg = "SNTN! ";
unsigned int score_bumper = 5;
unsigned int score_saonoitoknam = 50;
unsigned int total_score = 0;
  
MCUFRIEND_kbv tft;
 
//setup
 
void setup(void) {
Serial.begin(9600);
Serial.println(F("TFT LCD test"));
 
tft.reset();
 
uint16_t identifier = tft.readID();
if (identifier == 0x9325) {
Serial.println(F("Found ILI9325 LCD driver"));
} else if (identifier == 0x9328) {
Serial.println(F("Found ILI9328 LCD driver"));
} else if (identifier == 0x4535) {
Serial.println(F("Found LGDP4535 LCD driver"));
} else if (identifier == 0x7575) {
Serial.println(F("Found HX8347G LCD driver"));
} else if (identifier == 0x9595) {
Serial.println(F("Found HX8347-I LCD driver"));
} else if (identifier == 0x4747) {
Serial.println(F("Found HX8347-D LCD driver"));
} else if (identifier == 0x8347) {
Serial.println(F("Found HX8347-A LCD driver"));
} else if (identifier == 0x9341) {
Serial.println(F("Found ILI9341 LCD driver"));
} else if (identifier == 0x7783) {
Serial.println(F("Found ST7781 LCD driver"));
} else if (identifier == 0x8230) {
Serial.println(F("Found UC8230 LCD driver"));
} else if (identifier == 0x8357) {
Serial.println(F("Found HX8357D LCD driver"));
} else if (identifier == 0x0101) {
identifier = 0x9341;
Serial.println(F("Found 0x9341 LCD driver"));
} else if (identifier == 0x7793) {
Serial.println(F("Found ST7793 LCD driver"));
} else if (identifier == 0xB509) {
Serial.println(F("Found R61509 LCD driver"));
} else if (identifier == 0x9486) {
Serial.println(F("Found ILI9486 LCD driver"));
} else if (identifier == 0x9488) {
Serial.println(F("Found ILI9488 LCD driver"));
} else {
Serial.print(F("Unknown LCD driver chip: "));
Serial.println(identifier, HEX);
Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
Serial.println(F(" #define USE_ADAFRUIT_SHIELD_PINOUT"));
Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
Serial.println(F("If using the breakout board, it should NOT be #defined!"));
Serial.println(F("Also if using the breakout, double-check that all wiring"));
Serial.println(F("matches the tutorial."));
identifier = 0x9486;
}
tft.begin(identifier);
tft.fillScreen(BLACK);
tft.setRotation(1);
unsigned long start = micros();
showWelcome();
}

//loop

void loop()
{
  if (Serial.available())
  {
      char input = Serial.read();
    if (input == 'b')
    {
      total_score += score_bumper;
      printscore(bumpermsg, score_bumper);
    }
    if (input == 'l')
    {
      lose();
    }
  }
}

void printCenteredText(const char* text, int y, int size, uint16_t color) {
  int x = (tft.width() - strlen(text) * 6 * size) / 2;
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.setTextSize(size);
  tft.print(text);
}

void drawBorder(uint16_t color) {
  tft.drawRect(0, 0, tft.width(), tft.height(), color);
  tft.drawRect(2, 2, tft.width() - 4, tft.height() - 4, color);
}

void showWelcome() {
  tft.fillScreen(WHITE);
  drawBorder(BLACK);
  printCenteredText(init_text1, 60, 9, BLACK);

  char scoreStr[20];
  sprintf(scoreStr, "SCORE: %d", total_score);
  printCenteredText(scoreStr, 200, 7, BLACK);
}

void printscore(char *type, int score){
  tft.fillScreen(BLACK);  
  drawBorder(MAGENTA);

  char gainStr[20];
  sprintf(gainStr, "%s+%d",type, score);

  char scoreStr[20];
  sprintf(scoreStr, "%d", total_score);

  printCenteredText(gainStr, 40, 6, YELLOW);
  printCenteredText("TOTAL SCORE:", 130, 6, CYAN);
  printCenteredText(scoreStr, 220, 6, CYAN);
}
void lose(void){
  tft.fillScreen(RED);
  drawBorder(WHITE);
  printCenteredText(losemsg, 40, 7, WHITE);
  char finalscr[20];
  sprintf(finalscr, "%d", total_score);
  printCenteredText("Final Score:", 150, 6, WHITE);
  printCenteredText(finalscr, 230, 7, WHITE);
  total_score = 0;
  delay(2000);
  showWelcome();
}
