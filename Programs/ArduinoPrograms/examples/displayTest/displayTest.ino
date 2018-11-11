/**************************************************************************
 * This folowing program is a test of the 128 by 32 OLED Display
 * Using the Adafruit_GFX.h , Adafruit_SSD1306.h.
 * By William Brines
 * November 10, 2018
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 14
static const unsigned char PROGMEM logo_bmp[] =
{ B00000001, B10000000,
  B00000001, B10000000,
  B00000101, B10100000,
  B00011101, B10110000,
  B00110001, B10011000,
  B00110001, B10011000,
  B01100001, B10001100,
  B01100001, B10001100,
  B01100000, B00001100,
  B00110000, B00001100,
  B00110000, B00011000,
  B00011000, B00011000,
  B00001110, B01110000,
  B00000111, B11100000 };

void setup() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

}

void loop(){
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  //display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
/*
  // Draw a single pixel in white
  display.drawPixel(10, 10, WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  display.clearDisplay();
  display.drawLine(0, 0, display.width(), display.height()-1, WHITE); // Draws a line
  display.display();
  delay(2000);

  display.clearDisplay();
  display.drawRect( 1 , 1, display.width()-2, display.height()-2, WHITE);// Draw rectangle (outlines)
  display.display(); 
  delay(2000);

  display.clearDisplay();
  display.fillRect( 1, 1, display.width()-2, display.height()-2, WHITE);// Draw rectangle (Filled)
  display.display(); 
  delay(2000);

  display.clearDisplay();
  display.drawCircle(display.width()/2, display.height()/2, 2, WHITE);// Draw circle (outlines)
  display.display();
  delay(2000);

  display.clearDisplay();
  display.fillCircle(display.width() / 2, display.height() / 2, 10, WHITE);// Draw circle (Filled)
  display.display();
  delay(2000);

  display.clearDisplay();
  display.drawRoundRect( 10 , 10, display.width()-20, display.height()-20, // Draw a rounded Rect
  display.height()/4, WHITE);
  display.display();
  delay(2000);
  
  display.clearDisplay();
  display.fillRoundRect( 10 , 10, display.width()-20, display.height()-20, // Draw a filled rounded rect
  display.height()/4, WHITE);
  display.display();
  delay(2000);

  display.clearDisplay();
  display.drawTriangle(                                                   // Draw a triangle
      display.width()/2 + 10, display.height()/2 +10,
      display.width()/2 - 10, display.height()/2 -10,
      display.width()/2 + 10, display.height()/2 -10, WHITE);
  display.display();
  delay(2000);

  display.clearDisplay();
  display.fillTriangle(                                                    // Draw a filled triangle
      display.width()/2 + 10, display.height()/2 +10,
      display.width()/2 - 10, display.height()/2 -10,
      display.width()/2 + 10, display.height()/2 -10, WHITE);
  display.display();
  delay(2000);

  // Draw characters of the default font *****************************
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  
  display.write('L');
  display.write('o');
  display.write('v');
  display.write('e');
  
  display.display();
  delay(2000);
  
  //testdrawstyles();    // Draw 'stylized' characters*******************
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println("Hello, world!");

  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.print("0x"); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);

  //testscrolltext();    // Draw scrolling text****************************
    display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println("scroll");
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
*/
  // Draw a small bitmap image
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(10000);
  /*
  // Invert and restore display, pausing in-between
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
  delay(1000);
  */
}

