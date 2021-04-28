//Filename: Lab8.c
//Written By: Anoushka and Tanner
//Config: AVR ATMega2560 @16MHz
//Description: Servo Control

//Includes
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>


// Helpful LCD control defines//
#define LCD_Reset              0b00110000 //reset lcd 4bit
#define LCD_4bit_enable        0b00100000 //4bit can set line or font till after this is set
#define LCD_4bit_mode          0b00101000 //2 line display 4x8 font
#define LCD_4bit_displayOFF    0b00001000 //set display off
#define LCD_4bit_displayON     0b00001100 //set diplay on - no blink
#define LCD_4bit_displayON_B1  0b00001101 // display on w/ blink
#define LCD_4bit_displayCLEAR  0b00000001 // set all chars to "space"
#define LCD_4bit_entryMODE     0b00000110 // set cursor to left to right
#define LCD_4bit_cursorSET     0b10000000 // set cursor position

//For 2 line mode //
#define LineOneStart 0x00
#define LineTwoStart 0x40 //must set DDRAM adress in LCD for line 2//

//Pin definition for PORTB control lines //
#define LCD_EnablePin 1
#define LCD_RegisterSelectPin 0

//Our defines
#define USART_BAUD 57600
#define BAUD_PRESCALE F_CPU/(USART_BAUD*16) - 1

// Prototypes //
void LCD_init(void);
void LCD_E_RS_init(void);
void LCD_write_4bits(uint8_t);
void LCD_ENABLEPulse(void);
void LCD_write_instruction(uint8_t);
void LCD_write_char(char);
void InitUSART();
void printCharacterFromUSART();
void createMenu();
void printToUSART(char * string);
void menuDetermination(char * input);
void printCharactertoUSART(char character);
void interruptInput();
void servoInit();
void dcInit();
int calculateSpeed(int duty_cycle);
int OC1A_calculater(int degreesInput);



//global variable for the interrupt
volatile char receiveString[80] = "";
volatile int j = 0;
volatile int k = 0;

volatile bool takingInput = false;
volatile bool doneTakingInput = false;
volatile bool isSpeed = false;
volatile bool isAngle = false; 
volatile unsigned int speedInput = 0;
volatile signed int servoAngle = 0;
volatile char inputString[5] = "";
volatile int p = 0;

//Interrupt Service routine for the USART 
ISR(USART0_RX_vect)
{
  char input = UDR0;
  inputString[p] = input;
 
  if(input == '\n')
  {
    inputString[p] = '\0';
    doneTakingInput = true;
  }
  p++;
}

void interruptInput()
{
  p = 0;
  int i = 0;
  int lenInput = strlen(&inputString[0]);

  for (i = 0; i<lenInput; i++)
  {
    receiveString[(j%80)] = inputString[i];
    j++;
  }

  if(isSpeed)
  {
    speedInput = atoi(inputString);
    printToUSART(inputString);
    OCR5A = calculateSpeed(speedInput);
    isSpeed = false;
    
  }
  else if(isAngle)
  {
    servoAngle = atoi(inputString);
    OCR1A = OC1A_calculater(servoAngle);
    printToUSART(inputString);
    isAngle = false;
  }
  else
  {
    printToUSART(inputString);
  }
  testString();
  menuDetermination(inputString);
}

void testString(void)
{
  char speedBuffer[10];
  itoa(speedInput, &speedBuffer[0], 10);
  char angleBuffer[10];
  itoa(servoAngle, &angleBuffer[0], 10);
  char line1[16]= "dc motor: ";  
  char line2[16]= "servo angle: ";
  strcat(&line1[0], &speedBuffer[0]);
  strcat(&line2[0], &angleBuffer[0]);
  LCD_write_instruction(LCD_4bit_cursorSET | LineOneStart);
  _delay_ms(80); //delay must be > 1.53ms us //
  LCD_write_instruction(LCD_4bit_displayCLEAR);
  _delay_ms(80); //delay must be > 1.53ms us //
  int u = 0;
  LCD_write_instruction(LCD_4bit_cursorSET | LineOneStart);
        _delay_us(80);
  for( u; u<20; u++)
  {
     LCD_write_char(line1[u]);
  }
  u = 0;
  LCD_write_instruction(LCD_4bit_cursorSET | LineTwoStart);
        _delay_us(80);
  for( u; u<20; u++)
  {
     LCD_write_char(line2[u]);
  } 
}

// Important notes in sequence from page 26 in the KS0066U datasheet - initialize the LCD in 4bit two line mode //
// LCD is initiall set to 8bit mode - we need to reset the LCD controller to 4-bit mode we can set anything else //
void LCD_init(void)
{
  //wait for power up - more than 30ms for cdd to rise to 4.5V//
  _delay_ms(100);
  // note that we need to reset controller to enable 4-bit mode)//
  LCD_E_RS_init(); //Set the E and RS pins active low for each LCD reset //
  // reset and await activation //
  LCD_write_4bits(LCD_Reset);
  _delay_us(80); //delay must be >39us //
  ///////system reset complet set up LCD modes /////
  //at the point we are in 4bit mode(must sent low and high nibble seperate)
  // and can now set line numbers ad font size
  // NOTICE: we use two calls of LCD_write_4bits () functon for 8bigt mode
  //once in 4bit mode the set of instructon found in table 7 of data sheet
  LCD_write_instruction(LCD_4bit_mode);
  _delay_us(80); //dela must be > 39us//

  //from page 26 and table 7 indata sheet
  //we need to display = off, displa = clear, and entry mode = set //
  LCD_write_instruction(LCD_4bit_displayOFF);
  _delay_us(80); //delay must be > 39 us //

  LCD_write_instruction(LCD_4bit_displayCLEAR);
  _delay_ms(80); //delay must be > 1.53ms us //
 
  LCD_write_instruction(LCD_4bit_entryMODE);
  _delay_us(80); //delay must be > 39 us //

  // LCD should now be initialized to operate in 4-bit mode, 2 lines, 5x8 fontsize//
  // need to turn display on for use //
  LCD_write_instruction(LCD_4bit_displayON);
  _delay_us(80); //delay must be > 39 us //
}

void LCD_E_RS_init(void)
{
  //set up the E and RS lines to activate low for reset function //
  PORTB &= ~(1<<LCD_EnablePin);
  PORTB &= ~(1<<LCD_RegisterSelectPin);
}

// send a byte of data to the lcd module //
void LCD_write_4bits(uint8_t Data)
{
  //we are only interested in sending the data to the upper 4 bits of PORTA //
  PORTA &= 0b00001111; //upper nibble cleared //
  PORTA |= Data; //data wrote to datta lines on PORTA
  // data is now in upper nyble of PORTA - pulse to send //
  LCD_EnablePulse(); //pulse to enable read / write
}

//wRITE AN INSTRUCTION IN 4BIT MODE-- need to send upper and lower nyble //
void LCD_write_instruction(uint8_t Instruction)
{
  //ensure RS us low //
  //PORTB &= ~(1 <<LCD_RegisterSelectPin);
  LCD_E_RS_init(); //sets E and R active low for each LCD reset //
  LCD_write_4bits(Instruction); //write hi nyble
  LCD_write_4bits(Instruction << 4); //write low nyblle
}

// pulse enable pin on LCD controller to read/write data lines - should be atleast 230ns pulse width //
void LCD_EnablePulse(void)
{
  //set enable low hi low
  //PORTB &= ~(1<<LCD_EnablePin); //set enable low //
  //_delay_us(1) // wait to ensure pinlow //
  PORTB |= (1 <<LCD_EnablePin); //set enable hi //
  _delay_us(1); //wait to esure pin is hi //
  PORTB &= ~(1<<LCD_EnablePin); //set enable low //
  _delay_us(1); //wait to ensure the pin is low
}

//write character to display //
void LCD_write_char(char Data)
{
  //set up the E and the RS lines for data writing //
  PORTB |= (1<< LCD_RegisterSelectPin); //ensure RS pin is High //
  PORTB &= ~(1 <<LCD_EnablePin);//ensure enable pin is low //
  LCD_write_4bits(Data); //write to upper nybble //
  LCD_write_4bits(Data<<4); //write lower nybble //
  _delay_us(500); //need to wait > 43us //
}

//takes a char* and each character in the string
//it outputs to the LCD.
void fixedStringOutput(char * string)
{
  static int wrap = 1;
  //int s_size = strlen(string);
  int s_size = 4;
  int i = 0;
  
  while(i < s_size)
  {
    if(wrap == 17)
    {
       LCD_write_instruction(LCD_4bit_cursorSET | LineTwoStart);
       LCD_write_char(' ');
    }
    if (wrap == 33)
    {
       LCD_write_instruction(LCD_4bit_cursorSET | LineOneStart);
       LCD_write_char(' ');
       wrap = 0;
    }
    LCD_write_char(string[i]);
    wrap++;
    i++;
  }
}


//initialize USART0
void InitUSART()
{
  //enable Rx and Tx
  UCSR0B |= 0x98;
  //8-bit fram-async , 1-stop, etc.
  UCSR0C |= 0x06;
  //set Baud
  UBRR0L = BAUD_PRESCALE;
  UBRR0H=(BAUD_PRESCALE >> 8);  
  SREG |= 0x80;
  UCSR0A &= 0x3F;
}

void printCharacterFromUSART()
{
  //While the counter here is less than the receiveString counter in the ISR
  //sorry for not using more meaningful variable names.
  static int wrap = 1;
 
  while(k < j)    
  {
     if(wrap == 17)
    {
       LCD_write_instruction(LCD_4bit_cursorSET | LineTwoStart);
       LCD_write_char(' ');
    }
    if (wrap == 33)
    {
       LCD_write_instruction(LCD_4bit_cursorSET | LineOneStart);
       LCD_write_char(' ');
       wrap = 0;
    }
    wrap++;
    LCD_write_char(receiveString[(k%80)]);
    k++;
  }
}

void createMenu()
{
  char menuStatement[] = "\nPlease select the motor you want to control: \n1) DC Speed \n2) Servo Angle\n\nInput\t";
  printToUSART(menuStatement);
}

void printCharactertoUSART(char character)
{
  while(! (UCSR0A & (1 << UDRE0)));
  UDR0 = character; 
}

void printToUSART(char * menuString)
{
  int len = strlen(&menuString[0]);
  int i = 0;
  for (i = 0; i<len; i++)
  {
    printCharactertoUSART(menuString[i]);
  }
}

void menuDetermination(char * input)
{
  int lenInput = strlen(&input[0]);

  if(lenInput > 1)
  {
    return;
  }

  switch(input[0])
  {
    case '1':
        printToUSART("\n\nLooks like you'd like to change the speed of the DC motor, what speed would you like (0 - 100% duty)?");
        isSpeed = true;
        isAngle = false;
        break;
    case '2':
        printToUSART("\n\nLooks like you'd like to change the servo angle, what angle would you like (-120 - 120)?");
        isSpeed = false;
        isAngle = true;
        break;
    default:
        break;
  }
}

void servoInit()
{
  DDRB |= (1<<PB5);  /* Make OC1A pin as output */
  TCNT1 = 0;          /* Set timer1 count zero */
  ICR1 = 625;           /* Set TOP count for timer1 in ICR1 register */

  OCR1A = 500;

   /* Set Fast PWM, TOP in ICR1,*/
  TCCR1A = (1<<WGM11)|(1<<COM1A1) |(1<<COM1A0);    //Pg 145
  TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS12); //prescaler of 256 p.157
}

int OC1A_calculater(int degreesInput)
{
  return (degreesInput*(605-440)/180) + 440;
}

void dcInit()
{
  DDRL |= (1<<PL3);  /* Make OC5A pin as output */
  TCNT5 = 0;          /* Set timer5 count zero */
  ICR5 = 1000;           /* Set TOP count for timer5 in ICR5 register */

   /* Set Fast PWM, TOP in ICR5,*/
  TCCR5A = (1<<WGM51)|(1<<COM5A1) |(1<<COM5A0);    //Pg 145
  TCCR5B = (1<<WGM52)|(1<<WGM53)|(1<<CS50); //prescaler of 256 p.157

  OCR5A = 1000;
}

int calculateSpeed(int duty_cycle)
{
  return (100 - duty_cycle)*10;
}


int main(void)
{
  DDRB = 0x23;
  DDRA = 0xF0;
 
  // Initialize LCD for 4bit mode, two lines, and 5 x 8 dots //
  // Inits found on Page 26 of datasheet and Table 7 for function set instructions //
  LCD_init();
  InitUSART();
  createMenu();
  servoInit();
  dcInit();
  int i = 0;
  testString();
  while(1)
  {
    if(doneTakingInput)
    {
      doneTakingInput = false;
      interruptInput();
    }
    
  }
  return 1;
}
