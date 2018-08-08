/*!*********************************************************************************************************************
@file user_app1.c                                                                
@brief nRF interface application.

Provides a communication link between the SAM3U2 and nRF51422 processors.
The communication uses a SPI slave with flow control on the SAM3U2.
A simple protocol will be used for the messages:

[START_BYTE, LENGTH, COMMAND, DATA0, …, DATAn]
where,
START_BYTE = 0x5A
LENGTH = 1 + the number of data bytes

Messages will always be complete when transmitted or received.
The slave will initialize with SRDY asserted so it is ready for a message.
The master shall not assert CS if SRDY is not asserted.
The master will assert CS when it is clocking a message.
The slave shall deassert SRDY at the start of a message transfer.
The slave shall not assert MRDY if a message is being clocked.


Reserved Commands nRF51422 to SAM3U2 (0x00 - 0x1F):  
CMD  ARG_BYTE(s)        FUNCTION
--------------------------------------------------------
0x01 LED,STATE          LED number, ON(1) OFF (0)
0x02 LCD Message1       Null-terminated string to forward to LCD line 1 (line is erased first)
0x03 LCD Message2       Null-terminated string to forward to LCD line 2 (line is erased first)
0x04 Debug Message      Null-terminated string to forward to DebugPrintf();
0x05 BUZZER,FREQ        Activate BUZZER (1 or 2) at FREQ (0 for off)


Reserved Commands SAM3U2 to nRF51422 (0x20 - 0x3F):  
CMD  ARG_BYTE(s)           FUNCTION
--------------------------------------------------------
0x21 BUTTON_PRESSED        BUTTON number (works only with WasButtonPressed();

------------------------------------------------------------------------------------------------------------------------
GLOBALS
- NONE

CONSTANTS
- NONE

TYPES
- NONE

PUBLIC FUNCTIONS
- NONE

PROTECTED FUNCTIONS
- void UserApp1Initialize(void)
- void UserApp1RunActiveState(void)


**********************************************************************************************************************/

#include "configuration.h"

/***********************************************************************************************************************
Global variable definitions with scope across entire project.
All Global variable names shall start with "G_<type>UserApp1"
***********************************************************************************************************************/
/* New variables */
volatile u32 G_u32UserApp1Flags;                          /*!< @brief Global state flags */


/*--------------------------------------------------------------------------------------------------------------------*/
/* Existing variables (defined in other files -- should all contain the "extern" keyword) */
extern volatile u32 G_u32SystemTime1ms;                   /*!< @brief From main.c */
extern volatile u32 G_u32SystemTime1s;                    /*!< @brief From main.c */
extern volatile u32 G_u32SystemFlags;                     /*!< @brief From main.c */
extern volatile u32 G_u32ApplicationFlags;                /*!< @brief From main.c */


/***********************************************************************************************************************
Global variable definitions with scope limited to this local application.
Variable names shall start with "UserApp1_<type>" and be declared as static.
***********************************************************************************************************************/
static fnCode_type UserApp1_pfStateMachine;               /*!< @brief The state machine function pointer */
//static u32 UserApp1_u32Timeout;                           /*!< @brief Timeout counter used across states */

static u8 u8ShowData[16][12]={ {0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}};
static u8 u8FontData1[32] = {0};
static u8 u8SequenceOfWord = 0;
u8 u8HanziBlank[32] = {0};
static u8 u8NumberOfWord = 9;
static u16 u16Hanzi[20] = {'ÄÏ','¾©','¹¤','³Ì','Ñ§','Ôº','»¶','Ó­','Äú'};

/**********************************************************************************************************************
Function Definitions
**********************************************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------------*/
/*! @publicsection */                                                                                            
/*--------------------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------------------------------*/
/*! @protectedsection */                                                                                            
/*--------------------------------------------------------------------------------------------------------------------*/

/*!--------------------------------------------------------------------------------------------------------------------
@fn void UserApp1Initialize(void)

@brief
Initializes the State Machine and its variables.

Should only be called once in main init section.

Requires:
- NONE

Promises:
- NONE

*/
void UserApp1Initialize(void)
{
  AT91C_BASE_PIOA->PIO_PER  |= (PA_04_HSMCI_MCCDA|PA_05_HSMCI_MCDA0|PA_06_HSMCI_MCDA1|PA_07_HSMCI_MCDA2|PA_08_SD_CS_MCDA3|PA_03_HSMCI_MCCK);
  AT91C_BASE_PIOA->PIO_PER  |= (PA_15_BLADE_SCK|PA_14_BLADE_MOSI|PA_12_BLADE_UPOMI|PA_11_BLADE_UPIMO);
  AT91C_BASE_PIOA->PIO_PDR  &= ~(PA_04_HSMCI_MCCDA|PA_05_HSMCI_MCDA0|PA_06_HSMCI_MCDA1|PA_07_HSMCI_MCDA2|PA_08_SD_CS_MCDA3|PA_03_HSMCI_MCCK);
  AT91C_BASE_PIOA->PIO_PDR  &= ~(PA_15_BLADE_SCK|PA_14_BLADE_MOSI|PA_12_BLADE_UPOMI|PA_11_BLADE_UPIMO);
  
  AT91C_BASE_PIOA->PIO_OER  |= (PA_04_HSMCI_MCCDA|PA_05_HSMCI_MCDA0|PA_06_HSMCI_MCDA1|PA_07_HSMCI_MCDA2|PA_08_SD_CS_MCDA3|PA_03_HSMCI_MCCK);
  AT91C_BASE_PIOA->PIO_OER  |= (PA_15_BLADE_SCK|PA_14_BLADE_MOSI|PA_12_BLADE_UPOMI|PA_11_BLADE_UPIMO);
  AT91C_BASE_PIOA->PIO_ODR  &= ~(PA_04_HSMCI_MCCDA|PA_05_HSMCI_MCDA0|PA_06_HSMCI_MCDA1|PA_07_HSMCI_MCDA2|PA_08_SD_CS_MCDA3|PA_03_HSMCI_MCCK);
  AT91C_BASE_PIOA->PIO_ODR  &= ~(PA_15_BLADE_SCK|PA_14_BLADE_MOSI|PA_12_BLADE_UPOMI|PA_11_BLADE_UPIMO);
  
  AT91C_BASE_PIOB->PIO_PER  |= (PB_05_TP56|PB_06_TP58|PB_07_TP60|PB_08_TP62);
  AT91C_BASE_PIOB->PIO_PDR  &= ~(PB_05_TP56|PB_06_TP58|PB_07_TP60|PB_08_TP62);
  
  AT91C_BASE_PIOB->PIO_OER  |= (PB_05_TP56|PB_07_TP60|PB_08_TP62);
  AT91C_BASE_PIOB->PIO_OER  &= ~PB_06_TP58;
  
  AT91C_BASE_PIOB->PIO_ODR  &= ~(PB_05_TP56|PB_07_TP60|PB_08_TP62);
  AT91C_BASE_PIOB->PIO_ODR  |= PB_06_TP58;
  
 
  
  /* If good initialization, set state to Idle */
  if( 1 )
  {
    UserApp1_pfStateMachine = UserApp1SM_Idle;
  }
  else
  {
    /* The task isn't properly initialized, so shut it down and don't run */
    UserApp1_pfStateMachine = UserApp1SM_Error;
  }

} /* end UserApp1Initialize() */

  
/*!----------------------------------------------------------------------------------------------------------------------
@fn void UserApp1RunActiveState(void)

@brief Selects and runs one iteration of the current state in the state machine.

All state machines have a TOTAL of 1ms to execute, so on average n state machines
may take 1ms / n to execute.

Requires:
- State machine function pointer points at current state

Promises:
- Calls the function to pointed by the state machine function pointer

*/
void UserApp1RunActiveState(void)
{
  UserApp1_pfStateMachine();

} /* end UserApp1RunActiveState */


/*------------------------------------------------------------------------------------------------------------------*/
/*! @privatesection */                                                                                            
/*--------------------------------------------------------------------------------------------------------------------*/


/**********************************************************************************************************************
State Machine Function Definitions
**********************************************************************************************************************/
/*-------------------------------------------------------------------------------------------------------------------*/
/* What does this state do? */
static void UserApp1SM_Idle(void)
{
  static u8 NumberOfBlank = 0;
  static u8 u8TimeCounter = 0;
  static u8 u8MoveTimes = 0;
  static u8 u8nrfCheck = 0;
  
  u8nrfCheck = nrfNewMessageCheck();
  
  //Èç¹ûµÃµ½ÐÂÊý¾Ý£¬½øÐÐ´¦Àí
  if (u8nrfCheck != NRF_CMD_EMPTY)
  {
    AT91C_BASE_PIOA->PIO_SODR |=PA_04_HSMCI_MCCDA;//INH=1;ËùÓÐÏÔÊ¾¹Ø±Õ 
    //½«ÆÁÄ»Çå¿Õ
    WriteToShowData(1,u8HanziBlank);
    WriteToShowData(2,u8HanziBlank);
    WriteToShowData(3,u8HanziBlank);
    WriteToShowData(4,u8HanziBlank);
    WriteToShowData(5,u8HanziBlank);
    WriteToShowData(6,u8HanziBlank);
    
    WriteToHanzi();
    
  }
  
  u8TimeCounter++;
   
  //30ms shift to left one bit
  if (u8TimeCounter == 30)
  {
    u8TimeCounter = 0;
    LeftOneBit();
    u8MoveTimes++;
  }
  
  //Every time move 16 times (a word), write a word to the prepared area
  if (u8MoveTimes == 16)
  {
    u8MoveTimes = 0;
    
    if (u8SequenceOfWord < (u8NumberOfWord+1) )
    {
      GetFontData(u16Hanzi[u8SequenceOfWord]);
      u8SequenceOfWord++;
      WriteToShowData(1,u8FontData1);
    }
    
    //ËùÓÐ×ÖÏÔÊ¾Íêºó£¬ºóÃæ½Ó¿Õ°×
    if (u8SequenceOfWord == (u8NumberOfWord+1) )
    {
      WriteToShowData(1,u8HanziBlank);
      NumberOfBlank++;
    }
       
    //When there are 5 blanks(The screen is empty), restart
    if (NumberOfBlank == 5)
    {
      NumberOfBlank = 0;
      u8SequenceOfWord = 0;
    }
  }
   
  Show(); //Show the data in the u8ShowData[]
  

 
  
} /* end UserApp1SM_Idle() */
   

void  WriteToHanzi(void)
{
  static u8 au8DataFromNrf[128] = {0};
  u8SequenceOfWord = 0;
  u8 u8HanziArraySize = sizeof(u16Hanzi);
  u8 u8Data = 0;
  u16 u16HanziData = 0;
  u8 u8Times = 0;
  u8 u8ArrayBit = 0;
  
  nrfGetAppMessage(au8DataFromNrf);//»ñµÃÊý¾Ý
  u8NumberOfWord = au8DataFromNrf[1];
  
  //Çå³ýu16HanziÊý×éµÄÄÚÈÝ
  for(u8 i=0;i<u8HanziArraySize;i++)
  {
    u16Hanzi[i] = 0;
  }
  
  for (u8 j=2;j<u8NumberOfWord+2;j++)
  {    
    u8Data = au8DataFromNrf[j];
    u8Times++;
    
    if(u8Times == 1)
    {
      u16HanziData = u8Data;
      u16HanziData = u16HanziData<<8;
    }
    
    if (u8Times == 2)
    {
      u8Times = 0;
      u16HanziData |= u8Data;
      u16Hanzi[u8ArrayBit] = u16HanziData;
      u8ArrayBit++;
    }  
  } 
  
}

u32 GetAddress(u16 Hanzi)
{
  u8 u8MSB = Hanzi>>8; //GBCode high 8bits
  u8 u8LSB = Hanzi;//GBCode low 8bits
  u32 u32Address = 0;//the address of Hanzi,3 bytes
  u32 u32BaseAdd = 0;// base address
  
  
  
  if(u8MSB>=0xA4 && u8MSB<=0xA8 && u8LSB>=0xA1)
  {
    u32Address = u32BaseAdd;
  }
  else if(u8MSB>=0xA1 && u8MSB<=0xA9 && u8LSB>=0xA1)
  {
    u32Address = ( (u8MSB-0xA1)*94 + (u8LSB-0xA1) )*32 + u32BaseAdd;
  }
  else if(u8MSB>=0xB0 && u8MSB<=0xF7 && u8LSB>=0xA1)
  {
    u32Address = ( (u8MSB-0xB0)*94 + (u8LSB-0xA1)+846)*32 + u32BaseAdd;
  }
  
  return (u32Address);
}


void GetFontData(u16 Hanzi)
{
  u32 u32Address = GetAddress(Hanzi);
  u8 u8ArrayBit = 0;
  u8 u8FontData = 0;
  u8 u8Bit = 0;
  u8 u8ReadData = 0x03; //¶ÁÊý¾ÝÖ¸Áî
  u8 u8Address1 = u32Address>>16;//the first byte
  u8 u8Address2 = u32Address>>8; //the second byte
  u8 u8Address3 = u32Address;//the third byte
  
  AT91C_BASE_PIOB->PIO_CODR |= PB_05_TP56;//CS = 0;
  
  //send read data byte
  for (u8 i=0;i<8;i++)
  {
    AT91C_BASE_PIOB->PIO_CODR |= PB_07_TP60; //SCLK = 0;
    
    if ( (u8ReadData&0x80) == 0x80)
    {
      AT91C_BASE_PIOB->PIO_SODR |= PB_08_TP62; //SI = 1;
    }
    else
    {
      AT91C_BASE_PIOB->PIO_CODR |= PB_08_TP62; //SI = 0;
    }
    
    Delay(5);
    AT91C_BASE_PIOB->PIO_SODR |= PB_07_TP60; //SCLK = 1;
    Delay(5);
    u8ReadData = u8ReadData<<1;
    Delay(5);
  }
  
  //send the first address byte
  for (u8 j=0;j<8;j++)
  {
    AT91C_BASE_PIOB->PIO_CODR |= PB_07_TP60; //SCLK = 0;
    
    if ( (u8Address1&0x80) == 0x80)
    {
      AT91C_BASE_PIOB->PIO_SODR |= PB_08_TP62; //SI = 1;
    }
    else
    {
      AT91C_BASE_PIOB->PIO_CODR |= PB_08_TP62; //SI = 0;
    }
    
    Delay(5);
    AT91C_BASE_PIOB->PIO_SODR |= PB_07_TP60; //SCLK = 1;
    Delay(5);
    u8Address1 = u8Address1<<1;
    Delay(5);
  }
  
  //send the second address byte
  for (u8 k=0;k<8;k++)
  {
    AT91C_BASE_PIOB->PIO_CODR |= PB_07_TP60; //SCLK = 0;
    
    if ( (u8Address2&0x80) == 0x80)
    {
      AT91C_BASE_PIOB->PIO_SODR |= PB_08_TP62; //SI = 1;
    }
    else
    {
      AT91C_BASE_PIOB->PIO_CODR |= PB_08_TP62; //SI = 0;
    }
    
    Delay(5);
    AT91C_BASE_PIOB->PIO_SODR |= PB_07_TP60; //SCLK = 1;
    Delay(5);
    u8Address2 = u8Address2<<1;
    Delay(5);
  }
  
  //send the third address byte
  for (u8 l=0;l<8;l++)
  {
    AT91C_BASE_PIOB->PIO_CODR |= PB_07_TP60; //SCLK = 0;
    
    if ( (u8Address3&0x80) == 0x80)
    {
      AT91C_BASE_PIOB->PIO_SODR |= PB_08_TP62; //SI = 1;
    }
    else
    {
      AT91C_BASE_PIOB->PIO_CODR |= PB_08_TP62; //SI = 0;
    }
    
    Delay(5);
    AT91C_BASE_PIOB->PIO_SODR |= PB_07_TP60; //SCLK = 1;
    Delay(5);
    u8Address3 = u8Address3<<1;
    Delay(5);
  }
   
  //´ÓSOÖÐ¶Á³ö×ÖÄ£Êý¾Ý£¬¹²32*8 Î»
  for (u16 p=0;p<256;p++)
  {
    AT91C_BASE_PIOB->PIO_SODR |= PB_07_TP60; //SCLK = 1;
    
    //SOÎª¸ßµçÆ½
    if ( (AT91C_BASE_PIOB->PIO_PDSR & PB_06_TP58) == PB_06_TP58)
    {
      u8FontData |= 0x01;
      u8FontData = u8FontData<<1; 
      u8Bit++;
    }
    else//SOÎªµÍµçÆ½
    {
      u8FontData = u8FontData<<1;
      u8Bit++;
    }
    
    AT91C_BASE_PIOB->PIO_CODR |= PB_07_TP60; //SCLK = 0;
    
    //×óÒÆ7´Îºó£¬²¹ÉÏ×îµÍÎ» 
    if (u8Bit == 8)
    {
      //SOÎª¸ßµçÆ½£¬×îµÍÎ»²¹1
      if ( (AT91C_BASE_PIOB->PIO_PDSR & PB_06_TP58) == PB_06_TP58)
      {
        u8FontData |= 0x01;
      }
      else//SOÎªµçÆ½£¬×îµÍÎ»²¹0
      {
        u8FontData |= 0x00;
      }
      
      u8Bit = 0;
      u8FontData1[u8ArrayBit] = u8FontData;//Ã¿8Î»½«ÊýÄ£Êý¾Ý´æÈëÊý×éÖÐ
      u8FontData = 0;
      u8ArrayBit++;
    }   
  } 
  
  AT91C_BASE_PIOB->PIO_SODR |= PB_05_TP56;//CS = 1;
}


void LeftOneBit(void)
{
  static bool bAddOneNextBit = FALSE;
  static bool bAddOne = FALSE;
  for (u8 i=0;i<16;i++)
  {
    for (u8 j=0;j<12;j++)
    { 
      /*if the highest bit is 1,next byte shift to left and add one on the lowest bit*/
      if ( (u8ShowData[i][j]&0x80) == 0x80)
      {
        bAddOneNextBit = TRUE;
      }
      
      /*shift to left and add one on the lowest bit*/
      if (bAddOne)
      {
        bAddOne = FALSE;
        u8ShowData[i][j] = u8ShowData[i][j]<<1;
        u8ShowData[i][j] = 0x01|u8ShowData[i][j];
      }
      else/*shift to left*/
      {
        u8ShowData[i][j] = u8ShowData[i][j]<<1;
      }
      
      if (bAddOneNextBit)
      {
        bAddOneNextBit = FALSE;
        bAddOne = TRUE;
      }          
    }
  }
}


void Show(void)
{
  for (u8 i=0;i<16;i++)
  {
    MBI_data(i); //Load row data
    AT91C_BASE_PIOA->PIO_SODR |= PA_11_BLADE_UPIMO;// /OE=1,The output driver is disabled
    Delay(5);
    AT91C_BASE_PIOA->PIO_SODR |= PA_12_BLADE_UPOMI;// LE=1,data is transmitted to the output latch
    Delay(5);
    AT91C_BASE_PIOA->PIO_CODR |= PA_12_BLADE_UPOMI;// LE=0,data locked
    Delay(5);
    LineDisplay(i);    
    AT91C_BASE_PIOA->PIO_CODR |= PA_11_BLADE_UPIMO;// /OE=0,The output driver is enabled
    Delay(5);
  }
}


void WriteToShowData(u8 u8WordLocation,u8 *u8Data)
{
  for (u8 i=0;i<16;i++)
  {
    u8ShowData[i][(2*u8WordLocation)-1] = *u8Data;
    u8Data++;
    u8ShowData[i][(2*u8WordLocation)-2] = *u8Data;
    u8Data++;
  }     
}


void Delay(u8 Delaytime)
{
  for (u8 i=0;i<Delaytime;i++)
  {
    
  }
}


void MBI_data(u8 Line)
{
  u8 u8Data = 0;
  
  for (u8 i=2;i<12;i++)
  {
    u8Data = u8ShowData[Line][i];
    
    for (u8 j=0;j<8;j++)
    {
      AT91C_BASE_PIOA->PIO_CODR |= PA_15_BLADE_SCK;// M_CLK=0;
      
      if ( (0x01&u8Data) == 1)
      {
        AT91C_BASE_PIOA->PIO_SODR |= PA_14_BLADE_MOSI; //M_SDI=1,Light up the LED
      }
      else
      {
        AT91C_BASE_PIOA->PIO_CODR |= PA_14_BLADE_MOSI; //M_SDI=0,The LED is not bright
      }
      
      Delay(5);
      AT91C_BASE_PIOA->PIO_SODR |= PA_15_BLADE_SCK; // M_CLK=1 
      Delay(5);
      u8Data=u8Data>>1;  //start from low bit,and move to M_SDI bit by bit
      Delay(5);
    }
  }
}


void  LineDisplay(u8 Line_)
{
  AT91C_BASE_PIOA->PIO_SODR |=PA_03_HSMCI_MCCK;//CD_STB=1; 
  AT91C_BASE_PIOA->PIO_CODR |=PA_04_HSMCI_MCCDA;//INH=0; 
 
  switch (Line_)
  {
case 0:
      /*A=0,B=0,C=0,D=0,Line0*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3; 
      break;
       
case 1:
      /*A=1,B=0,C=0,D=0,Line1*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3;
      break;
      
case 2:
      /*A=0,B=1,C=0,D=0,Line2*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3;
      break;
      
case 3:
      /*A=1,B=1,C=0,D=0,Line3*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3;  
      break;
      
case 4:
      /*A=0,B=0,C=1,D=0,Line4*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3; 
      break;
      
case 5:
      /*A=1,B=0,C=1,D=0,Line5*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3;   
      break;
      
case 6:
      /*A=0,B=1,C=1,D=0,Line6*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3; 
      break;

case 7:
      /*A=1,B=1,C=1,D=0,Line7*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_CODR |=PA_08_SD_CS_MCDA3; 
      break;
      
case 8:
      /*A=0,B=0,C=0,D=1,Line8*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3; 
      break;

case 9:
      /*A=1,B=0,C=0,D=1,Line9*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3;  
      break;
      
case 10:
      /*A=0,B=1,C=0,D=1,Line10*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3;
      break;
      
case 11:
      /*A=1,B=1,C=0,D=1,Line11*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_CODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3;
      break;
      
case 12:
      /*A=0,B=0,C=1,D=1,Line12*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3;
      break;
      
case 13:
      /*A=1,B=0,C=1,D=1,Line13*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_CODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3;
      break;
      
case 14:
      /*A=0,B=1,C=1,D=1,Line14*/
      AT91C_BASE_PIOA->PIO_CODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3;
      break;
      
case 15:
      /*A=1,B=1,C=1,D=1,Line15*/
      AT91C_BASE_PIOA->PIO_SODR |=PA_05_HSMCI_MCDA0;
      AT91C_BASE_PIOA->PIO_SODR |=PA_06_HSMCI_MCDA1;
      AT91C_BASE_PIOA->PIO_SODR |=PA_07_HSMCI_MCDA2;
      AT91C_BASE_PIOA->PIO_SODR |=PA_08_SD_CS_MCDA3;
      break;
  }
   
}



/*-------------------------------------------------------------------------------------------------------------------*/
/* Handle an error */
static void UserApp1SM_Error(void)          
{
  
} /* end UserApp1SM_Error() */



/*--------------------------------------------------------------------------------------------------------------------*/
/* End of File                                                                                                        */
/*--------------------------------------------------------------------------------------------------------------------*/
