#include "..\inport\stdint.h"
#include "..\inport\stdio.h"
#include "..\inport\lpc177x_8x.h" 
#include "..\inport\string.h" 
#include "..\inport\ctype.h"
#include "Programming_drive.h"
#include "my_system_LPC177x_8x.h"
#define BITSET(VAL,BITNO)   (VAL|=(0x01<<BITNO))
#define BITRES(VAL,BITNO)   (VAL&=~(0x01<<BITNO))
#define BITTST(VAL,BITNO)   (VAL&(0x01<<BITNO))
#define bitcpy(dv,dvbit,sv,svbit) ((sv&(0x01<<svbit))? (dv|=(0x01<<dvbit)):(dv&=~(0x01<<dvbit)))


/*------------START-------------*/
/*********************************************************************//**
 * @brief       Compare two message
 * @param[in]   Tx_Msg transmit message
 * @param[in]   Rx_Msg receive message
 * @return      Bool   should be:
 *                      - TRUE(1):  if two message is the same
 *                      - FALSE(0): if two message is different
 **********************************************************************/
uint8_t str_cmp(uint8_t *str1, uint8_t *str2,uint32_t cnt)
{
  uint32_t i;
  for (i = 0; i < cnt; i++) 
    { if( *str1 == *str2 )
	   { str1++;  str2++; }
	  else
	  return  0x00;
    }
  return 0xff;
}


void strupr(uint8_t *p)
{
  while(*p)
   {*p = (uint8_t)toupper(*p); p++;}
}

void strlwr(uint8_t *p)
{while(*p){*p =tolower(*p);p++;}
}

/*************************************************************************************************************/
/*函数 uint8_t ishexstr(uint8_t *str) 用于判别str所指字串是否为全为十六进制字符，若是返值非0，否则返值0*/
/*************************************************************************************************************/
uint8_t ishexstr(uint8_t *p)
{ uint8_t result;
  result = 0xff;
  if(*p == '\0') return(0);
  while(*p != '\0')
   {if(isxdigit(*p))
     {p++;	}
	else 
	 {result =0; 
	  break;
	 }
   }
  return result;
}
/*------------字串处理小程序_END-------------*/

/*------------------CPU板自测试程序_START---------------------------*/

/*-------------------CPU板自测试第0项：时钟检测-------------------------*/
void CLOCK_TEST(void)
{
 if((LPC_SC->CLKSRCSEL & 0x01) == 0)  /* CLKSRCSEL[0]=0:irc_clk用作sysclk； =1：osc_clk	用作sysclk	  */
   sprintf((char *)screen_outbuf,"   CLOCK(IRC): CPU-%dHz PRP-%dHz EMC-%dHz USB-%dHz\r\n",SystemCoreClock,
                                  PeripheralClock,EMCClock,USBClock);
 else
   sprintf((char *)screen_outbuf,"   CLOCK(OSC): CPU-%dHz PRP-%dHz EMC-%dHz USB-%dHz\r\n",SystemCoreClock,
                                  PeripheralClock,EMCClock,USBClock);
 my_putstr(screen_outbuf);
}

/*-------------------CPU板自测试第1项：实时时钟RTC测试-------------------------*/
void RTC_TEST()
{
  static uint8_t F_Init=0;
  if(F_Init == 0 )
   { F_Init	= 0xff;
     RTC_Init();  
     setrtdt(2014,8,8,23,58,0);
   }
  readrtdt();
  sprintf((char *)screen_outbuf," 1. RTC_TEST  -------------------------------------------%04d %02d-%02d-%02d.%02d:%02d\r\n",
                                 RTDTRDBF.year,RTDTRDBF.month,RTDTRDBF.day,RTDTRDBF.hour,RTDTRDBF.minute,RTDTRDBF.second);
  my_putstr(screen_outbuf);
  return;
}

/*-------------------CPU板自测试第2项：CPU板输入开关测试-------------------------*/
void READK_TEST(void)
{uint8_t k1,k2,k3,k4,k5;
 k1 = GPIO_ByteIn_Read(2,1)&0x0f;
 k2 = (GPIO_ByteIn_Read(2,0) >> 4)&0x0f;
 k3 = GPIO_ByteIn_Read(1,1);
 k4 = GPIO_ByteIn_Read(1,0);
 k5 = GPIO_ByteIn_Read(2,0)&0x0f;
 sprintf((char *)screen_outbuf," 2. READ_K    -------------------------K1:%02x   K2:%02x   K3:%02x   K4:%02x   K5:%02x\r\n",
                                 k1, k2 ,k3, k4, k5);
 my_putstr(screen_outbuf);
 return;
}

/*-------------------CPU板自测试第3项：串口UART3测试-------------------------*/
static uint8_t UART3_TST(void);

void UART3_TEST(void)
{
 if(UART3_TST())
   my_putstr(" 3. UART3_TEST--------------------------------------------------------PASSED\r\n");
 else
   my_putstr(" 3. UART3_TEST--------------------------------------------------------FAILED\r\n");
 return;
}

static uint8_t UART3_TST(void)
{
 #define SENDMAX 20
  uint8_t sendbuf[SENDMAX],rxbuf[SENDMAX],i;
  /*串口3初始化*/
  UART_Init(LPC_UART3, 115200, 0, 3, 0 );     /*波特率：115200，无校验，8bit数据，1停止位*/
  UART_TxCmd(LPC_UART3,1);				 /*1-允许发送；0-禁止发送*/
  for(i=0 ;i < SENDMAX; i++ )
   {sendbuf[i] = i;
    rxbuf[i]  = 0;
   }
  for(i=0;i<SENDMAX;i++)
   {
	UART_SendByte(LPC_UART3,sendbuf[i]);
	DelayMS(20);
	if(BITTST(LPC_UART3->LSR,0))        //等待有接收字符)
	      rxbuf[i]=UART_ReceiveByte(LPC_UART3);
	else
	 break;
   }
  if(str_cmp(sendbuf,rxbuf,SENDMAX))
	  return 0xff;
  else 
      return 0;
}

/*-------------------CPU板自测试第4项：CAN口1测试-------------------------*/
typedef struct
{uint8_t dscr1;
 uint8_t dscr2;
 uint8_t data[8];
}SJA1000BF_TypeDef;
SJA1000BF_TypeDef  TXB,RXB;

void CAN1_TEST(void)
{static uint8_t CAN_Test(void);
 if( CAN_Test() )
    my_putstr(" 4. CAN1_TEST --------------------------------------------------------PASSED\r\n");
 else
    my_putstr(" 4. CAN1_TEST --------------------------------------------------------FAILED\r\n");
}

static uint8_t CAN_Test(void)
{ void CAN_InitMessage(void);
  uint8_t CAN_SendMsg(LPC_CAN_TypeDef* pCan);
  uint8_t CAN_ReceiveMsg (LPC_CAN_TypeDef* pCan);
  uint8_t str_cmp(uint8_t *str1, uint8_t *str2,uint32_t cnt);
  uint32_t i;

  CAN_Init(LPC_CAN1, 80000);	                         //将CAN1的通讯波特率设置为80Kbps
  CAN_ModeConfig(LPC_CAN1, CAN_SELFTEST_MODE, ENABLE);	 //设置自测试模式
  LPC_CANAF->AFMR = 0X02;								 //验收滤波器旁路
  CAN_InitMessage();									 //填写SJA1000格式发送缓冲区
  while(BITTST(LPC_CAN1->SR, 2) == 0 );					 //CAN的发送缓冲区1空？不空一直等
    CAN_SendMsg(LPC_CAN1);								 //将JA1000格式发送缓冲区中内容经转换后送CAN的发送缓冲区1
  BITSET(LPC_CAN1->CMR,4);	                             //向CAN发自发自收命令
  
  for (i=0; i<5000000;i++)
   {if(BITTST(LPC_CAN1->SR,0))
       { CAN_ReceiveMsg(LPC_CAN1);
		 if (str_cmp((uint8_t *)&TXB, (uint8_t *)&RXB,10))
		   return 0xff; 
	     else 
   		  return 0;
	   }
	}
  return 0;
}

/*********************************************************************//**
 * @brief         Initialize transmit and receive message for Bypass operation
 * @param[in]     none
 * @return        none
 **********************************************************************/
static void CAN_InitMessage(void)
{
   uint32_t i;
   /*填写发送缓冲*/
   TXB.dscr1 = 0x05;
   TXB.dscr2 = 8;
   TXB.dscr2 = TXB.dscr2 | (0x07<<5);
   //BITSET(TXB.dscr2,4);		//RTR = 1
   for(i=0;i<8;i++)
	  TXB.data[i]= 0x30+i;
    /*清接收缓冲*/
   RXB.dscr1 =0;
   RXB.dscr2 =0;
   for(i=0;i<8;i++)
	  RXB.data[i]=0;
}  

static uint8_t CAN_SendMsg(LPC_CAN_TypeDef* pCan)
{
	//Check status of Transmit Buffer 1
	if (pCan->SR & (1 << 2))
	{
		/* Write CAN TFI*/
		pCan->TFI1 = (uint32_t)(TXB.dscr2&0x0f)<<16; //DLC->TFI1[19:16]
		if(BITTST(TXB.dscr2,4))
		  BITSET(pCan->TFI1,30);                     //1->RTR
		/* Write CAN ID*/
		pCan->TID1 = (uint32_t)(TXB.dscr1<<3)+(uint32_t)((TXB.dscr2>>5)&0x07);
		/*Write 8 data bytes*/
		pCan->TDA1 = *((uint32_t *)&TXB.data[0]);
		pCan->TDB1 = *((uint32_t *)&TXB.data[4]);
		 /*Select transmit Buff*/
		 pCan->CMR = 0x20;		  //选择TB1
		 return 0xff;
	}
	else
	{
		return 0x00;
	}
}


/********************************************************************//**
 * @brief		Receive message data
 * @param[in]	 canId			 The Id of the expected CAN component
 *
 * @param[in]	CAN_Msg point to the CAN_MSG_Type Struct, it will contain received
 *  			message information such as: ID, DLC, RTR, ID Format
 * @return 		Status:
 * 				- SUCCESS: receive message successfully
 * 				- ERROR: receive message unsuccessfully
 *********************************************************************/
static uint8_t CAN_ReceiveMsg (LPC_CAN_TypeDef* pCan)
{
	//LPC_CAN_TypeDef* pCan = CAN_GetPointer(canId);

	//check status of Receive Buffer
	if((pCan->SR &0x00000001))
	 {
	  RXB.dscr1 = (uint8_t)(pCan->RID >> 3);
	  RXB.dscr2 =	(uint8_t)((pCan->RID & 0x07) << 5);
	  RXB.dscr2 =	RXB.dscr2 + (uint8_t)(((pCan->RFS) & 0x000F0000) >> 16);
	  if(BITTST(pCan->RFS,30))
	   BITSET(RXB.dscr2,4);
	  *((uint32_t *)&RXB.data[0]) = pCan->RDA;
	  *((uint32_t *)&RXB.data[4]) = pCan->RDB;
	  pCan->CMR = 0x04;
	 }
	else
	 {
		// no receive message available
		return 0x00;
	 }

	return 0xff;
}

/*-------------------CPU板自测试第5项：定时/计数器0、1、2、3测试-------------------------*/
extern uint32_t sys_tim3_cnt,sys_tim1_cnt,sys_tim0_cnt;
extern uint8_t  F_TCNT_END;

void TIMER_CNT_Test(void)
{
 TIM0_Init();
 TIM1_Init();
 TIM2_Init(9000);
 TIMER_TEST_Enable();
 sys_tim3_cnt = 0;
 sys_tim1_cnt = 0;
 sys_tim0_cnt = 0;
 F_TCNT_END = 0;
 for(;;)
 {if(F_TCNT_END )
   {F_TCNT_END = 0;
	sprintf((char *)screen_outbuf," 5. TIME0:    ---------------------------------------------------------%05d \n\r",sys_tim0_cnt);
	my_putstr(screen_outbuf);
	sprintf((char *)screen_outbuf,"    TIME1:    ---------------------------------------------------------%05d \n\r",sys_tim0_cnt);
    my_putstr(screen_outbuf);
	return;  
   }
 }
}

/*-------------------CPU板自测试第6项：温度芯片LM75A测试-------------------------*/
void TEMP_TESTS(void)
{static uint8_t F_Init=0;
 if(F_Init == 0 )
  {
   F_TEMP_updata = 0;	
   F_Init = 0xff;
   LM75_Enable();
  }
 DelayMS(2000);
 my_putstr(" 6. TEMP_TEST ------------------------------------------------------");
 if(F_TEMP_updata)
   { F_TEMP_updata = 0;	
	 if(BITTST(LM75_TEMP,15))
	   {
	    sprintf((char *)screen_outbuf,"-%03d.%02dC\n\r",((LM75_TEMP&0x7fff)/100),((LM75_TEMP&0x7fff)%100));
	    my_putstr(screen_outbuf);
	   }
	 else
	   {
	    sprintf((char *)screen_outbuf,"+%03d.%02dC\n\r",(LM75_TEMP/100),(LM75_TEMP%100));
	    my_putstr(screen_outbuf);
	   }								  
   }
 else
   {
	my_putstr("--ERROR!\n\r");
   }
}

/*-------------------CPU板自测试第7项：SPI口RAM芯片FM25测试-------------------------*/
uint8_t SPIRAM_TEST(void)			
{ uint8_t st,i,IDbf[9];
  uint8_t ID_CST[9]={0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0xC2,0x22,0x00};
  FRAM_Initial(20000000);	                //初始化SPP0的速率为20Mhz.	  
  FRAM_readID(&IDbf[0]);
  st = 0xff;
  for(i=0;i<9;i++)
   {if(IDbf[i] != ID_CST[i])
      {st = 0; break; }
   }
  if(st)
   my_putstr(" 7. SPIRAM_TEST--------------------------------PASSED(FM25V02: 32K x 8 bits)\r\n");
  else
   my_putstr(" 7. SPIRAM_TEST-------------------------------------------------------FAILED\r\n");
  return st;
}

/*-------------------CPU板自测试第8项：EMC FLASH芯片SST39VF020测试-------------------------*/
uint8_t OSFLASH_TEST(void)
{
   if(Read_FlashID() == 0xBFD6 )
    {
	  my_putstr(" 8. EMC_FLASH_TEST ------------------------PASSED(SST39VF020: 256K x 8 bits)\r\n");
	  return 0xff;
	}
   else
    { my_putstr(" 8. EMC_FLASH_TEST ---------------------------------------------------FAILED\r\n");
      return 0;
	}
}

/*-------------------CPU板自测试第9项：EMC_RAM测试-------------------------*/
static uint32_t OSRAM_TEST(void);

void outRAM_TEST(void)
{uint32_t cnt_kram;
 disable();
 cnt_kram = OSRAM_TEST();
 sprintf((char *)screen_outbuf," 9. EMC_RAM_TEST-------------------------------------------------------%03dKB\r\n",cnt_kram);
 my_putstr(screen_outbuf);
 enable();						    
}

static uint32_t OSRAM_TEST(void)
{
 uint32_t data_base,tmp1,tmp2,k_cnt,base_adr,off_adr,*ptr;
 uint32_t i,j;
 //int32_t k_cnt;
 k_cnt = 0;
 base_adr = 0x80000000;
 ptr = (uint32_t *)base_adr;
 data_base = tmp1 = *ptr;
 *ptr = ~tmp1;
 tmp2 = *ptr;
 *ptr = data_base;
 if( tmp1 != ~tmp2 )
   return k_cnt;
 
 off_adr = 0;
 for(i=0;i<516;i++)
 {k_cnt++;
  off_adr = i*1024;
  ptr = (uint32_t *)(base_adr + off_adr);
  for(j=0;j<256;j++, ptr++ )
   { 
     tmp1 = *ptr;
     *ptr = ~tmp1;
	 if(off_adr && (j == 0))
	  {	if(	*((uint32_t *)base_adr) != data_base )
	      {						
		    *((uint32_t *)base_adr) = data_base ;
			return (k_cnt-1);
		  }		  
	  }
	 tmp2 = *ptr;
	 *ptr = tmp1;
	 if( tmp1 != ~tmp2 )
	  return (k_cnt -1);
   }
 }
 return (k_cnt -1); 
}
/*-------------------CPU板自测试第10项：USB口测试-------------------------*/
uint8_t USB_TEST(void)
{
 if(USBDISK_Initial() == 0)
   {
    my_putstr(" 10.USB_TEST  --------------------------------------------------------PASSED\r\n");
	  return 0xff;
   }
 else
   {
	my_putstr(" 10.USB_TEST  --------------------------------------------------------FAILED\r\n");
	  return 0x00;
   }
}

/*-------------------CPU板自测试第11项：USB口测试-------------------------*/
static uint8_t PPIO_TST(void);
void PPIO_TEST(void)
{
 if(PPIO_TST())
   my_putstr(" 11.PPIO_TST  --------------------------------------------------------PASSED\r\n");
 else
   my_putstr(" 11.PPIO_TST  --------------------------------------------------------FAILED\r\n");
 return;
}

static uint8_t PPIO_TST(void)
{ uint8_t i,test_ch;
  test_ch = 0x55; 
  for (i=0;i<4;i++)
	{ 
	    //outportb(0x44+i,test_ch); 
	 outportb(0x45,test_ch); 
	}
  DelayMS(20);
  for (i=0;i<4;i++)
	{ 
	    //if(inportb(0x44+i) != test_ch)
	 if(inportb(0x45) != test_ch)
	   return 0;
	} 
  test_ch = 0xAA; 
  for (i=0;i<4;i++)
	{ 
	    //outportb(0x44+i,test_ch); 
	  outportb(0x45,test_ch); 
	}
  DelayMS(20);
  for (i=0;i<4;i++)
	{ 
	    //if(inportb(0x44+i) != test_ch)
	 if(inportb(0x45) != test_ch)
	    return 0;
	}
   return 0xff;
}


/*--------------------CPU板自测试程序主函数CPU_Board_SFT()-----------------*/


void CPU_Board_SFT(void)					   
{uint8_t i;
 LM75_Enable();
 LED_TEST_Enable();
 while(1)
 { my_putstr("\n\r           ------------------ CPU_BOARD TEST ----------------\n\r ");
   CLOCK_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   RTC_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   READK_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   UART3_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   CAN1_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   TIMER_CNT_Test();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   TEMP_TESTS();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   SPIRAM_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   OSFLASH_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   outRAM_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   USB_TEST();
   if(UART_AskReceiveByte(LPC_UART0)) break;
   PPIO_TEST();
   for(i=0; i< 80; i++)
    {DelayMS(100);
	 if(UART_AskReceiveByte(LPC_UART0)) goto sftst_exit;
   	}
  }
sftst_exit:
  NVIC_DisableIRQ(USB_IRQn);			    /*禁止USB中断*/
  LED_TEST_Disable( );
  LM75_Disable();
  TIMER_TEST_Disable();
  GPIO_ByteOut_Out(0,1,0xff);				/*LED全熄灭*/
  if((LPC_SC->CLKSRCSEL & 0x01) == 0)       /* CLKSRCSEL[0]=0:irc_clk用作sysclk； =1：osc_clk	用作sysclk	  */
  GPIO_PinOut_Out(0, 8, 0);				    /* 如外部晶振有问题，点亮LED6                                    */
  return;
}

/*------------------CPU板自测试程序_END-------------------------*/

/*------------------CPU板DEBUG解释命令_START------------------------*/
uint8_t cmdbuf[5][20];
uint8_t __attribute__ ((aligned (4))) BinBuffer[4096];	         /*二进制待写入缓冲区          */

/*-------------CPU板DEBUG解释命令中通用下程序-----------------------------------------------------*/

void dishex(uint8_t ch)
{ static uint8_t hexdata;
  hexdata=ch/16;
  hexdata+=0x30;
  if(hexdata>=0x3a) hexdata+=0x07;
  my_putchar(hexdata);
  hexdata=ch%16;
  hexdata+=0x30;
  if(hexdata>=0x3a) hexdata+=0x07;
  my_putchar(hexdata);
}

void dishex16(uint16_t u16_hexd)
{static uint8_t u8_hexd;
 u8_hexd = (uint8_t)((u16_hexd >> 8) & 0xff);
 dishex(u8_hexd);
 u8_hexd = (uint8_t)(u16_hexd & 0xff);
 dishex(u8_hexd);
}

void dishex32(uint32_t u32_hexd)
{static uint16_t u16_hexd;
 u16_hexd = (uint16_t)((u32_hexd >> 16) & 0xffff);
 dishex16(u16_hexd);
 u16_hexd = (uint16_t)(u32_hexd & 0xffff);
 dishex16(u16_hexd);
}


/*------------------CPU板DEBUG解释命令1:内存显示命令(MD)--------------------*/
void Dis_mem_data(uint8_t *madr);

void mem_dis(void)
{uint32_t u32_hexd;
 uint8_t input_ch;

 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
  while(1)
   {
	Dis_mem_data((uint8_t *)u32_hexd);
	for(;;)
	 {input_ch = my_getch();
	  if(input_ch == 0x20 )
		{u32_hexd +=128; break;}
	  if( input_ch ==  'Q' || input_ch ==  'q'  )
		return;
	}
   }
}

void Dis_mem_data(uint8_t *madr)
{ uint8_t i,j;
  uint8_t *p;
  p = madr;
  //my_putstr("\n\r");
  for(i=0;i<8;i++)
  {//my_putstr("\n\r");
   sprintf((char *)screen_outbuf,"%08p",p);
   my_putstr(screen_outbuf);
   my_putstr("   ");
   for(j=0;j<16;j++,p++)
   { dishex(*p);
     if(j == 7)
	  {my_putchar('-');	my_putchar('-');}
	 else
      my_putchar(' ');
   }
   my_putstr("\n\r");
  }
  my_putstr("\n\r");
}

/*------------------CPU板DEBUG解释命令2:内存字节修改命令(MWB)--------------------*/
void mwb_prg(void)
{static uint8_t *p_u8,u8_hexd,F_mdf,input_ch;
 uint32_t u32_hexd;

 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 p_u8 = (uint8_t *)u32_hexd;
 F_mdf = 0;
 while(1)
 {
  sprintf((char *)screen_outbuf,"%08p  ",p_u8);
  my_putstr(screen_outbuf);
  u8_hexd = *p_u8;
  dishex(u8_hexd);					    
  my_putstr("  ");
  F_mdf = 0;
  u8_hexd = 0;
  for(;;)
   {input_ch = my_getch();
    input_ch = toupper(input_ch);
    if(input_ch == 'Q') 
	  {
	   my_putstr("\n\r");
	   return;
	  }
    if((input_ch >= 0x30)&&(input_ch < 0x3a))
     { u8_hexd =(u8_hexd << 4) + (input_ch-0x30); F_mdf =0xff;  my_putchar(input_ch); }
	if((input_ch >= 0x41)&&(input_ch < 0x47))
	 { u8_hexd =(u8_hexd << 4) + (input_ch-0x37); F_mdf =0xff;  my_putchar(input_ch); }
	if( input_ch == 0x0d )
	  { if(F_mdf)
	     {
		  if(p_u8 >= (uint8_t *)0x80000)
		  *p_u8 = u8_hexd;
		 }
		p_u8++; 
		my_putstr("\r\n");
		break;
	  }
   }
 }	
}
/*------------------CPU板DEBUG解释命令3:内存半字修改命令(MWH)--------------------*/
void mwh_prg(void)
{static uint8_t F_mdf,input_ch;
 static uint16_t u16_hexd,*p_u16;
 static uint32_t u32_hexd;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);	
 BITRES(u32_hexd, 0 );
 p_u16 = (uint16_t *)u32_hexd;
 while(1)
 {
  sprintf((char *)screen_outbuf,"%08p  ",p_u16);
  my_putstr(screen_outbuf);
  u16_hexd = *p_u16;
  dishex16(u16_hexd);		    
  my_putstr("  ");
  u16_hexd = 0;
  F_mdf = 0;
  for(;;)
   {input_ch = my_getch();
    input_ch = toupper(input_ch);
    if(input_ch == 'Q') 
	  {my_putstr("\n\r");  return; }
    if((input_ch >= 0x30)&&(input_ch < 0x3a))
     { u16_hexd =(u16_hexd << 4) + (input_ch-0x30); F_mdf =0xff;  my_putchar(input_ch); }
	if((input_ch >= 0x41)&&(input_ch < 0x47))
	 { u16_hexd =(u16_hexd << 4) + (input_ch-0x37); F_mdf =0xff;  my_putchar(input_ch); }
	if( input_ch == 0x0d )
	  { if(F_mdf)
	     {
		  if(p_u16 >= (uint16_t *)0x80000)
		   *p_u16 = u16_hexd;
		 }
		p_u16++; 
		my_putstr("\r\n");
		break;
	  }
   }
 }	
}

/*------------------CPU板DEBUG解释命令4:内存字修改命令(MWW)--------------------*/
					 
void mww_prg(void)
{static uint8_t F_mdf,input_ch;
 static uint32_t u32_hexd,*p_u32;
 
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u32_hexd = u32_hexd & ~0x00000003;
 p_u32 = (uint32_t *)u32_hexd;
 while(1)
 {
  sprintf((char *)screen_outbuf,"%08p  ",p_u32);
  my_putstr(screen_outbuf);
  u32_hexd = *p_u32;
  dishex32(u32_hexd);		    
  my_putstr("  ");
  u32_hexd = 0;
  F_mdf = 0;
  for(;;)
   {input_ch = my_getch();
    input_ch = toupper(input_ch);
    if(input_ch == 'Q') 
	 {
	  my_putstr("\n\r");
	  return;
	 }
    if((input_ch >= 0x30)&&(input_ch < 0x3a))
     { u32_hexd =(u32_hexd << 4) + (input_ch-0x30); F_mdf =0xff;  my_putchar(input_ch); }
	if((input_ch >= 0x41)&&(input_ch < 0x47))
	 { u32_hexd =(u32_hexd << 4) + (input_ch-0x37); F_mdf =0xff;  my_putchar(input_ch); }
	if( input_ch == 0x0d )
	  { if(F_mdf)
	    {
		 if(p_u32 >= (uint32_t *)0x80000)
		 *p_u32 = u32_hexd;
		}
		p_u32++; 
		my_putstr("\r\n");
		break;
	  }
   }
 }	
}
/*------------------CPU板DEBUG解释命令5:EMC_PPI输入命令(PPI)--------------------*/
void ppi_prg(void)
{uint32_t u32_hexd;
 uint8_t u8_hexd,input_u8,input_ch;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u8_hexd = (uint8_t)u32_hexd;
 while(1)
  {input_u8 = inportb(u8_hexd);
   dishex(u8_hexd);
   my_putstr("  ");
   dishex(input_u8);	
   my_putstr("\n\r");				 
   for(;;)
	{input_ch = my_getch();
	 if( input_ch == 0x0d )break;
	 if(input_ch == 0x20 )
	  {u8_hexd ++; break;}
	 if( input_ch ==  'Q' || input_ch ==  'q'  )
	   return;
	}
  }
}					
/*------------------CPU板DEBUG解释命令6:跳转执行程序命令(GO)--------------------*/
void go_prg(void)					
{uint32_t u32_hexd;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 JUMP_TO(u32_hexd);
 return;	  
}
/*------------------CPU板DEBUG解释命令7:跳转到指定向量处的程序执行命令(GOV)--------------------*/
void gov_prg(void)
{uint32_t u32_hexd; 
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u32_hexd = (u32_hexd & ~0x03)+4;
 u32_hexd = *((uint32_t *)u32_hexd);					 
 JUMP_TO(u32_hexd);					 
 return;
}
/*------------------CPU板DEBUG解释命令8:改写片内FLASHB保护代码命令(CRP)--------------------*/
uint8_t write1_CRP(uint32_t new_crp);
void crp_prg(void)
{uint32_t u32_hexd; 
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);					 
 write1_CRP(u32_hexd);		             
 return;
}		            					

uint8_t write1_CRP(uint32_t new_crp)
{ uint32_t *ptr,old_crp,FLASH_adr;
  uint8_t st;
  FLASH_adr = 0X0000; 
  memcpy((uint8_t *)&BinBuffer[0], (uint8_t *)FLASH_adr, 4096);
  ptr =	 (uint32_t *)&BinBuffer[0x2fc];
  old_crp = *ptr;
  if(old_crp == new_crp ) return 0xff;
  *ptr = new_crp;
  disable();
  st = IAP_EraseSector(0, 0);  /* 返值=0表示成功*/
  if(st) return 0xff;	  
  st=IAP_Write((uint8_t *)FLASH_adr, (uint8_t *)BinBuffer, 4096);		/* 返值=0表示成功*/
  enable();
  if(st) return 0xff;
  return 0;
}
/*------------------CPU板DEBUG解释命令9:通用IO口输入命令(GPI)--------------------*/
void gpi_prg(void)
{uint8_t  u8_hexd,u8_digd,input_u8,input_ch;
 uint32_t u32_hexd;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u8_hexd = (uint8_t)u32_hexd;
 sscanf((char *)cmdbuf[2],"%x" ,&u32_hexd);
 u8_digd = (uint8_t)u32_hexd;
 if( ( (u8_hexd > 5) || (u8_digd > 3) )  ||  ( (u8_hexd == 5) && (u8_digd > 1) ))
  {my_putstr("ERROR !\n\r");return;}
 while(1)
  {input_u8 = GPIO_ByteIn_Read(u8_hexd, u8_digd);
   my_putchar('P');
   my_putchar( u8_hexd + 0x30 );					  
   my_putstr(":Byte");					  
   my_putchar( u8_digd + 0x30 );					  
   my_putstr(" -> ");					  					  
   dishex(input_u8);
   my_putstr("\n\r");
   for(;;)
   	{input_ch = my_getch();
	 if( input_ch == 0x0d )break;
	 if( input_ch ==  'Q' || input_ch ==  'q'  )
	   return;
	}				  	
  }					  
}
/*------------------CPU板DEBUG解释命令10:EMC_IO口输出命令(PPO)--------------------*/
void ppo_prg(void)
{uint32_t  u32_hexd;
 uint8_t  u8_hexd,u8_digd;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u8_hexd = (uint8_t)u32_hexd; u32_hexd = 0;
 sscanf((char *)cmdbuf[2],"%x" ,&u32_hexd);
 u8_digd = (uint8_t)u32_hexd;
 outportb(	u8_hexd, u8_digd );
 dishex(u8_hexd);
 my_putstr(" <- ");
 dishex(u8_digd);	
 my_putstr("\n\r");
 return;		              
}					 
/*------------------CPU板DEBUG解释命令11:存储器比较命令(MCMP)--------------------*/
void mcmp_prg(void)
{uint32_t  u32_hexd,i;
 uint8_t *M_dadr,*M_sadr;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 M_dadr = (uint8_t *)u32_hexd;
 u32_hexd = 0;
 sscanf((char *)cmdbuf[2],"%x" ,&u32_hexd);
 M_sadr =  (uint8_t *)u32_hexd;;
 u32_hexd = 0;
 sscanf((char *)cmdbuf[3],"%x" ,&u32_hexd);
 for(i=0; i < u32_hexd; i++,M_dadr++,M_sadr++)
   {if(*M_dadr !=  *M_sadr )
	 {my_putstr("\n\r");
	  sprintf((char *)screen_outbuf,"%08p  ",M_dadr);
      my_putstr(screen_outbuf);
	  dishex(*M_dadr);
	  my_putstr("<---->");
	  sprintf((char *)screen_outbuf,"%08p  ",M_sadr);
      my_putstr(screen_outbuf);
	  dishex(*M_sadr);
	  if(UART_AskReceiveByte(LPC_UART0)) 
	  {my_putstr("\n\r");return;}
	 }
   }
  my_putstr("\n\r");
  return;
}
/*------------------CPU板DEBUG解释命令11:存储器拷贝命令(MCPY)--------------------*/
void mcpy_prg(void)
{uint32_t  u32_hexd;
 uint8_t   *M_dadr,	*M_sadr;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 M_dadr = (uint8_t *)u32_hexd;
 u32_hexd = 0;
 sscanf((char *)cmdbuf[2],"%x" ,&u32_hexd);
 M_sadr =  (uint8_t *)u32_hexd;;
 u32_hexd = 0;
 sscanf((char *)cmdbuf[3],"%x" ,&u32_hexd);
 if(u32_hexd == 0 )return;
 if(M_dadr >= (uint8_t *)0x80000)
 memcpy( M_dadr,M_sadr,u32_hexd);
 my_putstr("\n\r");
 return;
}

/*------------------CPU板DEBUG解释命令12:GPIO字节输出设置命令(GPOS)--------------------*/
void gpos_prg(void)
{uint32_t u32_hexd;
 uint8_t  u8_hexd,u8_digd,input_u8;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u8_hexd = (uint8_t)u32_hexd;		             
 u32_hexd = 0;	 
 sscanf((char *)cmdbuf[2],"%x" ,&u32_hexd);
 u8_digd = (uint8_t)u32_hexd;
 if( ( (u8_hexd > 5) || (u8_digd > 3) )  ||  ( (u8_hexd == 5) && (u8_digd > 1) ))
	{my_putstr("ERROR !\n\r");return;}
 u32_hexd = 0;
 sscanf((char *)cmdbuf[3],"%x" ,&u32_hexd);
 input_u8 = (uint8_t)u32_hexd;
 GPIO_ByteOut_Set(u8_hexd, u8_digd, input_u8);
 my_putstr("\n\r");
 return;
} 

/*------------------CPU板DEBUG解释命令13:GPIO字节输出命令(GPO)--------------------*/
void gpo_prg(void)
{uint32_t u32_hexd;
 uint8_t  u8_hexd,u8_digd,input_u8;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u8_hexd = (uint8_t)u32_hexd;		             
 u32_hexd = 0;	 
 sscanf((char *)cmdbuf[2],"%x" ,&u32_hexd);
 u8_digd = (uint8_t)u32_hexd;
 if( ( (u8_hexd > 5) || (u8_digd > 3) )  ||  ( (u8_hexd == 5) && (u8_digd > 1) ))
	{my_putstr("ERROR !\n\r");return;}
 u32_hexd = 0;
 sscanf((char *)cmdbuf[3],"%x" ,&u32_hexd);
 input_u8 = (uint8_t)u32_hexd;
 GPIO_ByteOut_Out(u8_hexd, u8_digd, input_u8);
 my_putstr("\n\r");
 return;
}
 
void gpops_prg(void)
{uint32_t  u32_hexd;
 uint8_t   u8_hexd,u8_digd,input_u8;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u8_hexd = (uint8_t)u32_hexd; u32_hexd = 0;
 sscanf((char *)cmdbuf[2],"%d" ,&u32_hexd);
 u8_digd = (uint8_t)u32_hexd; u32_hexd = 0;
 sscanf((char *)cmdbuf[3],"%d" ,&u32_hexd);
 input_u8 = (uint8_t)u32_hexd;
 if( ( (u8_hexd > 5) || (u8_digd > 31) )  ||  ( (u8_hexd == 5) && (u8_digd > 4) ))
  {my_putstr("ERROR !\n\r");return;}
 GPIO_PinOut_Set(u8_hexd, u8_digd, input_u8);
 my_putchar('P');
 my_putchar( u8_hexd + 0x30 );
 my_putstr(":Pin");
 u8_hexd = (u8_digd/10) << 4;
 u8_hexd |= (u8_digd%10);
 dishex(u8_hexd);
					  
 my_putstr(" <- ");
 if(input_u8)
    my_putchar('1');
 else
    my_putchar('0');					  
 my_putstr("\n\r");
 return;
}		 

void gpopo_prg(void)
{uint32_t  u32_hexd;
 uint8_t   u8_hexd,u8_digd,input_u8;
 sscanf((char *)cmdbuf[1],"%x" ,&u32_hexd);
 u8_hexd = (uint8_t)u32_hexd; u32_hexd = 0;
 sscanf((char *)cmdbuf[2],"%d" ,&u32_hexd);
 u8_digd = (uint8_t)u32_hexd; u32_hexd = 0;
 sscanf((char *)cmdbuf[3],"%d" ,&u32_hexd);
 input_u8 = (uint8_t)u32_hexd;
 if( ( (u8_hexd > 5) || (u8_digd > 31) )  ||  ( (u8_hexd == 5) && (u8_digd > 4) ))
  {my_putstr("ERROR !\n\r");return;}
 GPIO_PinOut_Out(u8_hexd, u8_digd, input_u8);
 my_putchar('P');
 my_putchar( u8_hexd + 0x30 );
 my_putstr(":Pin");
 u8_hexd = (u8_digd/10) << 4;
 u8_hexd |= (u8_digd%10);
 dishex(u8_hexd);
 my_putstr(" <- ");
 if(input_u8)
    my_putchar('1');
 else
    my_putchar('0');					  
 my_putstr("\n\r");
 return;
}		
					 
//unsigned short k,argc;
#define  MD         1    /*存储器按字节显示:           MD    M_adr <CR>                  */
#define  MWB        2    /*存储器按字节改写:           MWB   M_adr <CR>                  */
#define  MWH        3    /*存储器按半字改写:           MWW   M_adr <CR>                  */
#define  MWW        4    /*存储器按字改写:             MWH   M_adr <CR>                  */
#define  MCMP       5    /*存储器比较:                 MCMP  M_sadr M_dadr M_size <CR>   */
#define  MCPY       6    /*存储器拷贝:                 MCPY  M_dadr M_sadr M_size <CR>   */
#define  GPOS       7    /*GPIO字节输出设置:           GPOS  P_adr  BYTE_adr  Value<CR>  */
#define  GPO        8    /*GPIO字节输出:               GPO   P_adr  BYTE_adr  Value<CR>  */
#define  GPI        9    /*GPIO字节输入:               GPI   P_adr  BYTE_adr<CR>         */
#define  GPOPS      10   /*GPIO输出位设置:             GPOPS P_adr  PIN_adr  Value<CR>   */
#define  GPOPO      11   /*GPIO输出位输出:             GPOPC P_adr  PIN_adr  Value<CR>   */
#define  PPO        12   /*PPIO字节输出:               PPO  PPIO_adr  Value<CR>	         */
#define  PPI        13   /*PPIO字节输入:               PPI  PPIO_adr<CR>		         */
#define  GO         14   /*转到指定程序执行:           GO   PRG_adr<CR>		             */
#define  GOV        15   /*转向量表+4所指程序地址执行: GOV   V_adr<CR>		             */
#define  CRP        16	 /*							   CRP   NEW_CRP<CR>				 */
#define  OSFPRGHEX  17   /*片外FLASH编程1：			 OSFPRGHEX<CR>					     */
 					     /*将U盘中定位于0x90000000处的hex文件写入到片外FLASHM。          */
#define  OSFPRGBIN  18   /*片外FLASH编程2：			 OSFPRGBIN<CR>					     */
 					     /*将片内0x00040000处的Bin文件写入到片外FLASH的0x90000000处。    */

typedef struct
{
 uint8_t command[20];
 uint8_t cmd_No;
}CMD_TypeDef;

CMD_TypeDef cmd_table[]={
   "md",MD,
   "mwb",MWB,
   "mwh",MWH,
   "mww",MWW,
   "mcmp",MCMP,
   "mcpy",MCPY,
   "gpo",GPO,
   "gpi",GPI,
   "gpos",GPOS,
   "gpops",GPOPS,
   "gpopo",GPOPO,
   "ppo",PPO,
   "ppi",PPI,
   "go", GO,
   "gov",GOV,
   "crp",CRP,
   "osfprghex",OSFPRGHEX,
   "osfprgbin",OSFPRGBIN,
   "",0xff
  };

uint8_t  inflash_to_outflash(void);
uint32_t USB_TO_FLASH(uint8_t F_out_side);

uint8_t search_cmd(uint8_t *s)
{ 
  uint8_t i;
  //uint8_t *p;
  strlwr(s);
  //while(*p){*p =tolower(*p);p++;}
  for(i=0;*cmd_table[i].command;i++)
    {if( !strcmp((const char *)cmd_table[i].command, (const char *)s) )
	  return cmd_table[i].cmd_No;
	}
  return 0;
}


void SFtest_and_CMDexplain(void)
{static  uint8_t   CMD_CODE;
 uint8_t   *p, argc;
 uint32_t  i;
//SFtest_and_Debug:  
  /*串口0初始化*/
  UART_Init(LPC_UART0, 19200, 0, 3, 0 );     /*波特率：19200，无校验，8bit数据，1停止位*/
  UART_TxCmd(LPC_UART0,  1);				 /*1-允许发送；0-禁止发送*/
  /*运行CPU板自测程序*/
  CPU_Board_SFT();
  for(;;)
  { 
   loop_start:
	my_putstr("\n\rIN: ");
    for(i=0;i<5;i++)
	  cmdbuf[i][0] = '\0';
    p = my_get_str();
    sscanf((char *)p,"%s %s %s %s %s ",cmdbuf[0],cmdbuf[1],cmdbuf[2],cmdbuf[3],cmdbuf[4]);
    for(i=0,argc=0;i<5;i++)
     {if(cmdbuf[i][0] !='\0')
	   {argc++; 
		//strlwr((uint8_t *)&cmdbuf[i]);
	   }
	 }
	if((argc > 4) || ( argc < 1))
	 {my_putstr("ERROR !\n\r");
		 goto loop_start;
	 }

	if(argc > 1)
	 {for(i=1;i<argc;i++)
	   if( ishexstr(cmdbuf[i]) == 0) 
	    {my_putstr("ERROR !\n\r");
		 goto loop_start;
	  	}
	 }
  	CMD_CODE=search_cmd(cmdbuf[0]);
	if(argc == 4)
	 {switch(CMD_CODE)
	    { case MCMP:
		         { mcmp_prg();	goto loop_start;  }
	      case MCPY:
		         { mcpy_prg();	goto loop_start;  }
		  case GPOS:
		         { gpos_prg();  goto loop_start;  }
		  case  GPO: 
				 { gpo_prg();   goto loop_start;  }
		  case GPOPS:
		         { gpops_prg();	goto loop_start;  }
		  case GPOPO: 
		         { gpopo_prg();	goto loop_start;  }
		  default:
	             {my_putstr("ERROR !\n\r");goto loop_start;}
		}
	 }
   if (	argc == 3)
	 {switch(CMD_CODE)
	    { case GPI:  
		         { gpi_prg();  goto loop_start;}
		  case PPO:  
		         { ppo_prg();  goto loop_start;}
		  default:
	             {my_putstr("ERROR !\n\r");goto loop_start;}
		}
	 }
   if( argc == 2 )
     {switch(CMD_CODE)
	    { case MD: 
		         { mem_dis();   goto loop_start;}
	      case MWB: 
		         { mwb_prg( );	goto loop_start;   }

		  case MWH:
		         { mwh_prg( );	goto loop_start;   }
		  case MWW: 
		         { mww_prg();	goto loop_start;  }
		  case PPI: 
		         { ppi_prg();	goto loop_start;  }
		  case GO:  
		         { go_prg();  	goto loop_start;  }
		  case GOV: 
		         { gov_prg();	goto loop_start;  }
		  case CRP:
		         { crp_prg();	goto loop_start;  }
		  default:
	             { my_putstr("ERROR !\n\r"); goto loop_start;}
		}
	 }
	if( argc == 1)
	 {switch(CMD_CODE)
	   {case OSFPRGHEX:
	               { USB_TO_FLASH(0xff);	   goto loop_start; }
	    case OSFPRGBIN:
		           { inflash_to_outflash();    goto loop_start; }
		default:
	               { my_putstr("ERROR !\n\r"); goto loop_start; }
	   }
	 }
	goto loop_start;
 }
}
/*--------------------FLASH编程部分_START -----------------------------------*/

/*****************************************************************************************************
 * 在此转换的Hex文件的条件：(1)程序的代码必须在0x40000~0x7ffff的256K空间内，且以0x40000为向量始地址。
 *							(2)程序中不允许有超出上述地址范围内赋常量的语句(如startup_LPC177x_8x.s中
 *                             在0x2fc~0x2ff间4个单元赋值0xFF)。
 * 该程序的集成环境设置如下：Project\Options for Taget'Taget1'...\Taget\Read/Onlay Memory Areas 的
 * IROM1 Start设为0x00     Size设为0x40000	default不勾 Startup不选；
 * IROM2 Start设为0x40000  Size设为0x40000	default勾上 Startup选中；
 *****************************************************************************************************/
//uint8_t __attribute__ ((aligned (4))) BinBuffer[4096];	         /*二进制待写入缓冲区          */

uint32_t  BinBuf_bytes_cnt;										 /*二进制待写入缓冲区字节计数器 */
uint32_t  BASE_START_ADR;										 /*FLASH编程起始地址            */
uint32_t  FLASH_Ext_Adr;
uint32_t  FLASH_Write_Star_Adr;									 /*每次写入IAP_FLASH时的写入首地址(必须256字节对齐)*/
uint32_t  FLASH_Current_Adr;									 /*当前IAP_FLASH地址*/
uint8_t   F_Start_Adr;											 /*检测到扩展地址0x40000标志*/
uint8_t   F_Generate_Sadr;								         /*重新生成IAP_FLASH写入地址标志*/
uint32_t  File_Size;									         /*已打开文件的字节数*/										
uint32_t  tot_bytes_analysed;								     /*已处理文件的字节数*/
uint8_t   *file_IObuffer;										 /*已打开文件的缓冲区处理指针*/
uint8_t   F_WRITE_0X9000;										 /*USB_TO_FLASH程序用于区分写片外/片内FLASH的标识*/


static void Clear_BinBuf(void)
{uint32_t i;
  for(i=0;i<1024;i++)
   BinBuffer[i] = 0xff;
}

static uint8_t asc_to_hex(uint8_t ch)
{
 if(ch >= 0x61) return(ch-0x57);
 if(ch >= 0x41) return(ch-0x37);
 return(ch-0x30);
}

uint8_t my_Flash_Prg(uint32_t Flash_Adr ,uint32_t Smem_Adr ,uint32_t Prg_size )
{ uint8_t st;
   disable(); 
   st = Flash_Program(Flash_Adr-0x90000000 , Smem_Adr , Prg_size);	  /*返回非0表示操作成功，返回0表示操作失败*/
   st = str_cmp((uint8_t *)Flash_Adr , (uint8_t *)Smem_Adr , Prg_size);
   enable();
   Clear_BinBuf();
   if(st)
	 return 0;
   else
     return 0xFF;
}

uint8_t   my_IAP_erase(uint32_t start_sec,uint32_t end_sec)
{
  uint8_t st;
   disable();
   st=IAP_EraseSector(start_sec, end_sec);  /* 返值=0表示成功*/
   enable();
   return (st);
}

uint8_t my_IAP_Write(uint8_t *dest, uint8_t *source, uint32_t size )
{
   uint8_t st;
   disable();
   st=IAP_Write(dest, source, size);		/* 返值=0表示成功*/
   st= IAP_Compare(dest, source, size);		/* 返值=0表示成功*/
   enable();
   Clear_BinBuf();
   return(st);
}

uint8_t my_Chip_Erase(void)					/*正确返回0*/
{uint8_t st;
  uint16_t i;
  st = 0;
  disable();
  for(i=0; i< 64 ;i++ )
   {
    if(Sector_Erase(i)== 0)
	 {st = 0xff; break;} 
   }
  enable();
  return (st);
}

uint8_t Line_process(uint8_t *linebuff,uint32_t cnt)
{uint32_t tmp_adr;
 uint32_t i;
 uint8_t ceke_sum,st=0;
 ceke_sum = 0;
 for(i=0;i<cnt;i++)
  	ceke_sum += linebuff[i];
 if(ceke_sum) return(6);
 switch (linebuff[3])			//判读本行数据类型：0x00-数据；0x01-文件结束；0x04-线性地址扩展；0x05-程序运行入口地址。
  {	case 0x05:				          /*程序运行入口地址记录，可不处理。*/
              break;
	case 0x04:						  /*扩展地址记录处理*/
			  FLASH_Ext_Adr = (uint32_t)ReadBE16U(&linebuff[4]);	   /*取16位扩展地址（大端存放）*/
			  FLASH_Ext_Adr = (FLASH_Ext_Adr<<16)& 0xffff0000;
			  F_Generate_Sadr = 0xFF; /*指示扩展基地址发生变化，且新的扩展基地址在FLASH_Ext_Adr变量中*/
			  break;	
	case 0x01:						  /*结束记录*/
	          if(BinBuf_bytes_cnt)					                    
                 { if(F_WRITE_0X9000)
					st= my_Flash_Prg(FLASH_Write_Star_Adr, (uint32_t)&BinBuffer[0], 1024);
				   else
				    st= my_IAP_Write((uint8_t *)FLASH_Write_Star_Adr, BinBuffer, 1024);
	               FLASH_Write_Star_Adr += 1024 ;
	               BinBuf_bytes_cnt = 0;
	              }
			  break;
	case 0x00:						  /*数据记录*/
	         tmp_adr = FLASH_Ext_Adr + (uint32_t)ReadBE16U(&linebuff[1]); /*本行数据在FLASH中的起始地址->tmp_adr*/
	         if(F_Generate_Sadr)							/*扩展基地址发生变化*/
			  {
			   F_Generate_Sadr = 0;
			   if(tmp_adr == BASE_START_ADR )				/*合法编程起始地址0x00040000/0x90000000*/
			    { 
				  F_Start_Adr = 0xFF;						/*置已检出起始地址标志*/
				  FLASH_Write_Star_Adr = tmp_adr;		    /*初始化本块写入FLASH的首地址*/
				  FLASH_Current_Adr    = tmp_adr;			/*初始化FLASH当前写入地址*/
				  BinBuf_bytes_cnt     = 0;					/*清待写入缓冲区中字节计数器*/
				  if(F_WRITE_0X9000)
					 st = my_Chip_Erase();					    /*擦除片外FLASH*/
				  else
				     st = my_IAP_erase(22,29);					/*擦除FLASH的0x40000~0x7ffff*/
				}
			   if( FLASH_Current_Adr != tmp_adr) 
			     { if(BinBuf_bytes_cnt)
				    {
					  if(F_WRITE_0X9000)
					    st = my_Flash_Prg(FLASH_Write_Star_Adr, (uint32_t)&BinBuffer[0], 1024);
					  else
					    st = my_IAP_Write((uint8_t *)FLASH_Write_Star_Adr, BinBuffer, 1024);
					  BinBuf_bytes_cnt = 0;
				    }
				   FLASH_Current_Adr    = tmp_adr;
				   FLASH_Write_Star_Adr = 256*(tmp_adr/256);	     /*256字节边界调整*/
				   BinBuf_bytes_cnt     = tmp_adr%256;
				 }
			   }
			 if( (tmp_adr >= BASE_START_ADR) && ( tmp_adr <= (BASE_START_ADR+ 0x3FFFF) ) && F_Start_Adr )
			  {if( tmp_adr != FLASH_Current_Adr )
			     {if( tmp_adr  <= (FLASH_Write_Star_Adr + 1024))		  /* 在当前的待写入缓冲区范围内*/
				    {
					  FLASH_Current_Adr    = tmp_adr;
					  BinBuf_bytes_cnt     = (tmp_adr-FLASH_Write_Star_Adr);
				    }
				  else
				    {if(BinBuf_bytes_cnt)
				       {
					     if(F_WRITE_0X9000)
					      st = my_Flash_Prg(FLASH_Write_Star_Adr, (uint32_t)&BinBuffer[0], 1024);
						 else
						  st = my_IAP_Write((uint8_t *)FLASH_Write_Star_Adr, BinBuffer, 1024);
					     BinBuf_bytes_cnt = 0;
				       }
				     FLASH_Current_Adr    = tmp_adr;
				     FLASH_Write_Star_Adr = 256*(tmp_adr/256);	     /*256字节边界调整*/
				     BinBuf_bytes_cnt     = tmp_adr%256;
				    }
				 }
			 if( linebuff[0] && linebuff[0] <=64 )					 /*记录中字节数magic的文件要改为64*/
                 {
				   for( i=0; i < linebuff[0]; i++ )
                    { 
					  BinBuffer[BinBuf_bytes_cnt] = linebuff[4+i];
			          BinBuf_bytes_cnt++;
					  FLASH_Current_Adr++;
                      if(BinBuf_bytes_cnt>=1024)					                    //写满1024字节
                       {
				        if(F_WRITE_0X9000)
					      st = my_Flash_Prg(FLASH_Write_Star_Adr, (uint32_t)&BinBuffer[0], 1024);
						else
						  st = my_IAP_Write((uint8_t *)FLASH_Write_Star_Adr, BinBuffer, 1024);
	                    FLASH_Write_Star_Adr += 1024 ;
	                    BinBuf_bytes_cnt = 0;
	                   }
                     }
				  }
			else
				return (0xff);										  
		  }
		 break;
	 default : return (0xff);
   }
  return (st);
}


uint32_t USB_TO_FLASH(uint8_t F_out_side)				  //正常返回0 ,输入参数为0，表示写入片内FLASH;非0写片外FLASH。
{
 static uint8_t Binlinebuff[80];						  /*二进制行记录缓冲区(每行的ASC码数据转换成二进制后放入该缓冲区)*/
 static uint8_t Head_code[16]=":02000004";				  /*任一hex文件第一行必须是扩展地址记录*/
 static int32_t fdr;									  /*待打开文件句柄*/
 //uint32_t File_Size;									  /*已打开文件的字节数*/
 uint32_t err_code=0;									  /*错误代码*/
 uint32_t Bin_Lcnt;										  /*二进制行记录缓冲区已有字节计数器*/
 uint32_t i;											  /*循环计数变量*/
 uint8_t  H_4Bit=0;										  /*下一处理数据为高4位标志*/
 uint8_t  tmpByte;										  /*高低4位处理中的临时存放单元*/
 uint8_t  *READ_FILE;
 //uint8_t  FILE0X9[20]="PCBSFT9A_B.hex";
 //uint8_t  FILE0X4[20]="PCBSFT4A_B.hex";
 uint8_t  FILE0X4[20]="PCB0004H.hex";
 uint8_t  FILE0X9[20]="PCB9000H.hex";
 //tmpByte =	GPIO_ByteIn_Read(2, 1);						  /*读K1值  */
 /*待增加数码显示板指示*/
 file_IObuffer = (uint8_t *)0x20004000;
 F_WRITE_0X9000	  = F_out_side;								  /*全局变量，由主函数调用前赋值，0-写到片内，非0-写片外*/
 if(USBDISK_Initial())
   {
    my_putstr("USB_disk Initialize Error !!!\n\r");
	return 0xff;
   }
 if(F_WRITE_0X9000)
	{READ_FILE = FILE0X9;
	 if(Read_FlashID() != 0xBFD6 )
	  {
       my_putstr("FLASH_ID Error !!!\n\r");
	   return 0xff;
      } 
	}
 else
   	READ_FILE = FILE0X4;
 fdr = FILE_Open(READ_FILE, RDONLY);	 /*打开指定文件   */
 if(fdr<=0)	
  {
   my_putstr("Could not open file: ");
   my_putstr(READ_FILE);
   my_putstr("\r\n");
   err_code =1; 
   goto err_process; 
  }			   /* 文件不能打开返回(1)          */
 else
  {my_putstr("Open file: ");
   my_putstr(READ_FILE);
   my_putstr("\r\n");
  }
 if(fdr>0)
  {File_Size = Get_Open_FileSise(fdr);                          /*取已打开文件字节数           */            
   if( !File_Size ) 
   { err_code =2; 												/*文件字节数为0返回(2)         */
     my_putstr("FILE SISE IS 0 ");
     my_putstr("\r\n");
     goto err_process; 
	}	
   sprintf((char *)screen_outbuf,"FILE SISE IS:%08d\r\n",File_Size);
   my_putstr(screen_outbuf);
   if( FILE_Read(fdr, file_IObuffer, 512) == 0 )				/*读最大512字节到UserBuffer    */
      {err_code =2; goto err_process; }						/*文件字节数为0返回(2)         */
  
   if( file_IObuffer[15] != 0x0D || file_IObuffer[16] != 0x0A || file_IObuffer[0] != ':' )
      {err_code =3; goto err_process; }					    /*不是hex文件返回(3)           */
  
   for(i=0; i<9; i++)
    {
	  if( file_IObuffer[i] != Head_code[i])
	    {err_code =3; goto err_process; }				    /*文件首行不是扩展地址记录返回(3)*/
	}

  /*file_head_OK: */
   if(F_WRITE_0X9000)
	BASE_START_ADR = 0x90000000;
   else
    BASE_START_ADR = 0x00040000;
   tot_bytes_analysed   = 0;								/*清已处理字节计数器                       */
   BinBuf_bytes_cnt = 0;									/*清二进制待写入缓冲区字节计数器(全程变量) */
   Bin_Lcnt = 0;											/*清二进制行记录缓冲区字节计数器           */
   FLASH_Ext_Adr =  0;								        /*清FLASH当前地址为0                       */
   Clear_BinBuf();											/*清二进制代码缓冲区为空(0xFF)             */
   while(tot_bytes_analysed < File_Size )					/*源文件未处理完一直循环                   */
    {
	  for(i=0;i<512;i++)
	   {tot_bytes_analysed++;
	    switch (file_IObuffer[i])
	     {case ':': Bin_Lcnt=0;
		            H_4Bit = 0xff; 
				    break;  
		  case 0x0D:
				    break; 
		  case 0x0A:										  /*一完整记录结束*/
				    err_code = Line_process(Binlinebuff,Bin_Lcnt); /*一行记录的数字已全部转为二进制并放入Binlinebuff中,进行处理*/
					if(err_code)
					  goto err_process;
					Bin_Lcnt = 0;							  /*清二进制行记录缓冲区字节计数器           */
					if(tot_bytes_analysed >= File_Size)
					 {
					  Bin_Lcnt = FLASH_Current_Adr - BASE_START_ADR;
					  sprintf((char *)screen_outbuf,"BIN SISE IS:%08d\r\n",Bin_Lcnt);
                      my_putstr(screen_outbuf);
					  err_code = 0;
					  goto err_process;
				 	  //return(0);
					 }						      /*正常返回处！！！*/
					break; 
		  default :
		           if(isxdigit(file_IObuffer[i] == 0))
					{err_code =4; goto err_process; }		  /*文件中有非HEX的ASC字符*/
				   if(H_4Bit )
				    {tmpByte = ((asc_to_hex(file_IObuffer[i]))<<4) & 0xf0;	 /*高4位*/
					 H_4Bit = 0;
					}
				   else
				    {H_4Bit = 0xff;
					 tmpByte = tmpByte | asc_to_hex(file_IObuffer[i]);	 	/*高、低4为拼合为8位的HEX*/
				   	 Binlinebuff[Bin_Lcnt++] = tmpByte;					/*装入二进制行记录缓冲区*/
				     if( Bin_Lcnt >= 70 )								/*记录中字节数magic的文件要改为70*/
					  {err_code =5; goto err_process; }					/*记录中的数据字节超出21*/
					}
				   break; 
	    }
	  }
     FILE_Read(fdr, file_IObuffer, 512);			/*读入下一个512字节到UserBuffer*/
    }
 }
 err_process: 
  FILE_Close(fdr);
  /*待增加数码显示板指示*/
  return(err_code);
}

uint8_t outflash_to_inflash(void)			  //OK返值为0
{uint32_t i;
 uint32_t OUT_FLASH_adr= 0X90000000;
 uint32_t IN_FLASH_adr = 0X40000;
 uint32_t *p;
 uint8_t   st = 0;
 my_putstr("Programing( Out_Flash(0x90000000~0x9003ffff)->In_Flash(0x40000~0x7ffff) ):\n\r");
 if(Read_FlashID() != 0xBFD6 )
	  {
       my_putstr("FLASH_ID Error !!!\n\r");
	   return 0x01;
      } 
 p = (uint32_t *)OUT_FLASH_adr;
 if(*p > 0x10000000	  && *p < 0x1000ffff)
   { p++;
     st = 0;
     for(i=0;i<5;i++,p++)
	  {//sprintf((char *)screen_outbuf,"P[%d]:%08x \n\r",i,*p);
       //my_putstr(screen_outbuf); 
	   if( (*p < 0x40000 || *p > 0x7ffff ) || (!BITTST(*p,0)))
		{st =0x02;
		 break;
		}
	  }
	}
 else
    { st = 0x02; }
 if(st) 
   { my_putstr("The BinFile in OUT_FLASH Not Correct ! \r\n");
   	 return(st);
   }
 if(my_IAP_erase(22,29))
  {
   my_putstr("The IN_FLASH Erase Fail ! \r\n");
   return 0x03;
  }
 for(i=0; i <256; i++)
  {//outflash_to_ram( (uint32_t)&BinBuffer[0], OUT_FLASH_adr, 1024 );
   memcpy((uint8_t *)&BinBuffer[0], (uint8_t *)OUT_FLASH_adr, 1024);
   st = my_IAP_Write((uint8_t *)IN_FLASH_adr, (uint8_t *)&BinBuffer[0], 1024 );
   if(st) break;
   OUT_FLASH_adr += 1024;
   IN_FLASH_adr  += 1024;
  }
  if(st) 
   {
    my_putstr("The IN_FLASH Programing Error ! \r\n");
    return 0x04;
   }
 //my_putstr("Programing Success ! \r\n");
 return 0;
}

uint8_t inflash_to_outflash(void)
{uint32_t i;
 uint32_t OUT_FLASH_adr= 0X90000000;
 uint32_t IN_FLASH_adr = 0X40000;
 uint32_t *p; 
 uint8_t   st = 0;
 my_putstr("Programing( In_Flash(0x40000~0x7fff)->Out_Flash(0x90000000~0x9003ffff) ):\n\r");
 if(Read_FlashID() != 0xBFD6 )
	  {
       my_putstr("FLASH_ID Error !!!\n\r");
	   return 0x01;
      } 
 p = (uint32_t *)IN_FLASH_adr;
 if(*p > 0x10000000	  && *p < 0x1000ffff)
   { p++;
     st = 0;
     for(i=0;i<5;i++,p++)
	  {//sprintf((char *)screen_outbuf,"P[%d]:%08x \n\r",i,*p);
       //my_putstr(screen_outbuf); 
	   if( (*p < 0x40000 || *p > 0x7ffff ) || (!BITTST(*p,0)))
		{st = 0x02;
		 break;
		}
	  }
	}
 else
    { st = 0x02; }
 if(st) 
   { my_putstr("The BinFile in INSide_FLASH Not Correct ! \r\n");
   	 return(st);
   } 
 if(my_Chip_Erase())
  {
   my_putstr("The OUT_FLASH Erase Fail ! \r\n");
   return 0x03;
  }
 for(i=0; i <256; i++)
  {//INflash_to_ram( (uint32_t)&BinBuffer[0], IN_FLASH_adr, 1024 );
   memcpy((uint8_t *)&BinBuffer[0], (uint8_t *)IN_FLASH_adr, 1024);
   st = my_Flash_Prg(OUT_FLASH_adr,(uint32_t)&BinBuffer[0], 1024);
   if(st) break;
   OUT_FLASH_adr += 1024;
   IN_FLASH_adr  += 1024;
  }
  if(st) 
   {
    my_putstr("The OUT_FLASH Programing Error ! \r\n");
    return 0x04;
   }
 my_putstr("Programing Success ! \r\n");
 return 0;
}

/*------------------K1安排---------------*	 
  K1_4(D2):ON-ISP;OFF-运行用户程序；
  用户程序中：使用K1_3，K1_1， K1_2来实现程序的选择：
  K1_3(D3)-ON , K1_2(D1)-ON	 ，K1_1(D0)-ON  ：(0)运行片外FLALSH 0X9000 0000处的应用程序；
  K1_3(D3)-ON , K1_2(D1)-ON	 ，K1_1(D0)-OFF ：(1)USB->FLALSH 0X0004 0000处的程序写入
  K1_3(D3)-ON , K1_2(D1)-OFF ，K1_1(D0)-ON  ：(2)USB->FLALSH 0X9000 0000处的程序写入
  K1_3(D3)-ON , K1_2(D1)-OFF ，K1_1(D0)-OFF ：(3)FLALSH 0X9000 0000 -> FLALSH 0X0004 0000处的程序写入
  K1_3(D3)-OFF, K1_2(D1)-ON  ，K1_1(D0)-ON  ：(4)运行片内的CPU测试和DEBUG软件
  K1_3(D3)-OFF, K1_2(D1)-ON  ，K1_1(D0)-OFF ：(5)运行片内槽控机调试软件
  K1_3(D3)-OFF, K1_2(D1)-OFF ，K1_1(D0)-ON  ：(6)保留
  K1_3(D3)-OFF, K1_2(D1)-OFF ，K1_1(D0)-OFF ：{7)运行片内FLALSH 0X0004 0000处的应用程序；
 
 *---------------------------------------*/
extern void PCB_DEBUG(void);


int main(void)
{
 uint8_t   input_ch,k1;
 uint32_t  *prg_start_adr;
main_start:
 GLM1788CPU_Initial(); 		   //目标板初始化
 UART_Init(LPC_UART0, 19200, 0, 3, 0 );  /*波特率：19200，无校验，8bit数据，1停止位*/
 UART_TxCmd(LPC_UART0,  1);				 /*1-允许发送；0-禁止发送*/	
 
 if((LPC_SC->CLKSRCSEL & 0x01) == 0)     /* CLKSRCSEL[0]=0:irc_clk用作sysclk； =1：osc_clk	用作sysclk	  */
  GPIO_PinOut_Out(0, 8, 0);				 /* 如外部晶振有问题，点亮LED6                                    */
 input_ch = GPIO_ByteIn_Read(2,1)&0x0f;
 k1 = input_ch&0x03;
 if(BITTST(input_ch,3))
   BITSET(k1,2);
 //k1 = 4;			//CZNQQQ
 sprintf((char *)screen_outbuf,"  K1 = %d\r\n",k1);
 my_putstr(screen_outbuf);
 switch( k1 )
  {
   case 0: 
        {
		 prg_start_adr =(uint32_t *)0x90000004;
		 if( (*prg_start_adr > 0x90000000 && *prg_start_adr < 0x9003ffff ) && (BITTST(*prg_start_adr,0)))
		   JUMP_TO(*prg_start_adr);
		 goto main_start;
		}
   case 1:
        {my_putstr("\r\n");
		 GPIO_PinOut_Out(0, 9, 0);			   //点亮LED7，表示正在FLASH编程。
		 if(USB_TO_FLASH(0))				   //调用USB_TO_flashs程序，输入参数0表示对片内；非0表示对片外，返值0表示OK。
		   {my_putstr("OnChip_HEXProgramming Error !\r\n");
			for(;;)
			 {GPIO_PinOut_Out(0, 10, 0);
			  DelayMS(100);
			  GPIO_PinOut_Out(0, 10, 1);
			  DelayMS(100);
			 }
		   }
		 else
		   {my_putstr("OnChip_HEXProgramming OK !\r\n");
			GPIO_PinOut_Out(0, 10, 0);			   //点亮(场亮)LED8，表示FLASH编程OK。
		   }
		 while(1);
		}
   case 2:
        {my_putstr("\r\n");
		 GPIO_PinOut_Out(0, 9, 0);			   //点亮LED7，表示正在FLASH编程。
		 if(USB_TO_FLASH(0xff))				   //调用USB_TO_flashs程序，输入参数0表示对片内；非0表示对片外，返值0表示OK。
		   {my_putstr("outChip_HEXProgramming Error !\r\n");
			for(;;)
			 {GPIO_PinOut_Out(0, 10, 0);
			  DelayMS(100);
			  GPIO_PinOut_Out(0, 10, 1);
			  DelayMS(100);
			 }
		   }
		 else
		   {my_putstr("outChip_HEXProgramming OK !\r\n");
			GPIO_PinOut_Out(0, 10, 0);			   //点亮(场亮)LED8，表示FLASH编程OK。
		   }
		 while(1);
		}
   case 3:
        {my_putstr("\r\n");
		 GPIO_PinOut_Out(0, 9, 0);			   //点亮LED7，表示正在FLASH编程。
		 if(outflash_to_inflash( ))		       //调用outflash_to_inflash程序,返值0表示OK。
		 {my_putstr("OnChip_BINProgramming Error !\r\n");
			for(;;)
			 {GPIO_PinOut_Out(0, 10, 0);
			  DelayMS(100);
			  GPIO_PinOut_Out(0, 10, 1);
			  DelayMS(100);
			 }
		   }
		 else
		   {my_putstr("OnChip_BINProgramming OK !\r\n");
			GPIO_PinOut_Out(0, 10, 0);			   //点亮(场亮)LED8，表示FLASH编程OK。
		   }
		 while(1);
		}
  case 4: 
        {
		 SFtest_and_CMDexplain();
		}
   case 5:
        {
		 PCB_DEBUG();
		}
   case 7:
        {
		 prg_start_adr =(uint32_t *)0x40004;
		 if( (*prg_start_adr > 0x40000 && *prg_start_adr < 0x7ffff ) && (BITTST(*prg_start_adr,0)))
		   JUMP_TO(*prg_start_adr);
		 goto main_start;
		}
   default: goto main_start;
  }
}
/*88888888888888888888888888888888888888888888888888888888888888888888888888888*/




											 
