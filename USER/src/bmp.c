#include "stm32f0xx.h"
#include <stdint.h>
#include <stdbool.h>

#include <math.h>

//SCL -> PC1
//SDA -> PC2

#define SCL_H()  GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define SCL_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_5)
#define SDA_H()  GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define SDA_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define SDA  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)

#define BMP180_SlaveAddr 0xee   //BMP180��������ַ

typedef uint8_t u8;

//BMP180У׼ϵ��

short AC1;
short AC2;
short AC3;
unsigned short AC4;
unsigned short AC5;
unsigned short AC6;
short B1;
short B2;
short MB;
short MC;
short MD;

 
uint8_t BMP180_ID=0;          //BMP180��ID
float True_Temp=0;       //ʵ���¶�,��λ:��
float True_Press=0;      //ʵ����ѹ,��λ:Pa
float True_Altitude=0;   //ʵ�ʸ߶�,��λ:m

/*�ⲿоƬIIC���ų�ʼ��
037
 *SCL:PC1
038
 *SDA:PC2
039
*/
void delay_us(int j)
{
	volatile int i;
	for (j = 0; j < 5; j++) 
		for(i= 0; i < 16; i++);
}
void DelayMs(uint32_t nTime);
void delay_ms(int j)
{
	DelayMs(j);
}
void IIC_PortInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;  //����һ��GPIO_InitTypeDef���͵Ľṹ��
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_5|GPIO_Pin_6);    //����
}

void IIC_Init(void)
{
    SCL_H();  //SCL = 1;
    delay_us(5);
    SDA_H();  //SDA = 1;
    delay_us(5);   
}

void IIC_Start(void)
{
    SDA_H();  //SDA = 1;
    delay_us(5);
    SCL_H();  //SCL = 1;
    delay_us(5);
    SDA_L();  //SDA = 0;
    delay_us(5);   
}

void IIC_Stop(void)
{
    SDA_L();   //SDA = 0;
    delay_us(5);
    SCL_H();   //SCL = 1;
    delay_us(5);
    SDA_H();   //SDA = 1;
    delay_us(5);
}

unsigned char IIC_ReceiveACK(void)
{
    unsigned char ACK;

    SDA_H();     //SDA=1;//Ҫ���͵�ƽ���������ٶ�,����������Ǵ�������,����Ҫ��
    SCL_H();     //SCL=1;
    delay_us(5);

    if (SDA==1)  //SDAΪ��
    {
        ACK = 1;   
    }
    else ACK = 0;  //SDAΪ��
    SCL_L();    //SCL = 0;//SCLΪ�͵�ƽʱSDA�ϵ����ݲ�����仯,Ϊ������һ���ֽ���׼��
    delay_us(5);

    return ACK;                
}

void IIC_SendACK(unsigned char ack)
{
    if (ack == 1)SDA_H();
    else if (ack == 0)SDA_L();

    //SDA = ack;

    SCL_H();   //SCL = 1;
    delay_us(5);
    SCL_L();   //SCL = 0;
    delay_us(5);
}

unsigned char IIC_SendByte(unsigned char dat)
{
    unsigned char i;
    unsigned char bResult=1;

    SCL_L();     //SCL = 0;//����ʱ����
    delay_us(5);       

    for( i=0;i<8;i++ ) //һ��SCK,��datһλһλ�����͵�SDA��
    {
        if( (dat<<i)&0x80 )SDA_H();   //SDA = 1;//�ȷ���λ
        else SDA_L();  //SDA = 0;
        delay_us(5);

        SCL_H();  //SCL = 1;
        delay_us(5);
        SCL_L();  //SCL = 0;
        delay_us(5);

    }
    bResult=IIC_ReceiveACK(); //������һ���ֽڵ�����,�ȴ�����Ӧ���ź�
    return bResult;  //����Ӧ���ź�

}

unsigned char IIC_ReadByte(void)
{
    unsigned char dat;
    unsigned char i;


    SCL_H();     //SCL = 1;//ʼ��������Ϊ��������׼��
    delay_us(5);

    for( i=0;i<8;i++ )
    {
        dat <<= 1;
        dat = dat | (SDA);
        delay_us(5);
        SCL_L();   //SCL = 0;
        delay_us(5);   
        SCL_H();   //SCL = 1;
        delay_us(5);   

    }
    return dat;
}

/*��BMP180�ж�1���ֽڵ�����*/

u8 BMP180_ReadOneByte(u8 ReadAddr)
{
    u8 temp = 0;
    u8 IIC_ComFlag = 1;   //IICͨ�ű�־,Ϊ0��־����,1��ʾͨ�Ŵ���

    IIC_Start();     //IIC start
    IIC_ComFlag = IIC_SendByte(BMP180_SlaveAddr);   //slave address+W:0
    //printf("IIC_ComFlag=%u \r\n",IIC_ComFlag);

    if (IIC_ComFlag == 0)                           //����ֵΪ0��ʾͨ������,���Լ���ͨ�š�����ͨ����
    {
        IIC_SendByte(ReadAddr);                      //�����������
        IIC_Start();
        IIC_SendByte(BMP180_SlaveAddr|0x01);         //slave address+R:1
        temp = IIC_ReadByte();                       //������
        IIC_SendACK(1);                
        IIC_Stop();

    }
    return (temp);     
}

/*��BMP180�ж�2���ֽڵ�����*/

short BMP180_ReadTwoByte(u8 ReadAddr)
{
    u8 IIC_ComFlag = 1;   //IICͨ�ű�־,Ϊ0��־����,1��ʾͨ�Ŵ���
    u8 MSB,LSB;
    short temp;

     
    IIC_Start();
    IIC_ComFlag = IIC_SendByte(BMP180_SlaveAddr);
    if (IIC_ComFlag == 0)
    {
        IIC_SendByte(ReadAddr);
        IIC_Start();
        IIC_SendByte(BMP180_SlaveAddr|0x01);
        MSB = IIC_ReadByte();       //�ȶ���λ
        IIC_SendACK(0);         //ACK
      //  LSB = IIC_ReadByte();      //�ٶ���λ
       // IIC_SendACK(1);        //NACK
        IIC_Stop();
    }
		IIC_Start();
    IIC_ComFlag = IIC_SendByte(BMP180_SlaveAddr);
    if (IIC_ComFlag == 0)
    {
        IIC_SendByte(ReadAddr+1);
        IIC_Start();
        IIC_SendByte(BMP180_SlaveAddr|0x01);
        LSB = IIC_ReadByte();       //�ȶ���λ
        IIC_SendACK(0);         //ACK
      //  LSB = IIC_ReadByte();      //�ٶ���λ
       // IIC_SendACK(1);        //NACK
        IIC_Stop();
    }
    temp = MSB*256+LSB;
    return temp;                                                   
}
 
/*��BMP180�ļĴ���дһ���ֽڵ�����*/
void Write_OneByteToBMP180(u8 RegAdd, u8 Data)
{
    IIC_Start();                       //IIC start
    IIC_SendByte(BMP180_SlaveAddr);   //slave address+W:0
    IIC_SendByte(RegAdd);
    IIC_SendByte(Data);
    IIC_Stop();
}

/*��ȡBMP180��У׼ϵ��*/

void Read_CalibrationData(void)
{
    AC1 = BMP180_ReadTwoByte(0xaa);
    AC2 = BMP180_ReadTwoByte(0xac);
    AC3 = BMP180_ReadTwoByte(0xae);
    AC4 = BMP180_ReadTwoByte(0xb0);
    AC5 = BMP180_ReadTwoByte(0xb2);
    AC6 = BMP180_ReadTwoByte(0xb4);
    B1 = BMP180_ReadTwoByte(0xb6);
    B2 = BMP180_ReadTwoByte(0xb8);
    MB = BMP180_ReadTwoByte(0xba);
    MC = BMP180_ReadTwoByte(0xbc);
    MD = BMP180_ReadTwoByte(0xbe); 
}

/*��BMP180û�о����������¶�ֵ*/
long Get_BMP180UT(void)
{

    long UT;

    Write_OneByteToBMP180(0xf4,0x2e);       //write 0x2E into reg 0xf4
    delay_ms(10);                                   //wait 4.5ms
    UT = BMP180_ReadTwoByte(0xf6);          //read reg 0xF6(MSB),0xF7(LSB)
 //   printf("UT:%ld \r\n",UT);

    return UT;
}

/*��BMP180û�о���������ѹ��ֵ*/

long Get_BMP180UP(void)
{

    long UP=0;
    Write_OneByteToBMP180(0xf4,0x34);       //write 0x34 into reg 0xf4
    delay_ms(10);                                    //wait 4.5ms
    UP = BMP180_ReadTwoByte(0xf6);
    UP &= 0x0000FFFF;
  //  printf("UP:%ld \r\n",UP);

    return UP;     
}

/*��δ�����������¶Ⱥ�ѹ��ֵת��Ϊʱ����¶Ⱥ�ѹ��ֵ
272
 *True_Temp:ʵ���¶�ֵ,��λ:��
273
 *True_Press:ʱ��ѹ��ֵ,��λ:Pa
274
 *True_Altitude:ʵ�ʺ��θ߶�,��λ:m
275
*/

void Convert_UncompensatedToTrue(long UT,long UP)
{

    long X1,X2,X3,B3,B5,B6,B7,T,P;
    unsigned long B4;

    X1 = ((UT-AC6)*AC5)>>15;      //printf("X1:%ld \r\n",X1);
    X2 = ((long)MC<<11)/(X1+MD);  //printf("X2:%ld \r\n",X2);
    B5 = X1+X2;                        //printf("B5:%ld \r\n",B5);
    T = (B5+8)>>4;                      //printf("T:%ld \r\n",T);
    True_Temp = T/10.0;            
//	printf("Temperature:%.1f \r\n",True_Temp);
    B6 = B5-4000;                       //printf("B6:%ld \r\n",B6);
    X1 = (B2*B6*B6)>>23;              //printf("X1:%ld \r\n",X1);
    X2 = (AC2*B6)>>11;                //printf("X2:%ld \r\n",X2);
    X3 = X1+X2;                         //printf("X3:%ld \r\n",X3);
    B3 = (((long)AC1*4+X3)+2)/4;    //printf("B3:%ld \r\n",B3);
    X1 = (AC3*B6)>>13;                //printf("X1:%ld \r\n",X1);
    X2 = (B1*(B6*B6>>12))>>16;      //printf("X2:%ld \r\n",X2);
    X3 = ((X1+X2)+2)>>2;              //printf("X3:%ld \r\n",X3);
    B4 = AC4*(unsigned long)(X3+32768)>>15;   //printf("B4:%lu \r\n",B4);
    B7 = ((unsigned long)UP-B3)*50000;        //printf("B7:%lu \r\n",B7);
    if (B7 < 0x80000000)
    {
        P = (B7*2)/B4; 
    }
    else P=(B7/B4)*2;                   //printf("P:%ld \r\n",P);          
    X1 = (P/256.0)*(P/256.0);       //printf("X1:%ld \r\n",X1);
    X1 = (X1*3038)>>16;               //printf("X1:%ld \r\n",X1);
    X2 = (-7357*P)>>16;               //printf("X2:%ld \r\n",X2);
    P = P+((X1+X2+3791)>>4);      //printf("P:%ld \r\n",P);
    True_Press = P;                 
//	printf("Press:%.1fPa \r\n",True_Press);
    True_Altitude = 44330*(1-pow((P/101325.0),(1.0/5.255)));           
  //  printf("Altitude:%.3fm \r\n",True_Altitude);   
}

int test_bmp180()
{

    long UT,UP;
	
	  IIC_PortInit();
    Read_CalibrationData();         //��ȡBMP180��У׼ϵ��

    while(1)
    {            

		 BMP180_ID = BMP180_ReadOneByte(0xd0);      //��ȡID��ַ
		// printf("BMP180_ID:0x%x \r\n",BMP180_ID);
		 UT = Get_BMP180UT();          
		 UP = Get_BMP180UP();                               
		 Convert_UncompensatedToTrue(UT,UP);            
		 delay_ms(1000);
	}
}

 
