/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include <stdlib.h>

__IO uint32_t uwTick = 0;
uint32_t tickstart = 0 ;

#define PID_INCREMENTAL 
//#define PID_ANTI_WINDUP

//#define PID_LOG_APPROACH_PROCESS
#define PID_LOG_RESULT_SPEND_TIME

#define USE_RANDOM_SEED_GENERATE_DATA
#define USE_TIMER_CALCULATE_TIMING

struct t_pid
{
	float SetSpeed;
	float ActualSpeed;
	float err;
	float err_last;	
	float Kp,Ki,Kd;
	
	#if defined (PID_INCREMENTAL)
	float err_next;
	#endif

	#if defined (PID_ANTI_WINDUP)
	float voltage;
	float integral;
	float umax;
	float umin;
	#endif
}pid;


void IncTick_1msIRQ(void)	// 1 ms timer interrupt
{
	uwTick++;
}

uint32_t GetTick(void)
{
	return uwTick;
}


void PID_init(float Kp , float Ki , float Kd)
{
    pid.SetSpeed = 0.0;
    pid.ActualSpeed = 0.0;
    pid.err = 0.0;
    pid.err_last = 0.0;
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;	

	#if defined (PID_INCREMENTAL)	
	pid.err_next = 0.0;
	#endif 

	#if defined (PID_ANTI_WINDUP)	
	pid.voltage = 0.0;
	pid.integral = 0.0;
	pid.umax = 400;
	pid.umin =-200; 
	#endif 
}

float PID_realize(float speed)
{
	float incrementSpeed;

	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;

	#if defined (PID_INCREMENTAL)

    incrementSpeed = pid.Kp*(pid.err-pid.err_next) + pid.Ki*pid.err + pid.Kd*(pid.err - 2*pid.err_next + pid.err_last);
    pid.ActualSpeed += incrementSpeed;
    pid.err_last = pid.err_next;
    pid.err_next = pid.err;
	#endif

	#if defined (PID_ANTI_WINDUP)
	int index;	
	if (pid.ActualSpeed>pid.umax)
	{
		if(abs(pid.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pid.err<0)
			{
				pid.integral+=pid.err;
			}
		}
	}
	else if (pid.ActualSpeed<pid.umin)
	{
		if(abs(pid.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			if(pid.err>0)
			{
				pid.integral+=pid.err;
			}
		}
	}
	else
	{
		if(abs(pid.err)>200)
		{
			index=0;
		}
		else
		{
			index=1;
			pid.integral+=pid.err;
		}
	}
	
	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	pid.err_last=pid.err;
	pid.ActualSpeed=pid.voltage*1.0;
	#endif
	
	return pid.ActualSpeed;
}

void PID_calculate(float target)
{

    int cnt = 0;
  	__IO int speed = 0;

	static uint16_t current = 0;
	static uint16_t previous = 0;

	uint16_t convergence = 0;

	if (previous == (uint16_t)target)
	{
//		printf("target same last data , skip flow\r\n");
		return;
	}

	current = (uint16_t)target;
	convergence = current % 10;
	
	while(1)
    {
		speed = (uint16_t) PID_realize(target);
//		printf("speed : %3d (target : %3d)\n",(int)speed,(int)target);

		if (((current - previous) <= convergence) || ((previous - current) <= convergence))
		{
			#if defined (PID_LOG_APPROACH_PROCESS)
			printf("cnt : %3d , speed : %3d (target : %3d)\r\n" ,cnt,(uint16_t)speed,(uint16_t)target);
			#endif
			cnt++;
		}

		if (((speed+1) == current) || (speed == current))
		{   			
			#if defined (PID_LOG_RESULT_SPEND_TIME)
//			printf("cnt : %3d , speed : %3d (target : %3d)\r\n" ,cnt,(uint16_t)speed,(uint16_t)target);

			#if defined (USE_TIMER_CALCULATE_TIMING)
			printf("cnt : %3d , time : %5d ms , speed : %5d (target : %5d)\r\n" ,cnt,GetTick() - tickstart,(uint16_t)speed,(uint16_t)target);
			#endif
				
			#endif
			cnt = 0;				
			break;
		}		
    }

	previous = (int)target;
	
}

void TMR3_IRQHandler(void)
{
	static uint32_t LOG = 0;
	static uint16_t CNT = 0;
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);

		#if defined (USE_TIMER_CALCULATE_TIMING)
		IncTick_1msIRQ();
		#endif
		
		if (CNT++ >= 1000)
		{		
			CNT = 0;
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
		}
		
    }
}


void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}


void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
//	UART_SetTimeoutCnt(UART0, 20);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	
    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);
	
    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
  	uint16_t data = 0;	

    SYS_Init();

    UART0_Init();

	TIMER3_Init();

	#if defined (USE_RANDOM_SEED_GENERATE_DATA)
	srand(0x1234);	//seed
	#endif

	PID_init(0.2 , 0.015 , 0.2);
	
    /* Got no where to go, just loop forever */
    while(1)
    {

		#if defined (USE_TIMER_CALCULATE_TIMING)
		// use timer to calculate each PID approach process spend time
		tickstart = GetTick();
		#endif

		#if defined (USE_RANDOM_SEED_GENERATE_DATA)
		data = rand()%(0xFFFF-1) + 1;	//0xFF	
		#endif

		// send random data to use PID flow , to approach to data 
		PID_calculate(data);
    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
