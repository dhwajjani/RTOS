// RTOS Framework - Spring 2018

// DHWAJ VANDANKUMAR JANI
//CCS version 7 stack size 512
// 14_rtos.c   Single-file with your project code

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include<stdlib.h>
#include <strings.h>
#include "tm4c123gh6pm.h"

#define RED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))//Pin 7-Port A
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 +  (0x400043FC-0x40000000)*32 + 6*4)))//Pin 6-Port A
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 +  (0x400043FC-0x40000000)*32 + 5*4)))//Pin 5-Port A
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 +  (0x400243FC-0x40000000)*32 + 3*4)))//Pin 3-Port E
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define PB1   (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))//Pin 4-Port F
#define PB2   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))//Pin 4-Port C
#define PB3   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))//Pin 5-Port C
#define PB4   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))//Pin 6-Port C
#define PB5   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))//Pin 7-Port C
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
#define Max_Char      70
struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
  char name[15];
  uint8_t running_task;
  bool status;
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;
uint32_t system_stack;
uint16_t filter_timeout=1000;
uint8_t field=0,position[Max_Char],valid,typo=0;
char str[Max_Char+1],print[100];
bool FirstUpdate=1,PI=0,preemptive=0,CreateThread_ok=0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer
#define SEMAPHORE_INVALID 0 //semaphore is not valid(not created)
#define SEMAPHORE_VALID   1//semaphore is valid(created)

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t Priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // 0=highest, 7=lowest
  uint8_t Skip_Count;            // used for Priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
  uint32_t time;                 //total accumulated time in 1sec(every systick)
  uint32_t iir_out;               // Output of IIR filter
  uint32_t start_time;            // Time at start of the task
  uint32_t end_time;              // Time at end of the task
  uint16_t cpu;                   //cpu percentage usage
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread


//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
    tcb[i].semaphore=0;
  }
  // clear out semaphores records
  for (i = 0; i < MAX_SEMAPHORES; i++)
   {
	 semaphores[i].status=SEMAPHORE_INVALID;
   }
  // initialize systick for 1ms system timer
  NVIC_ST_RELOAD_R = 0x9C40;
  NVIC_ST_CURRENT_R = 0x9C40;        //Configure Systick Timer at 1KHZ rate(1 milli Second)
}

void rtosStart()
{
  _fn fn;
  taskCurrent = rtosScheduler();
  //initialize the SP with tcb[task_current].sp;
 system_stack = get_sp();
 set_sp(tcb[taskCurrent].sp);
  fn = tcb[taskCurrent].pid;
  NVIC_ST_CTRL_R |= 0x07;    //Enable SysTick Timer before running first task
  tcb[taskCurrent].start_time =  NVIC_ST_CURRENT_R;
  (*fn)();
}

bool createThread(_fn fn, char name[], int Priority)
{
__asm(" SVC #06");
//allows tasks switches again
  return CreateThread_ok;
}

// Destroy a thread & removes any pending semaphore waiting
void destroyThread(_fn fn)
{
	__asm(" SVC #05");
}

void setThreadPriority(_fn fn, uint8_t Priority)
{
	uint8_t i;
	 for (i = 0; i < MAX_TASKS; i++)
		{
		 if(tcb[i].pid == fn)
		 {
			 tcb[i].Priority=Priority;
			 tcb[i].currentPriority=Priority;
			 break;
		 }
		}
}

struct semaphore* createSemaphore(uint8_t count,char name[])
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
    pSemaphore->queueSize = 0;
    pSemaphore->status=SEMAPHORE_VALID;
    strcpy(pSemaphore->name,name);
  }
  return pSemaphore;
}
v
void yield()
{
  // push registers, call scheduler, pop registers, return to new function
	__asm(" SVC #01");
}

// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
  // push registers, set state to delayed, store timeout, call scheduler, pop registers,
  // return to new function (separate unrun or ready processing)
     __asm(" SVC #02");
}

// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
	 __asm(" SVC #03");
}

// signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #04");
}

// prioritization to 8 levels
int rtosScheduler()
{
	 bool ok;
	  static uint8_t task = 0xFF;
	  ok = false;
	  while (!ok)
	  {
	    task++;
	    if (task >= MAX_TASKS)
	      task = 0;
	    if(tcb[task].Skip_Count == 0)
	    {
	    ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
	    tcb[task].Skip_Count = tcb[task].currentPriority;
	    }
	    else
	    tcb[task].Skip_Count--;
	  }
	  return task;
}

void systickIsr()
{
  uint8_t i;
 tcb[taskCurrent].time += tcb[taskCurrent].start_time;  //Add time for currently interrupted task (with endtime = 0)
  for (i = 0; i < MAX_TASKS; i++)
  {
	  if(tcb[i].state == STATE_DELAYED)
	  {
		  tcb[i].ticks--;
		  if(tcb[i].ticks == 0)
			tcb[i].state = STATE_READY;
	  }
	 tcb[i].start_time = 0;
	 tcb[i].end_time = 0;
  }
	  if(filter_timeout > 0)
	  {
		  filter_timeout--;
	  }
	  if(filter_timeout==0)
	  {
	   for (i = 0; i < MAX_TASKS; i++)
	    {
		   if(FirstUpdate)
		   {
			tcb[i].iir_out= tcb[i].time;
			if(i==(MAX_TASKS-1))
				FirstUpdate=0;
		   }
			else
		   {
	        tcb[i].iir_out = ((tcb[i].iir_out) * 0.9)+ ((tcb[i].time)*0.1);  //Alpha=0.9 for 10 samples
		   }
	        tcb[i].time = 0;
	   }
	   filter_timeout = 1000;
	  }
  tcb[taskCurrent].start_time =  NVIC_ST_CURRENT_R;   //record new start_time for current task after systick interrupt occurs
  if(preemptive==1)
  {
  if(tcb[taskCurrent].state == STATE_UNRUN)
	  tcb[taskCurrent].state= STATE_READY;                //If a task has been preempted in unrun state then make it ready to avoid stack overflow in pendsv
  NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
  }
}

void pendSvIsr()
{
	__asm(" PUSH {R4 ,R5 ,R6 ,R7 ,R8 ,R9 ,R10 ,R11}");
	__asm(" MOV   R10,LR");                                 //move LR value(FFFFFFF9) to R10
	tcb[taskCurrent].sp= get_sp();
	tcb[taskCurrent].end_time=  NVIC_ST_CURRENT_R;
	if((tcb[taskCurrent].start_time) > tcb[taskCurrent].end_time)
	{
	tcb[taskCurrent].time += tcb[taskCurrent].start_time - tcb[taskCurrent].end_time;
	tcb[taskCurrent].start_time=0;
	}
	set_sp(system_stack);
	taskCurrent=rtosScheduler();
	 if((NVIC_INT_CTRL_R & 0x4000000)==0)     //check whether systick exception(whether over flow has occured) is pending, if not then record starttime for the current task
	tcb[taskCurrent].start_time =  NVIC_ST_CURRENT_R;
	if(tcb[taskCurrent].state == STATE_READY)
	 {
		set_sp(tcb[taskCurrent].sp);
	 __asm(" POP {R4 ,R5 ,R6 ,R7 ,R8 ,R9 ,R10 ,R11}");
	 }
	if(tcb[taskCurrent].state == STATE_UNRUN)
	 {
	    set_sp(tcb[taskCurrent].sp);
	    get_pid();
	    __asm(" MOV    R11,R0");                               //move PID to R11 to push it as PC on stack
	    __asm(" MOV    R7,LR");
	    __asm(" MOV    R12,#0x01000000");                     //Move value to be pushed on stack for XPSR in R12
	    __asm(" PUSH { R12, R11, R7, R4, R3, R2, R1, R0}");    //Push XPSR, PC, LR, R12, R3-R0
	    __asm(" PUSH {R0,R1,R2,R3,R4,R10}");                   //Push garbage values fror R4-R7 & FFFFFFF9 for LR (all pushed by compiler at beginning of ISR)
     }
}


void svCallIsr()
{
 uint8_t SVC_number,i,k,m,Priority;
 char name[20];
 uint32_t tick,R0,R1,R2,R3,R12;
 bool found;
 R0=get_Reg(0);
 R1=get_Reg(1);
 R2=get_Reg(2);
 R3=get_Reg(3);
 SVC_number=getSVC_number();
 struct semaphore *pSemaphore;
 _fn fn;
switch(SVC_number)
{
       //---------------Yield------------------------------------------------------------------------------
case 1:tcb[taskCurrent].state= STATE_READY;
       NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
       break;
      //----------------Sleep------------------------------------------------------------------------------
case 2:tick=R0;
	   tcb[taskCurrent].ticks = tick;
       tcb[taskCurrent].state= STATE_DELAYED;
       NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
       break;
       //--------------- Wait-------------------------------------------------------------------------------
case 3: pSemaphore=R0;
	    tcb[taskCurrent].semaphore = pSemaphore;
       if( (pSemaphore->count) > 0 )
        {
         tcb[taskCurrent].state= STATE_READY;
         pSemaphore->running_task = taskCurrent;
         pSemaphore->count--;
        }
       else
        {
	      if(PI==true)
	       {
	    	i=tcb[taskCurrent].currentPriority;
            k=tcb[pSemaphore->running_task].currentPriority;
            if(i<k)
            {
		    tcb[pSemaphore->running_task].currentPriority = tcb[taskCurrent].currentPriority;
	        tcb[pSemaphore->running_task].Skip_Count=tcb[taskCurrent].Skip_Count;
            }
	       }
	     pSemaphore->processQueue[pSemaphore->queueSize] = taskCurrent;
	     pSemaphore->queueSize++;
	     tcb[taskCurrent].state = STATE_BLOCKED;
	     if(pSemaphore->running_task==taskCurrent)
	     pSemaphore->running_task = 10;
	     NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
        break;
       //---------------------Post---------------------------------------------------------------------------------
case 4:pSemaphore=R0;
	   pSemaphore->count++;
       tcb[taskCurrent].currentPriority = tcb[taskCurrent].Priority;
       if(pSemaphore->queueSize > 0) //(tcb[i].state == STATE_BLOCKED) && (tcb[i].semaphore == pSemaphore)
        {
	     tcb[pSemaphore->processQueue[0]].state = STATE_READY;
	     pSemaphore->running_task = pSemaphore->processQueue[0];
	     pSemaphore->count--;
	    for(i=1; i<pSemaphore->queueSize; i++)
	   	{
	      pSemaphore->processQueue[i-1] = pSemaphore->processQueue[i];    //shift Queueupwards
	   	}
	    pSemaphore->queueSize--;
	    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
        break;
        //--------------------------------DeleteThread-----------------------------------------------------------
case 5: fn=R0;
	    found=0;
       for(i=0; i< MAX_TASKS; i++)
       {
        if(tcb[i].pid==fn)
        {
	     found=1;
	     if(strcasecmp(tcb[i].name,"Idle")==0)
	      {
		   putsUart0("\n\rIDLE is Immortal!\r\n");
	      }
	    else
	      {
	       tcb[i].state = STATE_INVALID;
	       taskCount--;
	       pSemaphore=tcb[i].semaphore;
	       if(pSemaphore!=0)
	        {
              for(k=0;k<pSemaphore->queueSize;k++)
               {
                if(i==pSemaphore->processQueue[k])
                {
        	     for(m=k+1;m<pSemaphore->queueSize; m++)
        	     {
        		  pSemaphore->processQueue[m-1] = pSemaphore->processQueue[m];   //remove task from Queue & shift upwards
        	     }
        	   pSemaphore->queueSize--;
        	   break;
                }
               }
	       }
          }
	      break;
         }
        }
        if(found==0)
        {typo=1;}
        else
        {
	     for(i=0; i< MAX_TASKS; i++)
         {
		  tcb[i].cpu=0;
         }
	     FirstUpdate=1;
         }
         NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
         break;
       //------------------------------CreateThread---------------------------------------------------------------------
case 6: fn=R0;
        strcpy(name,R1);
        Priority=R2;
		i = 0;
       found = false;
        if (taskCount < MAX_TASKS)
       {
  // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS) && (tcb[i].state!=STATE_INVALID))
        {
          found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
   // find first available tcb record
         i = 0;
        while (tcb[i].state != STATE_INVALID) {i++;}
        tcb[i].state = STATE_UNRUN;
        tcb[i].pid = fn;
        tcb[i].sp = &stack[i][255];
        tcb[i].Priority = Priority;
        tcb[i].currentPriority = Priority;
        tcb[i].Skip_Count = Priority;
       strcpy(tcb[i].name, name);
    // increment task count
       taskCount++;
       CreateThread_ok = true;
        }
       }
       break;
       //-----------------------------------calc_CpuPercentage-----------------------------------------------------------------
case 7: i=R0;
	    tcb[i].cpu = (tcb[i].iir_out)/4000;  //cpu percentage= ((Bi(IIR output)/40000000)x100%)x100(multiply 100 for bias, so 89.69% will be stored as 8969)
	    break;
	    //----------------------------------------------------------------------------------------------------------------------------
default:
}
}
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
  // initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
	    // Note UART on port A must use APB
	    SYSCTL_GPIOHBCTL_R = 0;

	    // Enable GPIO port B and F peripherals
	    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOF| SYSCTL_RCGC2_GPIOA| SYSCTL_RCGC2_GPIOE;

	    // Configure LED pins
	    GPIO_PORTA_DIR_R |= 0xE0;  // bits 5,6 and 7 are outputs, other pins are inputs
	    GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	    GPIO_PORTA_DEN_R |= 0xE0;  // enable LEDs
	    GPIO_PORTE_DIR_R |= 0x08;  // bit 3 are output, other pins are inputs
	    GPIO_PORTE_DR2R_R |= 0x08; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	    GPIO_PORTE_DEN_R |= 0x08;  // enable LEDs
	    GPIO_PORTF_DIR_R |= 0x04;  //pin 4 as output
	    GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	    GPIO_PORTF_DEN_R |= 0x04;  //enable

	    //Configure Push Buttons
	    GPIO_PORTF_DIR_R &= ~0x10;  // set as input
	    GPIO_PORTF_DEN_R |=  0x10;  // enable Push Button
	    GPIO_PORTF_PUR_R |=  0x10;  // enable internal pull-up for push button
	    GPIO_PORTC_DIR_R &= ~0xF0;  // set as input
	    GPIO_PORTC_DEN_R |=  0xF0;  // enable Push Buttons
	    GPIO_PORTC_PUR_R |=  0xF0;  // enable internal pull-up for push button

	   // Configure UART0 pins(PA0 RX & PA1 TX)
	    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;       // turn-on UART0, leave other uarts in same status
	    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
	    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	   // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
	    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
	    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Returns a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
  uint8_t buttons=0;
  if(PB1==0)
    buttons+=1;
  if(PB2==0)
    buttons+=2;
  if(PB3==0)
    buttons+=4;
  if(PB4==0)
     buttons+=8;
  if(PB5==0)
     buttons+=16;
  return buttons;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}
void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}

void oneshot()
{
  while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash4Hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
	}
    if ((buttons & 16) != 0)
    {
      setThreadPriority(lengthyFn, 4);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}

void important()
{
	while(true)
	{
      wait(resource);
      BLUE_LED = 1;
      sleep(1000);
      BLUE_LED = 0;
      post(resource);
	}
}
//------------------------------------------------------------------------------------------------------------------------------------------------

void shell()
{
  while (true)
  {
	      putsUart0("\n\n\r");
	      //****************************************************************************************
	      uint8_t count=0;
	      char x;
	     while(1)
	      {
	    	 x= getcUart0();
	    	 if(x==8)                                 //checks for Backspace
	    	 {
	    		 if(count>0)
	    	   count=count-1;
	    	 }
	    	 else
	    	 {
	    		 if(x==13)                             //checks for enter key
	    		 {
	    			 str[count++]= '\0';
	    			 break;
	    		 }
	    		 else
	    		 {
	    			 if(x>=32)
	    			 {
	    				 str[count]=x;
	    				 count=count+1;
	    			 }
	    			 if(count>=Max_Char)
	    			 {
	    				 str[count++]='\0';
	    				 break;
	    			 }
	    		 }
	    	 }
	      }
	      //**************************************************************************************
	     char type[30];
	      uint8_t g,k=0,str_length=strlen(str);
	      uint32_t number;
	      typo=0;
	      field = 0;
	      for(g=0;g<str_length;g++)
	      {
	    	if((str[g]>=65 && str[g]<=90) || (str[g]>=97 && str[g]<=122) )
	    	{
	    		if(g==0 || str[g-1]=='\0')
	    		{
	    			type[field]='a';
	    			field=field+1;
	    		    position[k++]=g;
	    		}
	    	}
	    	else if(str[g]>=48 && str[g]<=57)
	    	{
	    		if(g==0 || str[g-1]=='\0')
	    		 {
	    		    type[field]='n';
	    			field=field+1;
	    			position[k++]=g;
	    		 }
	    	}
	    	else if(str[g]=='&')
	    	{
	    	  if(field==1 && str[g-1]=='\0')
	    	  {
	    		  type[field]='s';
	    		  field=field+1;
	    	      position[k++]=g;
	    	  }
	    	}
	    	else
	    	{
	    		str[g]='\0';
	    	}
	    }

	      //**************************************************************************************
	      valid=0;
	      uint8_t i,j,cpu,cpu_fraction,Kernel_int=99,Kernel_frac=99;
	      uint16_t total_cpu=0;
	      struct semaphore *pSemaphore;
	      bool found=0;
	      char temp[20],status[20];
	           if(Iscommand("PS",0))
	           {
	        	   valid =1;
	        	 putsUart0("\n\r PID    TASK NAME     CPU USAGE    CURRENT PRIORITY\tSTATUS   SEMAPHORE\r\n\n");
	        	   for(i=0; i<MAX_TASKS; i++)
	        	   {
	        		   pSemaphore=tcb[i].semaphore;
	        		  if(tcb[i].state != STATE_INVALID)
	        		  {
	        		  switch(tcb[i].state)
	        		  {
	        		  case 1: strcpy(status,"Unrun  "); break;
	        		  case 2: strcpy(status,"Ready  "); break;
	        		  case 3: strcpy(status,"Blocked   ");
	        		          strcat(status,pSemaphore->name); break;
	        		  case 4: strcpy(status,"Delayed"); break;
	        		  default:strcpy(status,"Invalid");
	        		  }
	        		  calc_CpuPercentage(i);
	        		  total_cpu+=tcb[i].cpu;
                      cpu= tcb[i].cpu/100;                           //extract integer part of cpu %
                      cpu_fraction = (tcb[i].cpu % 100);             //extract Fractional part of cpu %
                      ltoa(tcb[i].pid,print);                        //start of table formatting for display purpose//First Column PID
                      strcat(print,"\r\t");
                      strcat(print,tcb[i].name);                      //Second Column Name of the task
                      ltoa(cpu,temp);
                      strcat(print,"\r\t\t\t");
                      strcat(print,temp);                             //Third Column CPU
                      strcat(print,".");
                      ltoa((cpu_fraction/10),temp);                   //first digit of fraction after decimal point
                      strcat(print,temp);
                      ltoa((cpu_fraction%10),temp);                  //Second digit of fraction after decimal point
                      strcat(print,temp);                            //Third Column CPU Fraction part
                      strcat(print,"%\r\t\t\t\t\t   ");
                      ltoa(tcb[i].currentPriority,temp);
                      strcat(print,temp);                            //Fourth Column Current Priority of task
                      strcat(print,"\r\t\t\t\t\t\t\t");
                      strcat(print,status);                         //Fifth Column status of task & sixth colum name of semaphore if blocked
                      strcat(print,"\r\n\n");
                      putsUart0(print);
	        		  }
	        	   }
	        	   Kernel_int=99-(total_cpu/100);               //calculate integer part of Kernel percentage
	        	   Kernel_frac=99-(total_cpu%100);              //calculate fraction part of Kernel percentage
	        	   strcpy(print,"\r\tSYS_KERNEL\t");
	        	   ltoa(Kernel_int,temp);
	        	   strcat(print,temp);
	        	   strcat(print,".");
	        	   ltoa((Kernel_frac/10),temp);                 //print first digit after decimal point
	        	   strcat(print,temp);
	        	   ltoa((Kernel_frac%10),temp);                 //print second digit after decimal point
	        	   strcat(print,temp);
	        	   strcat(print,"%\r\n\n");
	        	   putsUart0(print);
	           }
	     else if(Iscommand("PIDOF",1 ))
	          	{
	    	 for(i=0; i<MAX_TASKS; i++)
	    		 {
	    	      if( strcasecmp(getstring(1),tcb[i].name)==0)
	    	      {
	    	    	  ltoa((tcb[i].pid),print);
	    	    	  putsUart0("\n\r");
	    	    	  putsUart0(print);
	    	    	  valid=1;
	    	    	  break;
	    	      }
	          	 }
	    	      if(valid==0)
	    	      {
	    	    	putsUart0("\n\rEnter Vaild Task Name\r\n");
	    	      }
	    	      valid=1;
	          	}
	     else if(Iscommand("KILL",1 ))
	     	    {
	    	     valid=1;
	    	     number=getnumber(1);
	    	    if(typo!=1)
	    	    {
	    	    destroyThread(number);
	    	    }
	    	    if(typo==1)
	    	    {
	    	    	putsUart0("\n\rerror please enter Valid PID\r\n");
	    	    	typo=0;
	    	    }
	     	    }
	      else if(Iscommand("REBOOT",0 ))
	            {
	    	     NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
	            }
	      else if(Iscommand("PI",1 ))
	            {
	    	      valid=1;
	    	     if(strcasecmp(getstring(1),"ON")==0)
	    	           	  {
	    	           		 PI=true;
	    	           		 putsUart0("\n\rPI Enabled\r\n");
	    	           	  }

	    	    else if(strcasecmp(getstring(1),"OFF")==0)
	    	           	  {
	    	           		  PI=false;
	    	           		 putsUart0("\n\rPI Disabled\r\n");
	    	           	  }

	            }
	       else if(Iscommand("PREEMPTIVE",1 ))
	            {
	    	      valid=1;
	    	     if(strcasecmp(getstring(1),"ON")==0)
	    	           	  {
	    	           		 preemptive=true;
	    	           		 putsUart0("\n\rpreemptive Enabled\r\n");
	    	           	  }

	    	    else if(strcasecmp(getstring(1),"OFF")==0)
	    	           	  {
	    	    	         preemptive=false;
	    	           		 putsUart0("\n\rpreemptive Disabled\r\n");
	    	           	  }

	            }

	      else if(Iscommand("IPCS",0 ))
	            {
	    	        valid=1;
	    	  putsUart0("\n\rSEMAPHORE ID\tNAME\t\tCOUNT \tQUESIZE\t  ACTIVE TASK\t  WAITING LIST\r\n\n");
                  for(i=0; i<MAX_SEMAPHORES; i++)
                  {  if(semaphores[i].status==SEMAPHORE_VALID)
            	      {
                	    ltoa(&semaphores[i],print);
                	    strcat(print,"\r\t\t");
                	    strcat(print,semaphores[i].name);
                	    ltoa((semaphores[i].count),temp);
                	    strcat(print,"\r\t\t\t\t  ");
                	    strcat(print,temp);
                	    ltoa((semaphores[i].queueSize),temp);
                	    strcat(print,"\r\t\t\t\t\t  ");
                	    strcat(print,temp);
                	    if(semaphores[i].running_task != 10)
                	    {
                	    strcat(print,"\r\t\t\t\t\t\t  ");
                	    strcat(print,tcb[semaphores[i].running_task].name);
                	    }
                    	  putsUart0(print);
                	    if((semaphores[i].queueSize)>0)
                	     {
                	      for(j=0;j<semaphores[i].queueSize;j++)
                	        {
                	    	 putsUart0("\r\t\t\t\t\t\t\t\t  ");
                	         putsUart0(tcb[semaphores[i].processQueue[j]].name);
                	         putsUart0("\n\n\r");
                	        }
                	      }
                	    else
                	    {
                	      putsUart0("\n\n\r");
                	    }
            	      }
                  }
	            }
	      else if((field==2) && (strcmp(getstring(1),"&")==0))
	           {
	    	      valid=1;
	    	  for(i=0; i<MAX_TASKS; i++)
	    	  	 {
	    	  	    if(strcasecmp(getstring(0),tcb[i].name)==0)
	    	  	    {
	    	  	    	if(tcb[i].state!=STATE_INVALID)
	    	  	    	{putsUart0("\n\rTask Already Running!\r\n");
	    	  	    	found=1;
	    	  	    	break;}
	    	  	    }
	    	  	 }
	    	        if(!found)
	    	        {
	    	        	if(strcasecmp(getstring(0),"Idle")==0)
	    	  	    	 {createThread(idle, "Idle", 7);}
	    	        	else if(strcasecmp(getstring(0),"Flash4Hz")==0)
	    	  	    	 {createThread(flash4Hz, "Flash4Hz", 0);}
	    	        	else if(strcasecmp(getstring(0),"OneShot")==0)
	    	  	    	 {createThread(oneshot, "OneShot", 4);}
	    	        	else if(strcasecmp(getstring(0),"Shell")==0)
	    	  	    	 {createThread(shell, "Shell", 4);}
	    	        	else if(strcasecmp(getstring(0),"LengthyFn")==0)
	    	  	    	 {createThread(lengthyFn, "LengthyFn", 6);}
	    	        	else if(strcasecmp(getstring(0),"ReadKeys")==0)
	    	  	    	 {createThread(readKeys, "ReadKeys", 6);}
	    	        	else if(strcasecmp(getstring(0),"Debounce")==0)
	    	  	    	 {createThread(debounce, "Debounce", 6);}
	    	        	else if(strcasecmp(getstring(0),"Important")==0)
	    	  	    	 {createThread(important, "Important", 0);}
	    	        	else if(strcasecmp(getstring(0),"Uncoop")==0)
	    	  	    	 {createThread(uncooperative, "Uncoop", 5);}
	    	        	else{putsUart0("\nEnter Correct task name");}
	    	  	    }
	           }
	      if(valid==0 || typo==1)
	      {
	    	  putsUart0("\n\rerror please enter right command\r\n");
	      }
  }
}

void set_sp(uint32_t *stack)
{
   __asm("   mov    R13, R0");
   __asm("   BX LR");
}

uint32_t get_sp()
{
	__asm("   mov    R0, R13");
}
uint32_t get_pid()
{
	return tcb[taskCurrent].pid;
}
uint32_t get_Reg(uint8_t reg)
{
	switch(reg)
	{
	case 0: __asm(" LDR R0, [R13,#80]");   break;   //Get R0 from stack for SVCISR
	case 1: __asm(" LDR R0, [R13,#84]");   break;   //Get R1 from stack for SVCISR
	case 2: __asm(" LDR R0, [R13,#88]");   break;   //Get R2 from stack for SVCISR
	case 3: __asm(" LDR R0, [R13,#92]");   break;   //Get R3 from stack for SVCISR
	case 12:__asm(" LDR R0, [R13,#96]");   break;   //Get R12 from stack for SVCISR
	default:
	}
}
uint8_t getSVC_number()
{
  __asm(" LDR R0, [R13,#96]");                 //get PC from stack
  __asm(" SUB R0, R0,#2");                     //subtract 2 from PC to get SVC instruction
  __asm(" LDRB R0, [R0]");                     //Load R0 with lower 8 bits of SVC instruction to get SVC number
}
void calc_CpuPercentage(uint8_t i)
{
	__asm(" SVC #07");
}
bool Iscommand(char* strcmd,uint8_t minargs)
      {
    	 bool result=0;
    	 if( strcasecmp(&str[position[0]],strcmd)==0 && field>minargs)
    	 {
    		 result=1;
    	 }
    	  return result;
      }
uint32_t getnumber(uint8_t fieldcount)
      {
    	  uint32_t number=0;
    	  uint8_t i;
    	  for(i=position[fieldcount];i<(position[fieldcount]+strlen(&str[position[fieldcount]]));i++)
    	  {
    	  if((str[i]>=65 && str[i]<=90) || (str[i]>=97 && str[i]<=122))
    	  {
    		  typo=1;
    		  break;
    	  }
    	  }
    	  if(typo!=1)
    	  {
    		  number = atoi(&str[position[fieldcount]]);
    	  }
    	  return number;
      }
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE)
	{
      yield();
	}
	return UART0_DR_R & 0xFF;
}
char* getstring(uint8_t fieldcount)
{
    	  return &str[position[fieldcount]];
}
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* strg)
{
	uint8_t i;
    for (i = 0; i < strlen(strg); i++)
	  putcUart0(strg[i]);
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  bool ok;

  // Initialize hardware
  initHw();
  rtosInit();

  // Power-up flash
  GREEN_LED = 1;
  waitMicrosecond(250000);
  GREEN_LED = 0;
  waitMicrosecond(250000);

  // Initialize semaphores
  keyPressed = createSemaphore(1,"keyPressed");
  keyReleased = createSemaphore(0,"keyReleased");
  flashReq = createSemaphore(5, "flashReq");
  resource = createSemaphore(1,"resource");

   ok =  createThread(idle, "Idle", 7);

   // Add other processes
   ok &= createThread(lengthyFn, "LengthyFn", 6);
   ok &= createThread(flash4Hz, "Flash4Hz", 2);
   ok &= createThread(oneshot, "OneShot", 2);
   ok &= createThread(readKeys, "ReadKeys", 6);
   ok &= createThread(debounce, "Debounce", 6);
   ok &= createThread(important, "Important", 0);
   ok &= createThread(uncooperative, "Uncoop", 5);
   ok &= createThread(shell, "Shell", 4);

  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
}
