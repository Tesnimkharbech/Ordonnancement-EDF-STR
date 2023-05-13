/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>
/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "croutine.h"
#include "timers.h"

#define NDEBUG 
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY	1
#define INCLUDE_vTaskSuspend 1

#define mainREGION_1_SIZE	7201
#define mainREGION_2_SIZE	29905
#define mainREGION_3_SIZE	6407

/*-----------------------------------------------------------*/

static void  prvInitialiseHeap(void);
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName);
void vApplicationTickHook(void);
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize);
static void prvSaveTraceFile(void);

/*-----------------------------------------------------------*/

StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

static BaseType_t xTraceRunning = pdTRUE;

/*-----------------------------------------------------------*/
#define NUM_OF_DISPLAY_SCREENS (5) //number of display screens
#define MAX_NUMBER_ALLOWED (NUM_OF_DISPLAY_SCREENS - 1)
#define left(i) (i)
#define right(i) ((i + 1) % NUM_OF_DISPLAY_SCREENS)
SemaphoreHandle_t waypoints[NUM_OF_DISPLAY_SCREENS]; //shared ressources (same number)
SemaphoreHandle_t entry_sem;
TaskHandle_t display_screen[NUM_OF_DISPLAY_SCREENS]; //Atask handle for each display screen

#define PI 3.14159265
const float g = 9.8f;
//Each display screen has its own data flight to be updated and shown :
double dist[NUM_OF_DISPLAY_SCREENS], Hm[NUM_OF_DISPLAY_SCREENS], v[NUM_OF_DISPLAY_SCREENS];

//Flight data Calculation (Updating) for a specefic display screen i :
void flight_data_Calc(int i) {

	double x[] = { 154.25, 10800.5, 12.0, 25.5, 516.75 };
	double y[] = { 4051.0, 18.6, 50246.25, 1997.0, 2000.0 };
	double ang = 2.23, Rz = 0.0, Rx = 0.0, m = 1000.0, S = 40.0, Cz = 1.0;

	dist[i] = sqrt(pow((x[left(i)] - x[right(i)]), 2) + pow((y[left(i)] - y[right(i)]), 2));
	Hm[i] = dist[i] * tan(ang * PI / 180.0);
	Rz = g * m;
	v[i] = sqrt(2 * Rz / (1.225 * S * Cz));
	vTaskDelay(100);
}
//Displaying the most recent updated flight data for a specefic display screen i :
void display_flight_data(int i) {
	printf("Updated Flight Parameters (based on the waypoints n %d & n %d) :\n", left(i), right(i));
	fflush(stdout);
	printf("Distance= %.3f km\n", dist[i]);
	fflush(stdout);
	printf("Altitude= %.3f km\n", Hm[i]);
	fflush(stdout);
	printf("Velocity= %.3f km/h\n", v[i] * 0.36);
	fflush(stdout);
}
//The display screen i wants to use the 2 waypoints (left and right) :
void use_waypoint(int i) {
	xSemaphoreTake(waypoints[left(i)], portMAX_DELAY);
	xSemaphoreTake(waypoints[right(i)], portMAX_DELAY);
	printf("The navigation display screen n %d is using the waypoint n %d and n %d to update the aircraft's navigation parameters.\n",i,left(i),right(i));
	fflush(stdout);
}
//The display screen i finished using the 2 waypoints (left and right) and is releasing them :
void release_waypoint(int i) {
	xSemaphoreGive(waypoints[left(i)]);
	xSemaphoreGive(waypoints[right(i)]);
	printf("The navigation display screen n %d released the waypoint n %d and n %d.\n", i, left(i), right(i));
	fflush(stdout);
}

void display_screen_task(void* param) {

	int i = *(int*)param;
	while (1) {
		xSemaphoreTake(entry_sem, portMAX_DELAY);
		printf("The navigation display screen n %d is showing the most recent updated data.\n", i);
		display_flight_data(i);
		//The navigation display screen i wants to update its flight data
		use_waypoint(i);
		printf("The navigation display screen n %d is calculating the aircraft's navigation parameters.\n", i);
		fflush(stdout);
		flight_data_Calc(i);
		vTaskDelay(2000);
		release_waypoint(i);

		xSemaphoreGive(entry_sem);

		vTaskDelay(2000);
	}
}

int main(void)
{
	int i;
	int param[NUM_OF_DISPLAY_SCREENS];
	prvInitialiseHeap();
	vTraceEnable(TRC_START);
	// Creating the five shared resources (waypoints)
	
	for (i = 0; i < NUM_OF_DISPLAY_SCREENS; i++) {
		waypoints[i] = xSemaphoreCreateMutex();
	}
	
	// This line avoids any deadlock:
	entry_sem = xSemaphoreCreateCounting(MAX_NUMBER_ALLOWED, MAX_NUMBER_ALLOWED);

	for (i = 0; i < NUM_OF_DISPLAY_SCREENS; i++) {
		param[i] = i;
		xTaskCreate(display_screen_task, "task", 30, &(param[i]), 2, NULL);
	}

	vTaskStartScheduler();
	for (;;);
	return 0;
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c, heap_2.c or heap_4.c is being used, then the
	size of the	heap available to pvPortMalloc() is defined by
	configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the xPortGetFreeHeapSize()
	API function can be used to query the size of free heap space that remains
	(although it does not provide information on how the remaining heap might be
	fragmented).  See http://www.freertos.org/a00111.html for more
	information. */
	vAssertCalled(__LINE__, __FILE__);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If application tasks make use of the
	vTaskDelete() API function to delete themselves then it is also important
	that vApplicationIdleHook() is permitted to return to its calling function,
	because it is the responsibility of the idle task to clean up memory
	allocated by the kernel to any task that has since deleted itself. */

	/* Uncomment the following code to allow the trace to be stopped with any
	key press.  The code is commented out by default as the kbhit() function
	interferes with the run time behaviour. */
	/*
		if( _kbhit() != pdFALSE )
		{
			if( xTraceRunning == pdTRUE )
			{
				vTraceStop();
				prvSaveTraceFile();
				xTraceRunning = pdFALSE;
			}
		}
	*/

#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		/* Call the idle task processing used by the full demo.  The simple
		blinky demo does not use the idle task hook.
		vFullDemoIdleFunction();*/
	}
#endif


}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName)
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  This function is
	provided as an example only as stack overflow checking does not function
	when running the FreeRTOS Windows port. */
	vAssertCalled(__LINE__, __FILE__);
}
/*-----------------------------------------------------------*/
/*
void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()).
#if ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 )
	{
		vFullDemoTickHookFunction();
	}
#endif // mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
//	printf("TICK : %d\n", (int)xTaskGetTickCount());
	*/

void vApplicationTickHook(void)
{
	/* The tickhook just increments the timers if tasks are active */
	/*ticks++;
	if (clc_active == TRUE)
		clc_ticks++;

	if (comm_active == TRUE)
		comm_ticks++;
		*/
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook(void)
{
	/* This function will be called once only, when the daemon task starts to
	execute	(sometimes called the timer task).  This is useful if the
	application includes initialisation code that would benefit from executing
	after the scheduler has been started. */
}
/*-----------------------------------------------------------*/

void vAssertCalled(unsigned long ulLine, const char* const pcFileName)
{
	static BaseType_t xPrinted = pdFALSE;
	volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

	/* Called if an assertion passed to configASSERT() fails.  See
	http://www.freertos.org/a00110.html#configASSERT for more information. */

	/* Parameters are not used. */
	(void)ulLine;
	(void)pcFileName;

	printf("ASSERT! Line %ld, file %s, GetLastError() %ld\r\n", ulLine, pcFileName, GetLastError());

	taskENTER_CRITICAL();
	{
		/* Stop the trace recording. */
		if (xPrinted == pdFALSE)
		{
			xPrinted = pdTRUE;
			if (xTraceRunning == pdTRUE)
			{
				vTraceStop();
				prvSaveTraceFile();
			}
		}

		/* You can step out of this function to debug the assertion by using
		the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
		value. */
		while (ulSetToNonZeroInDebuggerToContinue == 0)
		{
			__asm { NOP };
			__asm { NOP };
		}
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

static void prvSaveTraceFile(void)
{
	FILE* pxOutputFile;

	fopen_s(&pxOutputFile, "Trace.dump", "wb");

	if (pxOutputFile != NULL)
	{
		fwrite(RecorderDataPtr, sizeof(RecorderDataType), 1, pxOutputFile);
		fclose(pxOutputFile);
		printf("\r\nTrace output saved to Trace.dump\r\n");
	}
	else
	{
		printf("\r\nFailed to create trace dump file\r\n");
	}
}
/*-----------------------------------------------------------*/

static void  prvInitialiseHeap(void)
{
	/* The Windows demo could create one large heap region, in which case it would
	be appropriate to use heap_4.  However, purely for demonstration purposes,
	heap_5 is used instead, so start by defining some heap regions.  No
	initialisation is required when any other heap implementation is used.  See
	http://www.freertos.org/a00111.html for more information.

	The xHeapRegions structure requires the regions to be defined in start address
	order, so this just creates one big array, then populates the structure with
	offsets into the array - with gaps in between and messy alignment just for test
	purposes. */
	static uint8_t ucHeap[configTOTAL_HEAP_SIZE];
	volatile uint32_t ulAdditionalOffset = 19; /* Just to prevent 'condition is always true' warnings in configASSERT(). */
	const HeapRegion_t xHeapRegions[] =
	{
		/* Start address with dummy offsets						Size */
		{ ucHeap + 1,											mainREGION_1_SIZE },
		{ ucHeap + 15 + mainREGION_1_SIZE,						mainREGION_2_SIZE },
		{ ucHeap + 19 + mainREGION_1_SIZE + mainREGION_2_SIZE,	mainREGION_3_SIZE },
		{ NULL, 0 }
	};

	/* Sanity check that the sizes and offsets defined actually fit into the
	array. */
	configASSERT((ulAdditionalOffset + mainREGION_1_SIZE + mainREGION_2_SIZE + mainREGION_3_SIZE) < configTOTAL_HEAP_SIZE);

	/* Prevent compiler warnings when configASSERT() is not defined. */
	(void)ulAdditionalOffset;

	vPortDefineHeapRegions(xHeapRegions);
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize)
{
	/* If the buffers to be provided to the Idle task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
	state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize)
{
	/* If the buffers to be provided to the Timer task are declared inside this
	function then they must be declared static - otherwise they will be allocated on
	the stack and so not exists after this function exits. */
	static StaticTask_t xTimerTaskTCB;

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
