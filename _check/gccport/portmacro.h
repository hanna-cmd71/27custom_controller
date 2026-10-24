/*
 * _check/gccport/portmacro.h
 *
 * Syntax-check-only shim: FreeRTOS V10 GCC/ARM_CM4F port macros.
 * The real project ships only the RVDS (armcc) port whose `__asm { ... }`
 * blocks gcc cannot parse. This file exists so `_check/syn.ps1` can run
 * `arm-none-eabi-gcc -fsyntax-only` over application/, bsp/, components/.
 * It is NOT part of the Ke`il build and must never be added to the uvprojx.
 */
#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------- Type definitions. */
#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short

#define portSTACK_TYPE  uint32_t
#define portBASE_TYPE   long

typedef portSTACK_TYPE  StackType_t;
typedef long            BaseType_t;
typedef unsigned long   UBaseType_t;

#if ( configUSE_16_BIT_TICKS == 1 )
	typedef uint16_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffff
#else
	typedef uint32_t TickType_t;
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
	#define portTICK_TYPE_IS_ATOMIC 1
#endif

/* ------------------------------------------------- Architecture specifics. */
#define portSTACK_GROWTH            ( -1 )
#define portTICK_PERIOD_MS          ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT          8

#define portNVIC_INT_CTRL_REG       ( *( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT      ( 1UL << 28UL )

#define portSY_FULL_READ_WRITE      ( 15 )

/* ------------------------------------------------------ Scheduler utilities. */
#define portYIELD()                                                     \
{                                                                       \
	portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;                     \
	__asm volatile ( "dsb" ::: "memory" );                              \
	__asm volatile ( "isb" );                                           \
}

#define portEND_SWITCHING_ISR( xSwitchRequired ) \
	if( ( xSwitchRequired ) != pdFALSE ) { portYIELD(); }

#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )

/* ------------------------------------------------------- Critical section. */
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

#define portDISABLE_INTERRUPTS()                vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS()                 vPortSetBASEPRI( 0 )
#define portENTER_CRITICAL()                    vPortEnterCritical()
#define portEXIT_CRITICAL()                     vPortExitCritical()
#define portSET_INTERRUPT_MASK_FROM_ISR()       ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )  vPortSetBASEPRI( x )

/* ------------------------------------------------------------ Task macros. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) \
	void vFunction( void * pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) \
	void vFunction( void * pvParameters )

/* ------------------------------------------------------- Tickless support. */
#ifndef portSUPPRESS_TICKS_AND_SLEEP
	extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );
	#define portSUPPRESS_TICKS_AND_SLEEP( xExpectedIdleTime ) \
		vPortSuppressTicksAndSleep( xExpectedIdleTime )
#endif

/* ------------------------------------------- Optimised task selection. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if ( configUSE_PORT_OPTIMISED_TASK_SELECTION == 1 )
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) \
		( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) \
		( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )
	#define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) \
		uxTopPriority = ( 31UL - ( uint32_t ) __builtin_clz( ( uxReadyPriorities ) ) )
#endif

/* --------------------------------------------------------------- Misc. */
#define portNOP()               __asm volatile ( "nop" )
#define portINLINE              __inline
#ifndef portFORCE_INLINE
	#define portFORCE_INLINE    __attribute__( ( always_inline ) ) __inline
#endif

#define portALLOCATE_SECURE_CONTEXT( ulSecureStackSize )
#define portCLEAN_UP_TCB( pxTCB ) ( void ) pxTCB

#if ( configASSERT_DEFINED == 1 )
	extern void vPortValidateInterruptPriority( void );
	#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID() \
		vPortValidateInterruptPriority()
#else
	#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()
#endif

/* ------------------------------------------------- Interrupt helpers (GCC). */
static portFORCE_INLINE void vPortSetBASEPRI( uint32_t ulBASEPRI )
{
	__asm volatile ( "msr basepri, %0" ::"r" ( ulBASEPRI ) : "memory" );
}

static portFORCE_INLINE void vPortRaiseBASEPRI( void )
{
	uint32_t ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

	__asm volatile ( "msr basepri, %0" ::"r" ( ulNewBASEPRI ) : "memory" );
	__asm volatile ( "dsb" ::: "memory" );
	__asm volatile ( "isb" );
}

static portFORCE_INLINE void vPortClearBASEPRIFromISR( void )
{
	__asm volatile ( "msr basepri, %0" ::"r" ( 0UL ) : "memory" );
}

static portFORCE_INLINE uint32_t ulPortRaiseBASEPRI( void )
{
	uint32_t ulReturn, ulNewBASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY;

	__asm volatile ( "mrs %0, basepri" : "=r" ( ulReturn ) );
	__asm volatile ( "msr basepri, %0" ::"r" ( ulNewBASEPRI ) : "memory" );
	__asm volatile ( "dsb" ::: "memory" );
	__asm volatile ( "isb" );

	return ulReturn;
}

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
