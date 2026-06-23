#include "Cus_CAN_RTOS.h"
#include "CAN_Cus.h"



#if (USE_DEFAULT_RxFIFO_FULL_HOOK && CUS_USE_RTOS && CUS_CAN_RTOS_CMSIS)
  #include "cmsis_os2.h"

  static osThreadAttr_t fifoFullThreadAttr;
  static osThreadId_t fifoFullThreadID;

  static osSemaphoreAttr_t fifoFull_SemaphoreAttr;
  static osSemaphoreId_t g_FIfoFullsemaphore;

  __NO_RETURN void backUP( void )
  {
    while (1)
    {
      if ( osSemaphoreAcquire(g_FIfoFullsemaphore, osWaitForever) == osOK )
      {
        Cus_CAN_ProcessBackupBuffers();
      }
    }
  }


  void FIFO_Full_threadCreate( void )
  {
    fifoFull_SemaphoreAttr.name = "FifoFull";
    g_FIfoFullsemaphore = osSemaphoreNew(1, 0, &fifoFull_SemaphoreAttr);
    if ( !g_FIfoFullsemaphore )
    {
      for( ; ; );
    }

    fifoFullThreadAttr.name = "fifoFullTask";
    fifoFullThreadAttr.priority = osPriorityRealtime7;
    fifoFullThreadAttr.stack_size = 128 * 4;

    fifoFullThreadID = osThreadNew(backUP, NULL, &fifoFullThreadAttr);
    if ( !fifoFullThreadID )
    {
      for( ; ; );
    }
  }


  void Cus_CAN_BackupNotifyFromISR( void )
  {
    osSemaphoreRelease(g_FIfoFullsemaphore);
  }

#endif /* USE_DEFAULT_RxFIFO_FULL_HOOK */





