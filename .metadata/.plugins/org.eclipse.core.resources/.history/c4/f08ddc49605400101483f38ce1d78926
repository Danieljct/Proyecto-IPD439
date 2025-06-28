#define TRUE  1
#define FALSE 0
#define bool BYTE

#include "stm32l4xx_hal.h"
#include "ff_gen_drv.h"
#include "diskio.h"
#include "sd_functions.h"
#include "fatfs.h"
#include "ff.h"

uint16_t Timer1, Timer2;          /* 1ms Timer Counter */

static volatile DSTATUS Stat = STA_NOINIT;  /* Disk Status */
static uint8_t CardType;                    /* Type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0;       /* Power flag */
volatile uint8_t spiDmaTransferComplete = 1; // Bandera para indicar la finalización de la transferencia SPI (por interrupción)

// Implementación de los callbacks de finalización de transferencia SPI por interrupción
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
  // Asegurarse de que el callback es de la SPI de la SD
  if (hspi == HSPI_SDCARD) { // Asumiendo HSPI_SDCARD es el handle de SPI para la SD
      spiDmaTransferComplete = 1;
  }
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
  // Asegurarse de que el callback es de la SPI de la SD
  if (hspi == HSPI_SDCARD) { // Asumiendo HSPI_SDCARD es el handle de SPI para la SD
      spiDmaTransferComplete = 1;
  }
}


/***************************************
 * SPI functions
 **************************************/

/* slave select */
static void SELECT(void)
{
  HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
  // HAL_Delay(1); // Mantenemos el delay opcional si es necesario para el CS setup time
}

/* slave deselect */
static void DESELECT(void)
{
  HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
  // HAL_Delay(1); // Mantenemos el delay opcional si es necesario para el CS hold time
}

/* SPI transmit a byte */
static void SPI_TxByte(uint8_t data)
{
  // Asegurarse de que no hay transferencia previa pendiente antes de iniciar una nueva
  if(spiDmaTransferComplete == 1){
    spiDmaTransferComplete = 0; // Limpia la bandera antes de iniciar la transferencia
    // Usa HAL_SPI_Transmit_IT (modo interrupción)
    HAL_SPI_Transmit_IT(HSPI_SDCARD, &data, 1);
    while(spiDmaTransferComplete == 0); // Esperar a que la transferencia por interrupción termine
  }
  // Si spiDmaTransferComplete no era 1, significa que una transferencia previa aún no ha terminado.
  // En un sistema robusto, esto debería ser manejado como un error o con un timeout.
  // Para este contexto, el bucle de espera lo maneja implícitamente si la bandera no se resetea.
}

static void SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
  if(spiDmaTransferComplete == 1){
    spiDmaTransferComplete = 0; // Limpia la bandera antes de iniciar la transferencia
    // Usa HAL_SPI_Transmit_IT (modo interrupción)
    HAL_SPI_Transmit_IT(HSPI_SDCARD, buffer, len);
    while(spiDmaTransferComplete == 0); // Esperar a que la transferencia por interrupción termine
  }
}

static uint8_t SPI_RxByte(void)
{
  uint8_t dummy, data;
  dummy = 0xFF; // Envía byte dummy para recibir dato
  if (spiDmaTransferComplete == 1)
  {
    spiDmaTransferComplete=0; // Limpia la bandera antes de iniciar la transferencia
    // Usa HAL_SPI_TransmitReceive_IT (modo interrupción)
    HAL_SPI_TransmitReceive_IT(HSPI_SDCARD, &dummy, &data, 1);
    while(spiDmaTransferComplete == 0); // Esperar a que la transferencia por interrupción termine
  }
  return data;
}

static void SPI_RxBytePtr(uint8_t *buff)
{
  uint8_t dummy;
  dummy = 0xFF;

  if (spiDmaTransferComplete == 1)
  {
    spiDmaTransferComplete=0; // Limpia la bandera antes de iniciar la transferencia
    // Usa HAL_SPI_TransmitReceive_IT (modo interrupción)
    HAL_SPI_TransmitReceive_IT(HSPI_SDCARD, &dummy, buff, 1);
    while(spiDmaTransferComplete == 0); // Esperar a que la transferencia por interrupción termine
  }
}

/***************************************
 * SD functions
 **************************************/

/* wait SD ready */
static uint8_t SD_ReadyWait(void)
{
  uint8_t res;

  /* timeout 500ms */
  Timer2 = 500; // Asumiendo que Timer2 es una variable que se decrementa por un SysTick o similar

  /* if SD goes ready, receives 0xFF */
  do {
    res = SPI_RxByte();
  } while ((res != 0xFF) && Timer2); // Este bucle es bloqueante, usando Timer2 como timeout

  return res;
}

/* power on */
static void SD_PowerOn(void)
{
  uint8_t args[6];
  uint32_t cnt = 0x1FFF; // Contador para timeout de espera de respuesta

  /* transmit bytes to wake up */
  DESELECT();
  for(int i = 0; i < 10; i++) // Envía 10 bytes dummy
  {
    SPI_TxByte(0xFF);
  }

  /* slave select */
  SELECT();

  /* make idle state */
  args[0] = CMD0;   /* CMD0:GO_IDLE_STATE */
  args[1] = 0;
  args[2] = 0;
  args[3] = 0;
  args[4] = 0;
  args[5] = 0x95;   /* CRC */

  SPI_TxBuffer(args, sizeof(args)); // Envía el comando CMD0

  /* wait response */
  while ((SPI_RxByte() != 0x01) && cnt) // Espera respuesta 0x01
  {
    cnt--;
  }

  DESELECT();
  SPI_TxByte(0XFF); // Envía un byte dummy para liberar la línea (stuff byte)

  PowerFlag = 1;
}

/* power off */
static void SD_PowerOff(void)
{
  PowerFlag = 0;
}

/* check power flag */
static uint8_t SD_CheckPower(void)
{
  return PowerFlag;
}

/* receive data block */
static bool SD_RxDataBlock(BYTE *buff, UINT len)
{
  uint8_t token;

  /* timeout 200ms */
  Timer1 = 200; // Asumiendo que Timer1 es una variable que se decrementa

  /* loop until receive a response or timeout */
  do {
    token = SPI_RxByte();
  } while((token == 0xFF) && Timer1); // Espera por el token de inicio de bloque

  /* invalid response */
  if(token != 0xFE) return FALSE;

  /* receive data */
  // Usar HAL_SPI_TransmitReceive_IT para leer el bloque completo, y esperar su finalización
  uint8_t dummy_tx_buf[len]; // Buffer de dummies para transmitir
  memset(dummy_tx_buf, 0xFF, len);

  if (spiDmaTransferComplete == 1) {
      spiDmaTransferComplete = 0;
      HAL_SPI_TransmitReceive_IT(HSPI_SDCARD, dummy_tx_buf, buff, len);
      while(spiDmaTransferComplete == 0); // Esperar a que la transferencia termine
  } else {
      // Manejar error o timeout si la bandera no estaba lista para una nueva transmisión
      return FALSE;
  }

  /* discard CRC */
  SPI_RxByte(); // Lee CRC byte 1
  SPI_RxByte(); // Lee CRC byte 2

  return TRUE;
}

/* transmit data block */
#if _USE_WRITE == 1
static bool SD_TxDataBlock(const uint8_t *buff, BYTE token)
{
  uint8_t resp;
  uint8_t i = 0;

  /* wait SD ready */
  if (SD_ReadyWait() != 0xFF) return FALSE; // Asegura que la SD está lista

  /* transmit token */
  SPI_TxByte(token); // Envía el token de escritura

  /* if it's not STOP token, transmit data */
  if (token != 0xFD) // 0xFD es el token de parada de multi-bloque
  {
    // Usa SPI_TxBuffer (que ahora usa _IT) para enviar el bloque de datos
    SPI_TxBuffer((uint8_t*)buff, 512); // Envía el bloque de datos de 512 bytes

    /* discard CRC */
    SPI_RxByte(); // Lee CRC byte 1 (dummy)
    SPI_RxByte(); // Lee CRC byte 2 (dummy)

    /* receive response */
    // Este bucle espera la respuesta de la SD de aceptación del bloque
    while (i <= 64) // Timeout para la respuesta
    {
      resp = SPI_RxByte(); // Recibe la respuesta
      /* transmit 0x05 accepted */
      if ((resp & 0x1F) == 0x05) break; // 0x05 indica que el bloque fue aceptado
      i++;
    }

    /* recv buffer clear */
    // Este bucle asegura que el buffer de recepción SPI esté limpio
    while (SPI_RxByte() == 0); // Espera que la SD libere el busy signal (0)

  }

  /* transmit 0x05 accepted */
  if ((resp & 0x1F) == 0x05) return TRUE; // Retorna éxito si la respuesta fue 0x05

  return FALSE;
}
#endif /* _USE_WRITE */

/* transmit command */
static BYTE SD_SendCmd(BYTE cmd, uint32_t arg)
{
  uint8_t crc, res;

  /* wait SD ready */
  if (SD_ReadyWait() != 0xFF) return 0xFF; // Asegura que la SD está lista

  /* transmit command */
  SPI_TxByte(cmd);          /* Command */
  SPI_TxByte((uint8_t)(arg >> 24));   /* Argument[31..24] */
  SPI_TxByte((uint8_t)(arg >> 16));   /* Argument[23..16] */
  SPI_TxByte((uint8_t)(arg >> 8));  /* Argument[15..8] */
  SPI_TxByte((uint8_t)arg);       /* Argument[7..0] */

  /* prepare CRC */
  if(cmd == CMD0) crc = 0x95; /* CRC for CMD0(0) */
  else if(cmd == CMD8) crc = 0x87;  /* CRC for CMD8(0x1AA) */
  else crc = 1; // Para otros comandos, el CRC es fijo o se recalcula

  /* transmit CRC */
  SPI_TxByte(crc);

  /* Skip a stuff byte when STOP_TRANSMISSION */
  if (cmd == CMD12) SPI_RxByte(); // CMD12 es especial, requiere un byte dummy extra

  /* receive response */
  uint8_t n = 10; // Timeout para la respuesta
  do {
    res = SPI_RxByte(); // Recibe la respuesta del comando
  } while ((res & 0x80) && --n); // Espera hasta que el bit 7 (ocupado) sea 0 o timeout

  return res;
}

/***************************************
 * user_diskio.c functions
 **************************************/

/* initialize SD */
DSTATUS SD_disk_initialize(BYTE drv)
{
  uint8_t n, type, ocr[4];

  /* single drive, drv should be 0 */
  if(drv) return STA_NOINIT;

  /* no disk */
  if(Stat & STA_NODISK) return Stat;

  /* power on */
  SD_PowerOn();

  /* slave select */
  SELECT();

  /* check disk type */
  type = 0;

  /* send GO_IDLE_STATE command */
  if (SD_SendCmd(CMD0, 0) == 1) // Envía CMD0
  {
    /* timeout 1 sec */
    Timer1 = 1000;

    /* SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
    if (SD_SendCmd(CMD8, 0x1AA) == 1) // Envía CMD8
    {
      /* operation condition register */
      for (n = 0; n < 4; n++) // Lee 4 bytes de OCR
      {
        ocr[n] = SPI_RxByte();
      }

      /* voltage range 2.7-3.6V */
      if (ocr[2] == 0x01 && ocr[3] == 0xAA)
      {
        /* ACMD41 with HCS bit */
        do {
          // Envía CMD55 y luego CMD41
          if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0) break;
        } while (Timer1);

        /* READ_OCR */
        if (Timer1 && SD_SendCmd(CMD58, 0) == 0) // Envía CMD58
        {
          /* Check CCS bit */
          for (n = 0; n < 4; n++) // Lee 4 bytes de OCR
          {
            ocr[n] = SPI_RxByte();
          }

          /* SDv2 (HC or SC) */
          type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // Determina tipo de tarjeta
        }
      }
    }
    else
    {
      /* SDC V1 or MMC */
      type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC; // Determina tipo

      do
      {
        if (type == CT_SD1)
        {
          if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0) break; /* ACMD41 */
        }
        else
        {
          if (SD_SendCmd(CMD1, 0) == 0) break; /* CMD1 */
        }

      } while (Timer1);

      /* SET_BLOCKLEN */
      if (!Timer1 || SD_SendCmd(CMD16, 512) != 0) type = 0; // Establece tamaño de bloque a 512 bytes
    }
  }

  CardType = type;

  /* Idle */
  DESELECT(); // Deselecciona la SD
  SPI_RxByte(); // Envía un byte dummy para asegurar el estado IDLE

  /* Clear STA_NOINIT */
  if (type)
  {
    Stat &= ~STA_NOINIT; // Si la inicialización fue exitosa
  }
  else
  {
    /* Initialization failed */
    SD_PowerOff(); // Si falló
  }

  return Stat;
}

/* return disk status */
DSTATUS SD_disk_status(BYTE drv)
{
  if (drv) return STA_NOINIT;
  return Stat;
}

/* read sector */
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
  /* pdrv should be 0 */
  if (pdrv || !count) return RES_PARERR;

  /* no disk */
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  /* convert to byte address */
  if (!(CardType & CT_SD2)) sector *= 512;

  SELECT(); // Selecciona la SD

  if (count == 1)
  {
    /* READ_SINGLE_BLOCK */
    if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) count = 0;
  }
  else
  {
    /* READ_MULTIPLE_BLOCK */
    if (SD_SendCmd(CMD18, sector) == 0)
    {
      do {
        if (!SD_RxDataBlock(buff, 512)) break;
        buff += 512;
      } while (--count);

      /* STOP_TRANSMISSION */
      SD_SendCmd(CMD12, 0); // Envía CMD12 para detener la transmisión multi-bloque
    }
  }

  /* Idle */
  DESELECT(); // Deselecciona la SD
  SPI_RxByte(); // Envía un byte dummy

  return count ? RES_ERROR : RES_OK;
}

/* write sector */
#if _USE_WRITE == 1
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
  /* pdrv should be 0 */
  if (pdrv || !count) return RES_PARERR;

  /* no disk */
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  /* write protection */
  if (Stat & STA_PROTECT) return RES_WRPRT;

  /* convert to byte address */
  if (!(CardType & CT_SD2)) sector *= 512;

  SELECT(); // Selecciona la SD

  if (count == 1)
  {
    /* WRITE_BLOCK */
    if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
      count = 0;
  }
  else
  {
    /* WRITE_MULTIPLE_BLOCK */
    if (CardType & CT_SD1)
    {
      SD_SendCmd(CMD55, 0);
      SD_SendCmd(CMD23, count); /* ACMD23 */
    }

    if (SD_SendCmd(CMD25, sector) == 0)
    {
      do {
        if(!SD_TxDataBlock(buff, 0xFC)) break;
        buff += 512;
      } while (--count);

      /* STOP_TRAN token */
      if(!SD_TxDataBlock(0, 0xFD)) // Envía token de parada multi-bloque
      {
        count = 1;
      }
    }
  }

  /* Idle */
  DESELECT(); // Deselecciona la SD
  SPI_RxByte(); // Envía un byte dummy

  return count ? RES_ERROR : RES_OK;
}
#endif /* _USE_WRITE */

/* ioctl */
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
  DRESULT res;
  uint8_t n, csd[16], *ptr = buff;
  WORD csize;

  /* pdrv should be 0 */
  if (drv) return RES_PARERR;
  res = RES_ERROR;

  if (ctrl == CTRL_POWER)
  {
    switch (*ptr)
    {
    case 0:
      SD_PowerOff();    /* Power Off */
      res = RES_OK;
      break;
    case 1:
      SD_PowerOn();   /* Power On */
      res = RES_OK;
      break;
    case 2:
      *(ptr + 1) = SD_CheckPower();
      res = RES_OK;   /* Power Check */
      break;
    default:
      res = RES_PARERR;
    }
  }
  else
  {
    /* no disk */
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    SELECT(); // Selecciona la SD

    switch (ctrl)
    {
    case GET_SECTOR_COUNT:
      /* SEND_CSD */
      if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
      {
        if ((csd[0] >> 6) == 1)
        {
          /* SDC V2 */
          csize = csd[9] + ((WORD) csd[8] << 8) + 1;
          *(DWORD*) buff = (DWORD) csize << 10;
        }
        else
        {
          /* MMC or SDC V1 */
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
          *(DWORD*) buff = (DWORD) csize << (n - 9);
        }
        res = RES_OK;
      }
      break;
    case GET_SECTOR_SIZE:
      *(WORD*) buff = 512;
      res = RES_OK;
      break;
    case CTRL_SYNC:
      if (SD_ReadyWait() == 0xFF) res = RES_OK;
      break;
    case MMC_GET_CSD:
      /* SEND_CSD */
      if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
      break;
    case MMC_GET_CID:
      /* SEND_CID */
      if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
      break;
    case MMC_GET_OCR:
      /* READ_OCR */
      if (SD_SendCmd(CMD58, 0) == 0)
      {
        for (n = 0; n < 4; n++)
        {
          *ptr++ = SPI_RxByte();
        }
        res = RES_OK;
      }
    default:
      res = RES_PARERR;
    }

    DESELECT();
    SPI_RxByte();
  }

  return res;
}
