#ifndef __GSM_TEST_H
#define __GSM_TEST_H

/*
 * at process event
 */
#define LORA_MAC_FRAME_NEED_ACK 0x10

#define RS485_MAC_FRAME_SYNC 0xaa
typedef __packed  struct _LoarMacFrame {
	uint8_t sync;
	uint8_t macheader;
	uint32_t dstaddr;
	uint32_t srcaddr;
	uint8_t cmd;
}LoraMacFrame;
typedef struct
{
    LoraMacFrame **   p_pack_buf;           /**< Pointer to FIFO buffer memory.                      */
    uint16_t           buf_size_mask;   /**< Read/write index mask. Also used for size checking. */
    volatile uint32_t  read_pos;        /**< Next read position in the FIFO buffer.              */
    volatile uint32_t  write_pos;       /**< Next write position in the FIFO buffer.             */
} LoraPackFifo;


typedef __packed struct _LoraUpgradeAskPackage {
	uint32_t offset;
} LoraUpgradeAskPackage;

typedef __packed struct _LoraUpgradeSendPackage {
	uint32_t offset;
	uint8_t buf[16];
} LoraUpgradeSendPackage;

enum LORA_CMD {
	LORA_CMD_HTBT = 1,
	LORA_CMD_HTBT_ACK,
	LORA_CMD_LOCK,
	LORA_CMD_LOCK_ACK,
	LORA_CMD_POWER_REPORT,
	LORA_CMD_LOCK_REPORT,
	LORA_CMD_LOCK_REPORT_ACK,
	LORA_CMD_CHARGE_FULL,
	LORA_CMD_UNLOCK,
	LORA_CMD_UNLOCK_ACK,
	LORA_CMD_QUERY,
	LORA_CMD_QUERY_ACK,
	LORA_CMD_QUERY_ACK_WITH_LOCKED,
	LORA_CMD_ICCARD_REPORT,
	LORA_CMD_H_QUERY,
	LORA_CMD_H_QUERY_ACK,
	LORA_CMD_UPGRADE_ASK_PACKAGE,		/* client发送设备请求 */
	LORA_CMD_UPGRADE_SEND_PACKAGE,	/* master应答的数据 */
};


#endif

