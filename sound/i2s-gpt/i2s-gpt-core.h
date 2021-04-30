#ifndef __GPT_I2S_H__
#define __GPT_I2S_H__

/*  I2S Status Flags */
#define I2S_RAW_FRAME_SYNC           (0x1 << 0)
#define I2S_RAW_RX_OVERFLOW          (0x1 << 1)
#define I2S_RAW_TX_UNDERFLOW         (0x1 << 2)
#define I2S_RAW_RXR_DATA_AVAILABLE   (0x1 << 3)
#define I2S_RAW_RXL_DATA_AVAILABLE   (0x1 << 4)
#define I2S_RAW_RXR_HIGH             (0x1 << 5)
#define I2S_RAW_RXR_LOW              (0x1 << 6)
#define I2S_RAW_RXL_HIGH             (0x1 << 7)
#define I2S_RAW_RXL_LOW              (0x1 << 8)
#define I2S_RAW_TXR_FULL             (0x1 << 9)
#define I2S_RAW_TXL_FULL             (0x1 << 10)
#define I2S_RAW_TXR_HIGH             (0x1 << 11)
#define I2S_RAW_TXR_LOW              (0x1 << 12)
#define I2S_RAW_TXL_HIGH             (0x1 << 13)
#define I2S_RAW_TXL_LOW              (0x1 << 14)

#define I2S_MODE_sft			  (0)
#define I2S_CHSIZE_sft			  (2)
#define I2S_FRMSIZE_sft			  (4)

/*  I2S Control Register Definitions */
#define MODE_I2S                  (0x0)
#define MODE_I2S_LJ               (0x1)
#define MODE_I2S_RJ               (0x2)
#define I2S_CHSIZE_16             (0x0 << 2)
#define I2S_CHSIZE_20             (0x1 << 2)
#define I2S_CHSIZE_24             (0x2 << 2)
#define I2S_CHSIZE_32             (0x3 << 2)
#define I2S_FRMSIZE_32            (0x0 << 4)
#define I2S_FRMSIZE_48            (0x1 << 4)
#define I2S_FRMSIZE_64            (0x2 << 4)
#define I2S_FRMSIZE_128           (0x3 << 4)
#define I2S_FRMSIZE_256           (0x4 << 4)
#define I2S_FRMSIZE_384           (0x5 << 4)
#define I2S_EN                    (0x1 << 7)
#define I2S_DIS                   (0x0 << 7)
#define I2S_BTXL_1Q               (0x0 << 8)
#define I2S_BTXL_2Q               (0x1 << 8)
#define I2S_BTXL_3Q               (0x2 << 8)
#define I2S_BTXH_1Q               (0x0 << 10)
#define I2S_BTXH_2Q               (0x1 << 10)
#define I2S_BTXH_3Q               (0x2 << 10)
#define I2S_BRXL_1Q               (0x0 << 12)
#define I2S_BRXL_2Q               (0x1 << 12)
#define I2S_BRXL_3Q               (0x2 << 12)
#define I2S_BRXH_1Q               (0x0 << 14)
#define I2S_BRXH_2Q               (0x1 << 14)
#define I2S_BRXH_3Q               (0x2 << 14)

#define I2S_FRAME_SYNC_EN         (I2S_RAW_FRAME_SYNC         << 16)
#define I2S_RX_OVERFLOW_EN        (I2S_RAW_RX_OVERFLOW        << 16)
#define I2S_TX_UNDERFLOW_EN       (I2S_RAW_TX_UNDERFLOW       << 16)
#define I2S_RXR_DATA_AVAIL_EN     (I2S_RAW_RXR_DATA_AVAILABLE << 16)
#define I2S_RXL_DATA_AVAIL_EN     (I2S_RAW_RXL_DATA_AVAILABLE << 16)
#define I2S_RXR_HIGH_EN           (I2S_RAW_RXR_HIGH           << 16)
#define I2S_RXR_LOW_EN            (I2S_RAW_RXR_LOW            << 16)
#define I2S_RXL_HIGH_EN           (I2S_RAW_RXL_HIGH           << 16)
#define I2S_RXL_LOW_EN            (I2S_RAW_RXL_LOW            << 16)
#define I2S_TXR_FULL_EN           (I2S_RAW_TXR_FULL           << 16)
#define I2S_TXL_FULL_EN           (I2S_RAW_TXL_FULL           << 16)
#define I2S_TXR_HIGH_EN           (I2S_RAW_TXR_HIGH           << 16)
#define I2S_TXR_LOW_EN            (I2S_RAW_TXR_LOW            << 16)
#define I2S_TXL_HIGH_EN           (I2S_RAW_TXL_HIGH           << 16)
#define I2S_TXL_LOW_EN            (I2S_RAW_TXL_LOW            << 16)

#define I2S_MASTER_sft            (31)
#define I2S_MASTER                (0x1 << I2S_MASTER_sft)

#define I2S_UMASK_BST         (I2S_BTXL_2Q | I2S_BRXH_2Q)

#define I2S_UMASK_INT		  (I2S_RXR_HIGH_EN | I2S_RXL_HIGH_EN | I2S_TXR_LOW_EN | I2S_TXL_LOW_EN | I2S_RXR_DATA_AVAIL_EN | I2S_RXL_DATA_AVAIL_EN)

#define I2S_TXL     (0x10)
#define I2S_TXR     (0x14)
#define I2S_RXL     (0x20)
#define I2S_RXR     (0x24)

#define I2S_UCR			0xFFC
#define I2S_USFR		0xFF8

extern int i2s_gpt_mode_setup(u8 mode, u8 align_mode, u16 sample_size, 
		u16 frame_size);
extern void i2s_gpt_enable(bool enable);
extern int i2s_gpt_playback(u32 L_data, u32 R_data);
extern uint64_t i2s_gpt_capture(void);
extern int i2s_gpt_sclk_setup(unsigned int sclk_freq);
#endif
