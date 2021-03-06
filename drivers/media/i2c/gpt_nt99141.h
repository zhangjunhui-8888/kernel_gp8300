
#ifndef GPT_NT99141_H_
#define GPT_NT99141_H_

#ifndef END_FLAG
#define END_FLAG  0xfc
#endif

#define NT99141_MAX_WIDTH 1920
#define NT99141_MAX_HEIGHT 1080

#define NT99141_PAGE 0x0
#define NT99141_PAGE_MAX 0X1

static const unsigned char INIT_TAB_720P[]={
	0x31, 0x09,0x04,
        0x30, 0x69,0x03,
        0x30, 0x6A,0x05,
        0x30, 0x40,0x04,
        0x30, 0x41,0x02,
        0x30, 0x42,0xFF,
        0x30, 0x43,0x08,
        0x30, 0x52,0xE0,
        0x30, 0x5F,0x66,
        0x31, 0x06,0x03,
        0x31, 0x10,0x22,
        0x31, 0x11,0x54,
        0x31, 0x12,0x22,
        0x31, 0x13,0x55,
        0x31, 0x14,0x05,
        0x32, 0xF0,0x00,
        0x32, 0x90,0x01,
        0x32, 0x91,0x88,
        0x32, 0x96,0x01,
        0x32, 0x97,0xC1,
        0x32, 0x50,0x80,
        0x32, 0x57,0x60,
        0x32, 0x9B,0x01,
        0x32, 0xA1,0x00,
        0x32, 0xA2,0xDA,
        0x32, 0xA3,0x01,
        0x32, 0xA4,0x6E,
        0x32, 0xA5,0x01,
        0x32, 0xA6,0x39,
        0x32, 0xA7,0x01,
        0x32, 0xA8,0xEB,
        0x32, 0x10,0x10,
        0x32, 0x11,0x10,
        0x32, 0x12,0x10,
        0x32, 0x13,0x10,
        0x32, 0x14,0x0C,
        0x32, 0x15,0x0C,
        0x32, 0x16,0x0C,
        0x32, 0x17,0x0C,
        0x32, 0x18,0x0C,
        0x32, 0x19,0x0C,
        0x32, 0x1A,0x0C,
        0x32, 0x1B,0x0C,
        0x32, 0x1C,0x09,
        0x32, 0x1D,0x09,
        0x32, 0x1E,0x09,
        0x32, 0x1F,0x09,
        0x32, 0xB0,0x55,
        0x32, 0xB1,0x7D,
        0x32, 0xB2,0x7D,
        0x32, 0xB3,0x55,
        0x32, 0x70,0x0A,
        0x32, 0x71,0x14,
        0x32, 0x72,0x1E,
        0x32, 0x73,0x32,
        0x32, 0x74,0x45,
        0x32, 0x75,0x56,
        0x32, 0x76,0x75,
        0x32, 0x77,0x90,
        0x32, 0x78,0xA7,
        0x32, 0x79,0xBA,
        0x32, 0x7A,0xD9,
        0x32, 0x7B,0xEB,
        0x32, 0x7C,0xF5,
        0x32, 0x7D,0xF9,
        0x32, 0x7E,0xFA,
        0x33, 0x02,0x00,
        0x33, 0x03,0x4D,
        0x33, 0x04,0x00,
        0x33, 0x05,0x96,
        0x33, 0x06,0x00,
        0x33, 0x07,0x1D,
        0x33, 0x08,0x07,
        0x33, 0x09,0xCF,
        0x33, 0x0A,0x06,
        0x33, 0x0B,0x92,
        0x33, 0x0C,0x01,
        0x33, 0x0D,0x9F,
        0x33, 0x0E,0x00,
        0x33, 0x0F,0xC4,
        0x33, 0x10,0x07,
        0x33, 0x11,0x64,
        0x33, 0x12,0x07,
        0x33, 0x13,0xD9,
        0x32, 0xF6,0x0F,
        0x32, 0xF9,0x63,
        0x32, 0xFA,0x36,
        0x33, 0x26,0x02,
        0x33, 0x27,0x04,
        0x33, 0x28,0x04,
        0x33, 0x29,0x02,
        0x33, 0x2A,0x02,
        0x33, 0x2B,0x1D,
        0x33, 0x2C,0x1D,
        0x33, 0x2D,0x04,
        0x33, 0x2E,0x1E,
        0x33, 0x2F,0x1F,
        0x33, 0x25,0x5F,
        0x33, 0x30,0x00,
        0x33, 0x31,0x0A,
        0x33, 0x32,0x60,
        0x33, 0x38,0x18,
        0x33, 0x39,0xA5,
        0x33, 0x3A,0x5A,
        0x33, 0x3F,0x17,
        0x33, 0x60,0x10,
        0x33, 0x61,0x18,
        0x33, 0x62,0x2F,
        0x33, 0x63,0x37,
        0x33, 0x64,0x80,
        0x33, 0x65,0x80,
        0x33, 0x66,0x68,
        0x33, 0x67,0x50,
        0x33, 0x68,0x60,
        0x33, 0x69,0x50,
        0x33, 0x6A,0x40,
        0x33, 0x6B,0x30,
        0x33, 0x6C,0x00,
        0x33, 0x6D,0x20,
        0x33, 0x6E,0x1C,
        0x33, 0x6F,0x18,
        0x33, 0x70,0x10,
        0x33, 0x71,0x30,
        0x33, 0x72,0x3F,
        0x33, 0x73,0x3F,
        0x33, 0x74,0x3F,
        0x33, 0x75,0x03,
        0x33, 0x76,0x03,
        0x33, 0x77,0x03,
        0x33, 0x78,0x03,
        0x33, 0x79,0x06,
        0x33, 0x7A,0x10,
        0x33, 0x7B,0x28,
        0x33, 0x7C,0x28,
        0x33, 0x7D,0x10,
        0x33, 0x7E,0x14,
        0x33, 0x7F,0x18,
        0x33, 0x80,0x20,
        0x33, 0x81,0x60,
        0x33, 0x82,0x60,
        0x33, 0x83,0x7F,
        0x33, 0x84,0x7F,
        0x33, 0x85,0xFF,
        0x33, 0x8A,0x34,
        0x33, 0x8B,0x7F,
        0x33, 0x8C,0x10,
        0x33, 0x8D,0x23,
        0x33, 0x8E,0x7F,
        0x33, 0x8F,0x14,
        0x30, 0x12,0x02,
        0x30, 0x13,0x1E,
        0x32, 0xBF,0x40,
        0x32, 0xC0,0x7E,
        0x32, 0xC1,0x7E,
        0x32, 0xC2,0x7E,
        0x32, 0xC3,0x00,
        0x32, 0xC4,0x20,
        0x32, 0xC5,0x20,
        0x32, 0xC6,0x20,
        0x32, 0xC7,0x00,
        0x32, 0xC8,0x6C,
        0x32, 0xC9,0x7E,
        0x32, 0xCA,0x9E,
        0x32, 0xCB,0x9E,
        0x32, 0xCC,0x9E,
        0x32, 0xCD,0x9E,
        0x32, 0xDB,0x6B,
        0x32, 0xD0,0x01,
        0x32, 0x00,0x3E,
        0x32, 0x01,0x0F,
        0x30, 0x28,0x08,
        0x30, 0x29,0x02,
        0x30, 0x2A,0x00,
        0x30, 0x22,0x25,
        0x30, 0x23,0x24,
        0x30, 0x02,0x00,
        0x30, 0x03,0x04,
        0x30, 0x04,0x00,
        0x30, 0x05,0x04,
        0x30, 0x06,0x05,
        0x30, 0x07,0x03,
        0x30, 0x08,0x02,
        0x30, 0x09,0xD3,
        0x30, 0x0A,0x06,
        0x30, 0x0B,0x7C,
        0x30, 0x0C,0x04,
        0x30, 0x0D,0x3C,
        0x30, 0x0E,0x05,
        0x30, 0x0F,0x00,
        0x30, 0x10,0x02,
        0x30, 0x11,0xD0,
        0x32, 0xB8,0x36,
        0x32, 0xB9,0x2A,
        0x32, 0xBB,0x87,
        0x32, 0xBC,0x30,
        0x32, 0xBD,0x33,
        0x32, 0xBE,0x2D,
        0x32, 0x01,0x3F,
        0x30, 0x21,0x06,
        0x30, 0x60,0x01,
        0x30, 0x25,0x02,
	END_FLAG,END_FLAG,END_FLAG,

};


static const unsigned char INIT_TAB_36M_720P[]={
	0x31, 0x09,0x04,
        0x30, 0x69,0x03,
        0x30, 0x6A,0x05,
        0x30, 0x40,0x04,
        0x30, 0x41,0x02,
        0x30, 0x42,0xFF,
        0x30, 0x43,0x08,
        0x30, 0x52,0xE0,
        0x30, 0x5F,0x66,
        0x31, 0x06,0x03,
        0x31, 0x10,0x22,
        0x31, 0x11,0x54,
        0x31, 0x12,0x22,
        0x31, 0x13,0x55,
        0x31, 0x14,0x05,
        0x32, 0xF0,0x00,
        0x32, 0xF1,0x10,
        0x32, 0xF3,0x10,
        0x32, 0x90,0x01,
        0x32, 0x91,0x88,
        0x32, 0x96,0x01,
        0x32, 0x97,0xC1,
        0x32, 0x50,0x80,
        0x32, 0x57,0x60,
        0x32, 0x9B,0x01,
        0x32, 0xA1,0x00,
        0x32, 0xA2,0xDA,
        0x32, 0xA3,0x01,
        0x32, 0xA4,0x6E,
        0x32, 0xA5,0x01,
        0x32, 0xA6,0x39,
        0x32, 0xA7,0x01,
        0x32, 0xA8,0xEB,
        0x32, 0x10,0x10,
        0x32, 0x11,0x10,
        0x32, 0x12,0x10,
        0x32, 0x13,0x10,
        0x32, 0x14,0x0C,
        0x32, 0x15,0x0C,
        0x32, 0x16,0x0C,
        0x32, 0x17,0x0C,
        0x32, 0x18,0x0C,
        0x32, 0x19,0x0C,
        0x32, 0x1A,0x0C,
        0x32, 0x1B,0x0C,
        0x32, 0x1C,0x09,
        0x32, 0x1D,0x09,
        0x32, 0x1E,0x09,
        0x32, 0x1F,0x09,
        0x32, 0xB0,0x55,
        0x32, 0xB1,0x7D,
        0x32, 0xB2,0x7D,
        0x32, 0xB3,0x55,
        0x32, 0x70,0x0A,
        0x32, 0x71,0x14,
        0x32, 0x72,0x1E,
        0x32, 0x73,0x32,
        0x32, 0x74,0x45,
        0x32, 0x75,0x56,
        0x32, 0x76,0x75,
        0x32, 0x77,0x90,
        0x32, 0x78,0xA7,
        0x32, 0x79,0xBA,
        0x32, 0x7A,0xD9,
        0x32, 0x7B,0xEB,
        0x32, 0x7C,0xF5,
        0x32, 0x7D,0xF9,
        0x32, 0x7E,0xFA,
        0x33, 0x02,0x00,
        0x33, 0x03,0x4D,
        0x33, 0x04,0x00,
        0x33, 0x05,0x96,
        0x33, 0x06,0x00,
        0x33, 0x07,0x1D,
        0x33, 0x08,0x07,
        0x33, 0x09,0xCF,
        0x33, 0x0A,0x06,
        0x33, 0x0B,0x92,
        0x33, 0x0C,0x01,
        0x33, 0x0D,0x9F,
        0x33, 0x0E,0x00,
        0x33, 0x0F,0xC4,
        0x33, 0x10,0x07,
        0x33, 0x11,0x64,
        0x33, 0x12,0x07,
        0x33, 0x13,0xD9,
        0x32, 0xF6,0x0F,
        0x32, 0xF9,0x63,
        0x32, 0xFA,0x36,
        0x33, 0x26,0x02,
        0x33, 0x27,0x04,
        0x33, 0x28,0x04,
        0x33, 0x29,0x02,
        0x33, 0x2A,0x02,
        0x33, 0x2B,0x1D,
        0x33, 0x2C,0x1D,
        0x33, 0x2D,0x04,
        0x33, 0x2E,0x1E,
        0x33, 0x2F,0x1F,
        0x33, 0x25,0x5F,
        0x33, 0x30,0x00,
        0x33, 0x31,0x0A,
        0x33, 0x32,0x60,
        0x33, 0x38,0x18,
        0x33, 0x39,0xA5,
        0x33, 0x3A,0x5A,
        0x33, 0x3F,0x17,
        0x33, 0x60,0x10,
        0x33, 0x61,0x18,
        0x33, 0x62,0x2F,
        0x33, 0x63,0x37,
        0x33, 0x64,0x80,
        0x33, 0x65,0x80,
        0x33, 0x66,0x68,
        0x33, 0x67,0x50,
        0x33, 0x68,0x60,
        0x33, 0x69,0x50,
        0x33, 0x6A,0x40,
        0x33, 0x6B,0x30,
        0x33, 0x6C,0x00,
        0x33, 0x6D,0x20,
        0x33, 0x6E,0x1C,
        0x33, 0x6F,0x18,
        0x33, 0x70,0x10,
        0x33, 0x71,0x30,
        0x33, 0x72,0x3F,
        0x33, 0x73,0x3F,
        0x33, 0x74,0x3F,
        0x33, 0x75,0x03,
        0x33, 0x76,0x03,
        0x33, 0x77,0x03,
        0x33, 0x78,0x03,
        0x33, 0x79,0x06,
        0x33, 0x7A,0x10,
        0x33, 0x7B,0x28,
        0x33, 0x7C,0x28,
        0x33, 0x7D,0x10,
        0x33, 0x7E,0x14,
        0x33, 0x7F,0x18,
        0x33, 0x80,0x20,
        0x33, 0x81,0x60,
        0x33, 0x82,0x60,
        0x33, 0x83,0x7F,
        0x33, 0x84,0x7F,
        0x33, 0x85,0xFF,
        0x33, 0x8A,0x34,
        0x33, 0x8B,0x7F,
        0x33, 0x8C,0x10,
        0x33, 0x8D,0x23,
        0x33, 0x8E,0x7F,
        0x33, 0x8F,0x14,
        0x30, 0x12,0x02,
        0x30, 0x13,0x1E,
        0x32, 0xBF,0x40,
        0x32, 0xC0,0x7E,
        0x32, 0xC1,0x7E,
        0x32, 0xC2,0x7E,
        0x32, 0xC3,0x00,
        0x32, 0xC4,0x20,
        0x32, 0xC5,0x20,
        0x32, 0xC6,0x20,
        0x32, 0xC7,0x00,
        0x32, 0xC8,0x6C,
        0x32, 0xC9,0x7E,
        0x32, 0xCA,0x9E,
        0x32, 0xCB,0x9E,
        0x32, 0xCC,0x9E,
        0x32, 0xCD,0x9E,
        0x32, 0xDB,0x6B,
        0x32, 0xD0,0x01,
        0x32, 0x00,0x3E,
        0x32, 0x01,0x0F,
        0x30, 0x28,0x08,
        0x30, 0x29,0x02,
        0x30, 0x2A,0x00,
        //0x30, 0x22,0x25,
        0x30, 0x22,0x26,
        0x30, 0x23,0x24,
        0x30, 0x02,0x00,
        0x30, 0x03,0x04,
        0x30, 0x04,0x00,
        0x30, 0x05,0x04,
        0x30, 0x06,0x05,
        0x30, 0x07,0x03,
        0x30, 0x08,0x02,
        0x30, 0x09,0xD3,
        0x30, 0x0A,0x06,
        0x30, 0x0B,0x7C,
        0x30, 0x0C,0x04,
        0x30, 0x0D,0x3C,
        0x30, 0x0E,0x05,
        0x30, 0x0F,0x00,
        0x30, 0x10,0x02,
        0x30, 0x11,0xD0,
        0x32, 0xB8,0x36,
        0x32, 0xB9,0x2A,
        0x32, 0xBB,0x87,
        0x32, 0xBC,0x30,
        0x32, 0xBD,0x33,
        0x32, 0xBE,0x2D,
        0x32, 0x01,0x3F,
        0x30, 0x21,0x06,
        0x30, 0x60,0x01,
	END_FLAG,END_FLAG,END_FLAG,
};


static const unsigned char INIT_TAB_745M_720P[]={
	0x31, 0x09,0x04,
        0x30, 0x69,0x03,
        0x30, 0x6A,0x05,
        0x30, 0x40,0x04,
        0x30, 0x41,0x02,
        0x30, 0x42,0xFF,
        0x30, 0x43,0x08,
        0x30, 0x52,0xE0,
        0x30, 0x5F,0x66,
        0x31, 0x06,0x03,
        0x31, 0x10,0x22,
        0x31, 0x11,0x54,
        0x31, 0x12,0x22,
        0x31, 0x13,0x55,
        0x31, 0x14,0x05,
        0x32, 0xF0,0x0 ,
        0x32, 0x90,0x01,
        0x32, 0x91,0x88,
        0x32, 0x96,0x01,
        0x32, 0x97,0xC1,
        0x32, 0x50,0x80,
        0x32, 0x57,0x60,
        0x32, 0x9B,0x01,
        0x32, 0xA1,0x00,
        0x32, 0xA2,0xDA,
        0x32, 0xA3,0x01,
        0x32, 0xA4,0x6E,
        0x32, 0xA5,0x01,
        0x32, 0xA6,0x39,
        0x32, 0xA7,0x01,
        0x32, 0xA8,0xEB,
        0x32, 0x10,0x10,
        0x32, 0x11,0x10,
        0x32, 0x12,0x10,
        0x32, 0x13,0x10,
        0x32, 0x14,0x0C,
        0x32, 0x15,0x0C,
        0x32, 0x16,0x0C,
        0x32, 0x17,0x0C,
        0x32, 0x18,0x0C,
        0x32, 0x19,0x0C,
        0x32, 0x1A,0x0C,
        0x32, 0x1B,0x0C,
        0x32, 0x1C,0x09,
        0x32, 0x1D,0x09,
        0x32, 0x1E,0x09,
        0x32, 0x1F,0x09,
        0x32, 0xB0,0x55,
        0x32, 0xB1,0x7D,
        0x32, 0xB2,0x7D,
        0x32, 0xB3,0x55,
        0x32, 0x70,0x0A,
        0x32, 0x71,0x14,
        0x32, 0x72,0x1E,
        0x32, 0x73,0x32,
        0x32, 0x74,0x45,
        0x32, 0x75,0x56,
        0x32, 0x76,0x75,
        0x32, 0x77,0x90,
        0x32, 0x78,0xA7,
        0x32, 0x79,0xBA,
        0x32, 0x7A,0xD9,
        0x32, 0x7B,0xEB,
        0x32, 0x7C,0xF5,
        0x32, 0x7D,0xF9,
        0x32, 0x7E,0xFA,
        0x33, 0x02,0x00,
        0x33, 0x03,0x4D,
        0x33, 0x04,0x00,
        0x33, 0x05,0x96,
        0x33, 0x06,0x00,
        0x33, 0x07,0x1D,
        0x33, 0x08,0x07,
        0x33, 0x09,0xCF,
        0x33, 0x0A,0x06,
        0x33, 0x0B,0x92,
        0x33, 0x0C,0x01,
        0x33, 0x0D,0x9F,
        0x33, 0x0E,0x00,
        0x33, 0x0F,0xC4,
        0x33, 0x10,0x07,
        0x33, 0x11,0x64,
        0x33, 0x12,0x07,
        0x33, 0x13,0xD9,
        0x32, 0xF6,0x0F,
        0x32, 0xF9,0x63,
        0x32, 0xFA,0x36,
        0x33, 0x26,0x02,
        0x33, 0x27,0x04,
        0x33, 0x28,0x04,
        0x33, 0x29,0x02,
        0x33, 0x2A,0x02,
        0x33, 0x2B,0x1D,
        0x33, 0x2C,0x1D,
        0x33, 0x2D,0x04,
        0x33, 0x2E,0x1E,
        0x33, 0x2F,0x1F,
        0x33, 0x25,0x5F,
        0x33, 0x30,0x00,
        0x33, 0x31,0x0A,
        0x33, 0x32,0x60,
        0x33, 0x38,0x18,
        0x33, 0x39,0xA5,
        0x33, 0x3A,0x5A,
        0x33, 0x3F,0x17,
        0x33, 0x60,0x10,
        0x33, 0x61,0x18,
        0x33, 0x62,0x2F,
        0x33, 0x63,0x37,
        0x33, 0x64,0x80,
        0x33, 0x65,0x80,
        0x33, 0x66,0x68,
        0x33, 0x67,0x50,
        0x33, 0x68,0x60,
        0x33, 0x69,0x50,
        0x33, 0x6A,0x40,
        0x33, 0x6B,0x30,
        0x33, 0x6C,0x00,
        0x33, 0x6D,0x20,
        0x33, 0x6E,0x1C,
        0x33, 0x6F,0x18,
        0x33, 0x70,0x10,
        0x33, 0x71,0x30,
        0x33, 0x72,0x3F,
        0x33, 0x73,0x3F,
        0x33, 0x74,0x3F,
        0x33, 0x75,0x03,
        0x33, 0x76,0x03,
        0x33, 0x77,0x03,
        0x33, 0x78,0x03,
        0x33, 0x79,0x06,
        0x33, 0x7A,0x10,
        0x33, 0x7B,0x28,
        0x33, 0x7C,0x28,
        0x33, 0x7D,0x10,
        0x33, 0x7E,0x14,
        0x33, 0x7F,0x18,
        0x33, 0x80,0x20,
        0x33, 0x81,0x60,
        0x33, 0x82,0x60,
        0x33, 0x83,0x7F,
        0x33, 0x84,0x7F,
        0x33, 0x85,0xFF,
        0x33, 0x8A,0x34,
        0x33, 0x8B,0x7F,
        0x33, 0x8C,0x10,
        0x33, 0x8D,0x23,
        0x33, 0x8E,0x7F,
        0x33, 0x8F,0x14,
        0x30, 0x12,0x02,
        0x30, 0x13,0x1E,
        0x32, 0xBF,0x60,
        0x32, 0xC0,0x5A,
        0x32, 0xC1,0x5A,
        0x32, 0xC2,0x5A,
        0x32, 0xC3,0x00,
        0x32, 0xC4,0x20,
        0x32, 0xC5,0x20,
        0x32, 0xC6,0x20,
        0x32, 0xC7,0x00,
        0x32, 0xC8,0xDF,
        0x32, 0xC9,0x5A,
        0x32, 0xCA,0x7A,
        0x32, 0xCB,0x7A,
        0x32, 0xCC,0x7A,
        0x32, 0xCD,0x7A,
        0x32, 0xDB,0x7B,
        0x32, 0x00,0x3E,
        0x32, 0x01,0x0F,
        0x30, 0x28,0x24,
        0x30, 0x29,0x20,
        0x30, 0x2A,0x04,
        0x30, 0x22,0x25,
        0x30, 0x23,0x24,
        0x30, 0x02,0x00,
        0x30, 0x03,0x04,
        0x30, 0x04,0x00,
        0x30, 0x05,0x04,
        0x30, 0x06,0x05,
        0x30, 0x07,0x03,
        0x30, 0x08,0x02,
        0x30, 0x09,0xD3,
        0x30, 0x0A,0x06,
        0x30, 0x0B,0x7C,
        0x30, 0x0C,0x02,
        0x30, 0x0D,0xE6,
        0x30, 0x0E,0x05,
        0x30, 0x0F,0x00,
        0x30, 0x10,0x02,
        0x30, 0x11,0xD0,
        0x32, 0xB8,0x3F,
        0x32, 0xB9,0x31,
        0x32, 0xBB,0x87,
        0x32, 0xBC,0x38,
        0x32, 0xBD,0x3C,
        0x32, 0xBE,0x34,
        0x30, 0x60,0x01,
	END_FLAG,END_FLAG,END_FLAG,

};


static  const unsigned char INIT_TAB_640_360[] = {
	0x31, 0x09,0x04,
        0x30, 0x69,0x03,
        0x30, 0x6A,0x05,
        0x30, 0x40,0x04,
        0x30, 0x41,0x02,
        0x30, 0x42,0xFF,
        0x30, 0x43,0x08,
        0x30, 0x52,0xE0,
        0x30, 0x5F,0x66,
        0x31, 0x06,0x03,
        0x31, 0x10,0x22,
        0x31, 0x11,0x54,
        0x31, 0x12,0x22,
        0x31, 0x13,0x55,
        0x31, 0x14,0x05,
        0x32, 0xF0,0x0 ,
        0x32, 0x90,0x01,
        0x32, 0x91,0x88,
        0x32, 0x96,0x01,
        0x32, 0x97,0xC1,
        0x32, 0x50,0x80,
        0x32, 0x57,0x60,
        0x32, 0x9B,0x01,
        0x32, 0xA1,0x00,
        0x32, 0xA2,0xDA,
        0x32, 0xA3,0x01,
        0x32, 0xA4,0x6E,
        0x32, 0xA5,0x01,
        0x32, 0xA6,0x39,
        0x32, 0xA7,0x01,
        0x32, 0xA8,0xEB,
        0x32, 0x10,0x10,
        0x32, 0x11,0x10,
        0x32, 0x12,0x10,
        0x32, 0x13,0x10,
        0x32, 0x14,0x0C,
        0x32, 0x15,0x0C,
        0x32, 0x16,0x0C,
        0x32, 0x17,0x0C,
        0x32, 0x18,0x0C,
        0x32, 0x19,0x0C,
        0x32, 0x1A,0x0C,
        0x32, 0x1B,0x0C,
        0x32, 0x1C,0x09,
        0x32, 0x1D,0x09,
        0x32, 0x1E,0x09,
        0x32, 0x1F,0x09,
        0x32, 0xB0,0x55,
        0x32, 0xB1,0x7D,
        0x32, 0xB2,0x7D,
        0x32, 0xB3,0x55,
        0x32, 0x70,0x0A,
        0x32, 0x71,0x14,
        0x32, 0x72,0x1E,
        0x32, 0x73,0x32,
        0x32, 0x74,0x45,
        0x32, 0x75,0x56,
        0x32, 0x76,0x75,
        0x32, 0x77,0x90,
        0x32, 0x78,0xA7,
        0x32, 0x79,0xBA,
        0x32, 0x7A,0xD9,
        0x32, 0x7B,0xEB,
        0x32, 0x7C,0xF5,
        0x32, 0x7D,0xF9,
        0x32, 0x7E,0xFA,
        0x33, 0x02,0x00,
        0x33, 0x03,0x4D,
        0x33, 0x04,0x00,
        0x33, 0x05,0x96,
        0x33, 0x06,0x00,
        0x33, 0x07,0x1D,
        0x33, 0x08,0x07,
        0x33, 0x09,0xCF,
        0x33, 0x0A,0x06,
        0x33, 0x0B,0x92,
        0x33, 0x0C,0x01,
        0x33, 0x0D,0x9F,
        0x33, 0x0E,0x00,
        0x33, 0x0F,0xC4,
        0x33, 0x10,0x07,
        0x33, 0x11,0x64,
        0x33, 0x12,0x07,
        0x33, 0x13,0xD9,
        0x32, 0xF6,0x0F,
        0x32, 0xF9,0x63,
        0x32, 0xFA,0x36,
        0x33, 0x26,0x02,
        0x33, 0x27,0x04,
        0x33, 0x28,0x04,
        0x33, 0x29,0x02,
        0x33, 0x2A,0x02,
        0x33, 0x2B,0x1D,
        0x33, 0x2C,0x1D,
        0x33, 0x2D,0x04,
        0x33, 0x2E,0x1E,
        0x33, 0x2F,0x1F,
        0x33, 0x25,0x5F,
        0x33, 0x30,0x00,
        0x33, 0x31,0x0A,
        0x33, 0x32,0x60,
        0x33, 0x38,0x18,
        0x33, 0x39,0xA5,
        0x33, 0x3A,0x5A,
        0x33, 0x3F,0x17,
        0x33, 0x60,0x10,
        0x33, 0x61,0x18,
        0x33, 0x62,0x2F,
        0x33, 0x63,0x37,
        0x33, 0x64,0x80,
        0x33, 0x65,0x80,
        0x33, 0x66,0x68,
        0x33, 0x67,0x50,
        0x33, 0x68,0x60,
        0x33, 0x69,0x50,
        0x33, 0x6A,0x40,
        0x33, 0x6B,0x30,
        0x33, 0x6C,0x00,
        0x33, 0x6D,0x20,
        0x33, 0x6E,0x1C,
        0x33, 0x6F,0x18,
        0x33, 0x70,0x10,
        0x33, 0x71,0x30,
        0x33, 0x72,0x3F,
        0x33, 0x73,0x3F,
        0x33, 0x74,0x3F,
        0x33, 0x75,0x03,
        0x33, 0x76,0x03,
        0x33, 0x77,0x03,
        0x33, 0x78,0x03,
        0x33, 0x79,0x06,
        0x33, 0x7A,0x10,
        0x33, 0x7B,0x28,
        0x33, 0x7C,0x28,
        0x33, 0x7D,0x10,
        0x33, 0x7E,0x14,
        0x33, 0x7F,0x18,
        0x33, 0x80,0x20,
        0x33, 0x81,0x60,
        0x33, 0x82,0x60,
        0x33, 0x83,0x7F,
        0x33, 0x84,0x7F,
        0x33, 0x85,0xFF,
        0x33, 0x8A,0x34,
        0x33, 0x8B,0x7F,
        0x33, 0x8C,0x10,
        0x33, 0x8D,0x23,
        0x33, 0x8E,0x7F,
        0x33, 0x8F,0x14,
        0x30, 0x12,0x02,
        0x30, 0x13,0x1E,
        0x32, 0xBF,0x40,
        0x32, 0xC0,0x7E,
        0x32, 0xC1,0x7E,
        0x32, 0xC2,0x7E,
        0x32, 0xC3,0x00,
        0x32, 0xC4,0x20,
        0x32, 0xC5,0x20,
        0x32, 0xC6,0x20,
        0x32, 0xC7,0x00,
        0x32, 0xC8,0x6C,
        0x32, 0xC9,0x7E,
        0x32, 0xCA,0x9E,
        0x32, 0xCB,0x9E,
        0x32, 0xCC,0x9E,
        0x32, 0xCD,0x9E,
        0x32, 0xDB,0x6B,
        0x32, 0xD0,0x01,
        0x32, 0x00,0x3E,
        0x32, 0x01,0x0F,
        0x30, 0x28,0x08,
        0x30, 0x29,0x02,
        0x30, 0x2A,0x00,
        0x30, 0x22,0x25,
        0x30, 0x23,0x24,
        0x30, 0x02,0x00,
        0x30, 0x03,0x04,
        0x30, 0x04,0x00,
        0x30, 0x05,0x04,
        0x30, 0x06,0x05,
        0x30, 0x07,0x03,
        0x30, 0x08,0x02,
        0x30, 0x09,0xD3,
        0x30, 0x0A,0x06,
        0x30, 0x0B,0x7C,
        0x30, 0x0C,0x04,
        0x30, 0x0D,0x3C,
        0x30, 0x0E,0x02,
        0x30, 0x0F,0x80,
        0x30, 0x10,0x01,
        0x30, 0x11,0x68,
        0x32, 0xB8,0x36,
        0x32, 0xB9,0x2A,
        0x32, 0xBB,0x87,
        0x32, 0xBC,0x30,
        0x32, 0xBD,0x33,
        0x32, 0xBE,0x2D,
        0x32, 0x01,0x3F,
        0x30, 0x21,0x06,
        0x30, 0x60,0x01,
	END_FLAG,END_FLAG,END_FLAG,

};


#endif /*GPT_NT99141_H_*/
