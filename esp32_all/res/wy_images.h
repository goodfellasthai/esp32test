#ifndef IMAGES_H
#define IMAGES_H


#define lowbatt_width 	 16
#define lowbatt_height 	 30
const unsigned char lowbatt_bmp [] PROGMEM ={
0x00,0x01,0xe0,0x07,0xe8,0x17,0x54,0x2a,0x02,0x60,0x02,0x40,0x02,0x40,0x02,0x40,
0x02,0x40,0x02,0x60,0x02,0x40,0x02,0x41,0x82,0x60,0x82,0x40,0x42,0x41,0x82,0x62,
0x02,0x41,0x82,0x40,0x02,0x40,0x02,0x40,0x02,0x40,0x02,0x60,0x02,0x40,0x22,0x60,
0xae,0x5f,0x6a,0x49,0xaa,0x56,0xaa,0x68,0xfc,0x3f,0x00,0x00,0x00,0x00,0x00,0x00};

#define wy_width 	 128
#define wy_height 	 64
const unsigned char wy_bits [] PROGMEM ={
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xf0,0xff,0x7f,0x00,0x00,0x30,0x00,0x00,0x98,0x0f,0x9d,0xe7,0xc1,0x8f,0xf7,0x01,
0xf0,0xff,0xff,0x00,0x00,0x30,0x00,0x00,0x38,0x8f,0x7e,0xdf,0x23,0x9c,0xcf,0x01,
0xe0,0xff,0xff,0x00,0x00,0x78,0x00,0x00,0x1c,0x1f,0x1c,0xcf,0x03,0xbe,0xe7,0x03,
0xe0,0xff,0xff,0x00,0x00,0x78,0x00,0x00,0x1c,0x7e,0x3e,0xcf,0xe3,0x9f,0xc7,0x01,
0xc0,0xff,0xff,0x01,0x00,0xfc,0x00,0x00,0x0e,0x3c,0x1c,0xcf,0xf3,0xbc,0xe7,0x03,
0xc0,0xff,0xff,0x01,0x00,0xfc,0x00,0x00,0x0e,0x1c,0x3e,0xcf,0x7b,0x9e,0xc7,0x01,
0xc0,0xff,0xff,0x03,0x00,0xfe,0x01,0x00,0x06,0x18,0x1c,0x8f,0xf3,0xbc,0xe7,0x03,
0x80,0xff,0xff,0x03,0x00,0xff,0x03,0x00,0x07,0x08,0x3e,0xcf,0x73,0x9e,0xc7,0x01,
0x80,0xff,0xff,0x03,0x00,0xff,0x03,0x00,0x07,0x06,0x1c,0xcf,0xe3,0x3f,0xe7,0x03,
0x00,0xff,0xff,0x07,0x00,0xff,0x03,0x80,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xff,0xff,0x07,0x80,0xff,0x07,0x80,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xfe,0xff,0x0f,0xc0,0xff,0x0f,0x80,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xfe,0xff,0x0f,0xc0,0xff,0x0f,0xc0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xfe,0xff,0x0f,0xe0,0xff,0x1f,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xfc,0xff,0x1f,0xe0,0xff,0x1f,0xe0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xfc,0xff,0x1f,0xf0,0xff,0x3f,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xf8,0xff,0x3f,0xf0,0xff,0x3f,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xf8,0xff,0x3f,0xf8,0xff,0x7f,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xf8,0xff,0x3f,0xfc,0xff,0xff,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xf0,0xff,0x7f,0xfc,0xff,0xff,0x38,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xf0,0xff,0x7f,0xfe,0xff,0xff,0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xe0,0xff,0xff,0xee,0xff,0xff,0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xe0,0xff,0xff,0xe7,0xff,0xff,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xc0,0xff,0xff,0xc7,0xff,0xff,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xc0,0xff,0xff,0xc3,0xff,0xff,0x07,0x00,0x03,0x00,0x00,0x00,0x40,0x00,0x00,
0x00,0xc0,0xff,0xff,0x81,0xff,0xff,0x07,0x00,0x03,0x08,0x00,0x00,0x04,0x00,0x00,
0x00,0x80,0xff,0xff,0x81,0xff,0xff,0x07,0x80,0x26,0xdd,0x6b,0xf3,0x7e,0xcf,0x01,
0x00,0x80,0xff,0xff,0x01,0xff,0xff,0x03,0xc0,0xb2,0x65,0xda,0x96,0xa6,0x69,0x03,
0x00,0x00,0xff,0xff,0x00,0xfe,0xff,0x03,0xc0,0x97,0x2c,0x4a,0x9a,0xa4,0x28,0x01,
0x00,0x00,0xff,0x7f,0x00,0xfe,0xff,0x01,0x20,0x94,0x25,0x4b,0xca,0xa2,0x2d,0x01,
0x00,0x00,0xff,0x7f,0x00,0xfc,0xff,0x01,0x20,0xf4,0xec,0x49,0x7a,0x2e,0x27,0x01,
0x00,0x00,0xfe,0x3f,0x00,0xfc,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xfe,0x3f,0x00,0xf8,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xfc,0x1f,0x00,0xf8,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xfc,0x1f,0x00,0xf0,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xfc,0x0f,0x00,0xf0,0x7f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xf8,0x0f,0x00,0xe0,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xf8,0x07,0x00,0xc0,0x3f,0x00,0x80,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xf0,0x03,0x00,0xc0,0x1f,0x00,0xc0,0x00,0x03,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xf0,0x03,0x00,0xc0,0x1f,0x00,0xc0,0xcc,0x77,0xfe,0x0e,0x00,0x00,0x00,
0x00,0x00,0xf0,0x01,0x00,0x80,0x0f,0x00,0x80,0x69,0xc9,0x92,0x02,0x00,0x00,0x00,
0x00,0x00,0xe0,0x01,0x00,0x00,0x0f,0x00,0x00,0x2b,0x79,0x9b,0x0e,0x00,0x00,0x00,
0x00,0x00,0xe0,0x00,0x00,0x00,0x07,0x00,0x20,0x39,0x09,0xd1,0x08,0x00,0x00,0x00,
0x00,0x00,0xc0,0x00,0x00,0x00,0x06,0x00,0xe0,0x19,0x7b,0x49,0x07,0x00,0x00,0x00,
0x00,0x00,0x40,0x00,0x00,0x00,0x06,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

#define logo_width 128
#define logo_height 53
const static char logo_bits[] = {

  0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x01, 0xF0, 0x03,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xC0, 0x07, 0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x1F, 0xF0, 0x1F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xE0, 0x1F, 0xF0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xE0, 0x3F, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1F, 0xF8, 0x3F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xE0, 0x1F, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xF0, 0x1F, 0xF8, 0x1F, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x1F, 0xFC, 0x1F,
  0x80, 0xFF, 0x8F, 0x7F, 0xC0, 0xFF, 0x7F, 0xC0, 0xFF, 0x03, 0xC0, 0x3F,
  0xF0, 0x1F, 0xFC, 0x1F, 0xE0, 0xFF, 0x87, 0x3F, 0xC0, 0xFF, 0x7F, 0xF0,
  0xFF, 0x01, 0xF8, 0xFF, 0xF0, 0x0F, 0xFC, 0x1F, 0xE0, 0xFF, 0x87, 0x3F,
  0xE0, 0xFF, 0x7F, 0xF8, 0xFF, 0x01, 0xFE, 0xFF, 0xF8, 0x0F, 0xFE, 0x0F,
  0xF0, 0xFF, 0x87, 0x3F, 0xE0, 0xFF, 0x7F, 0xFC, 0xFF, 0x01, 0xFF, 0xFF,
  0xF8, 0xFF, 0xFF, 0x0F, 0xF0, 0xFF, 0xC7, 0x3F, 0xE0, 0xFF, 0x3F, 0xFC,
  0xFF, 0x81, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF, 0x0F, 0xF0, 0xFF, 0xC3, 0x1F,
  0xE0, 0xFF, 0x3F, 0xFC, 0xFF, 0xC0, 0xFF, 0x7F, 0xFC, 0xFF, 0xFF, 0x0F,
  0xF8, 0xFF, 0xC3, 0x1F, 0xE0, 0xFF, 0x3F, 0xFC, 0xFF, 0xE0, 0xFF, 0x7F,
  0xFC, 0xFF, 0xFF, 0x07, 0xF8, 0x03, 0xC0, 0x1F, 0x00, 0xFE, 0x01, 0xFE,
  0x00, 0xF0, 0x3F, 0x70, 0xFC, 0xFF, 0xFF, 0x07, 0xF8, 0x03, 0xE0, 0x0F,
  0x00, 0xFE, 0x00, 0xFE, 0x00, 0xF0, 0x1F, 0x60, 0xFC, 0xFF, 0xFF, 0x07,
  0xF8, 0x03, 0xE0, 0x0F, 0x00, 0xFE, 0x00, 0xFE, 0x00, 0xF0, 0x07, 0x20,
  0xFE, 0xFF, 0xFF, 0x07, 0xFC, 0xFF, 0xE1, 0x0F, 0x00, 0xFF, 0x00, 0xFF,
  0x7F, 0xF8, 0x07, 0x00, 0xFE, 0x83, 0xFF, 0x03, 0xFC, 0xFF, 0xF1, 0x0F,
  0x00, 0x7F, 0x00, 0xFF, 0x7F, 0xF8, 0x03, 0x00, 0xFE, 0x83, 0xFF, 0x03,
  0xFC, 0xFF, 0xF0, 0x07, 0x00, 0x7F, 0x00, 0xFF, 0x7F, 0xFC, 0x03, 0x00,
  0xFE, 0x81, 0xFF, 0x03, 0xFC, 0xFF, 0xF0, 0x07, 0x00, 0x7F, 0x00, 0xFF,
  0x3F, 0xFC, 0x03, 0x00, 0xFF, 0xC1, 0xFF, 0x01, 0xFE, 0xFF, 0xF0, 0x07,
  0x80, 0x3F, 0x80, 0xFF, 0x3F, 0xFC, 0x03, 0x00, 0xFF, 0xC1, 0xFF, 0x01,
  0xFE, 0xFF, 0xF8, 0x07, 0x80, 0x3F, 0x80, 0xFF, 0x3F, 0xFC, 0x03, 0x10,
  0xFF, 0xC1, 0xFF, 0x01, 0xFE, 0x00, 0xF8, 0x03, 0x80, 0x3F, 0x80, 0x3F,
  0x00, 0xFC, 0x03, 0x0C, 0xFF, 0xC0, 0xFF, 0x01, 0xFF, 0x00, 0xF8, 0x03,
  0xC0, 0x3F, 0x80, 0x3F, 0x00, 0xFC, 0x07, 0x0E, 0xFF, 0xE0, 0xFF, 0x00,
  0x7F, 0x00, 0xF8, 0x03, 0xC0, 0x1F, 0xC0, 0x3F, 0x00, 0xFC, 0xFF, 0x0F,
  0xFF, 0xE0, 0xFF, 0x00, 0xFF, 0x7F, 0xFC, 0xFF, 0xC1, 0x1F, 0xC0, 0xFF,
  0x1F, 0xFC, 0xFF, 0x0F, 0x7F, 0xE0, 0xFF, 0x00, 0xFF, 0x3F, 0xFC, 0xFF,
  0xC1, 0x1F, 0xC0, 0xFF, 0x0F, 0xF8, 0xFF, 0x07, 0x7E, 0xE0, 0xFF, 0x80,
  0xFF, 0x3F, 0xFC, 0xFF, 0xE0, 0x1F, 0xC0, 0xFF, 0x0F, 0xF8, 0xFF, 0x07,
  0x7C, 0xF0, 0x7F, 0x80, 0xFF, 0x3F, 0xFE, 0xFF, 0xE0, 0x0F, 0xE0, 0xFF,
  0x0F, 0xF0, 0xFF, 0x07, 0xF8, 0xF0, 0x7F, 0x80, 0xFF, 0x1F, 0xFE, 0xFF,
  0xE0, 0x0F, 0xE0, 0xFF, 0x0F, 0xE0, 0xFF, 0x07, 0xE0, 0xF0, 0x7F, 0x80,
  0xFF, 0x1F, 0xFE, 0xFF, 0xE0, 0x0F, 0xE0, 0xFF, 0x07, 0x80, 0xFF, 0x03,
  0x00, 0xF0, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x3C, 0x00, 0x00, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xF8, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0x8F, 0xF7, 0xFF, 0x7C,
  0xBC, 0xC7, 0xF3, 0xFF, 0xFC, 0xBC, 0x07, 0x00, 0x00, 0xFC, 0x1F, 0x00,
  0x8F, 0x73, 0xFF, 0xFE, 0xBE, 0xC7, 0xFB, 0xFF, 0xFE, 0xBD, 0x03, 0x00,
  0x00, 0xFC, 0x1F, 0x80, 0x8F, 0x73, 0xFF, 0xEF, 0xFE, 0xE7, 0xFB, 0x77,
  0xEF, 0xFD, 0x03, 0x00, 0x00, 0xFC, 0x1F, 0x80, 0xDF, 0x7B, 0x9C, 0xE7,
  0xFE, 0xF7, 0xE3, 0x71, 0xE7, 0xFD, 0x03, 0x00, 0x00, 0xFC, 0x0F, 0xC0,
  0xDF, 0x79, 0x9E, 0xE3, 0xFE, 0xF3, 0xE3, 0x78, 0xE7, 0xFF, 0x03, 0x00,
  0x00, 0xFC, 0x0F, 0xE0, 0xDF, 0x39, 0x8E, 0xF3, 0xFF, 0xFB, 0xE7, 0xF8,
  0xE7, 0xFE, 0x01, 0x00, 0x00, 0xFC, 0x0F, 0xE0, 0xDF, 0x3F, 0x8E, 0x7F,
  0xFF, 0xFF, 0xE7, 0x38, 0x7F, 0xEE, 0x01, 0x00, 0x00, 0xF8, 0x0F, 0x70,
  0xDC, 0x1F, 0x0E, 0x3F, 0xFF, 0x9D, 0xF7, 0x38, 0x3F, 0xEF, 0x01, 0x00,
  0x00, 0xF8, 0x0F, 0x00, 0x00, 0x03, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
  0x08, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x0F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

const char activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const char inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

#endif
