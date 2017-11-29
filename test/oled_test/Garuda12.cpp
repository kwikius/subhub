
#include "gfxfont.h"

constexpr uint8_t Garuda12pt7bBitmaps[] = {
  0xFF, 0xFF, 0xFF, 0xC3, 0xC0, 0xCF, 0x3C, 0xF3, 0xCF, 0x30, 0x0C, 0x30,
  0x61, 0x83, 0x1C, 0x18, 0xCF, 0xFF, 0xFF, 0xFC, 0x61, 0x83, 0x0C, 0x18,
  0xC0, 0xC6, 0x3F, 0xFF, 0xFF, 0xF3, 0x0C, 0x18, 0xC0, 0xC6, 0x0E, 0x30,
  0x61, 0x80, 0x06, 0x03, 0xF1, 0xFF, 0x33, 0x7C, 0x67, 0x8C, 0xB1, 0x87,
  0x30, 0x7E, 0x07, 0xF0, 0x1F, 0x03, 0x70, 0x66, 0x0C, 0xF1, 0x9F, 0x33,
  0x76, 0xCF, 0xF8, 0x7E, 0x03, 0x00, 0x3C, 0x06, 0x0F, 0xC0, 0x83, 0x9C,
  0x30, 0x61, 0x84, 0x0C, 0x31, 0x81, 0x8E, 0x20, 0x1F, 0x8C, 0x01, 0xE1,
  0x00, 0x00, 0x67, 0x80, 0x09, 0xF8, 0x03, 0x73, 0x80, 0xCC, 0x30, 0x19,
  0x86, 0x06, 0x30, 0xC0, 0xC6, 0x18, 0x30, 0xE7, 0x06, 0x0F, 0xC1, 0x80,
  0xF0, 0x0F, 0x00, 0x7E, 0x03, 0x1C, 0x0C, 0x30, 0x38, 0xC0, 0x76, 0x01,
  0xF8, 0x07, 0x80, 0x3F, 0x01, 0xCE, 0x6E, 0x1D, 0xB0, 0x3C, 0xC0, 0xF3,
  0x01, 0xC6, 0x1F, 0x9F, 0xE7, 0x1E, 0x08, 0xFF, 0xF0, 0x08, 0xCC, 0x66,
  0x31, 0x98, 0xC6, 0x31, 0x8C, 0x63, 0x0C, 0x63, 0x0C, 0x61, 0x84, 0x86,
  0x18, 0xC3, 0x18, 0xC3, 0x18, 0xC6, 0x31, 0x8C, 0x66, 0x31, 0x98, 0xCC,
  0x40, 0x18, 0x18, 0xFF, 0x7E, 0x3C, 0x66, 0x24, 0x0C, 0x03, 0x00, 0xC0,
  0x30, 0xFF, 0xFF, 0xF0, 0xC0, 0x30, 0x0C, 0x03, 0x00, 0x6D, 0xBC, 0xFF,
  0xF0, 0xF0, 0x0C, 0x31, 0x86, 0x18, 0x63, 0x8C, 0x30, 0xC7, 0x18, 0x61,
  0x86, 0x30, 0xC0, 0x1F, 0x07, 0xF1, 0xC7, 0x30, 0x6E, 0x0F, 0x80, 0xF0,
  0x1E, 0x03, 0xC0, 0x78, 0x0F, 0x01, 0xE0, 0x3C, 0x0E, 0xC1, 0x9C, 0x71,
  0xFC, 0x1F, 0x00, 0x0C, 0x71, 0xDF, 0xEF, 0x30, 0xC3, 0x0C, 0x30, 0xC3,
  0x0C, 0x30, 0xC3, 0x0C, 0x1F, 0x0F, 0xF9, 0x83, 0x60, 0x3C, 0x06, 0x00,
  0xC0, 0x38, 0x06, 0x01, 0x80, 0xE0, 0x38, 0x0E, 0x03, 0x80, 0xE0, 0x18,
  0x07, 0xFF, 0xFF, 0xE0, 0x1E, 0x07, 0xF1, 0xC7, 0x70, 0x6C, 0x0C, 0x01,
  0x80, 0x60, 0x78, 0x0F, 0x80, 0x18, 0x01, 0x80, 0x3C, 0x07, 0xC1, 0xD8,
  0x73, 0xFC, 0x1F, 0x00, 0x01, 0x80, 0x30, 0x0E, 0x03, 0xC0, 0x78, 0x1B,
  0x07, 0x61, 0xCC, 0x31, 0x8C, 0x33, 0x86, 0x7F, 0xFF, 0xFE, 0x03, 0x00,
  0x60, 0x0C, 0x01, 0x80, 0x7F, 0xCF, 0xF9, 0x80, 0x30, 0x06, 0x00, 0xDE,
  0x37, 0xE7, 0x0E, 0xC0, 0xE0, 0x0C, 0x01, 0x80, 0x3C, 0x07, 0x81, 0xD8,
  0x73, 0xFC, 0x1F, 0x00, 0x0F, 0x07, 0xF9, 0xC3, 0x30, 0x36, 0x01, 0x80,
  0x33, 0xC6, 0xFE, 0xF0, 0xDC, 0x0F, 0x01, 0xE0, 0x3C, 0x06, 0xC1, 0xDC,
  0x31, 0xFE, 0x1F, 0x00, 0xFF, 0xFF, 0xF0, 0x18, 0x0E, 0x03, 0x01, 0x80,
  0x60, 0x30, 0x0C, 0x07, 0x01, 0x80, 0x60, 0x18, 0x0C, 0x03, 0x00, 0xC0,
  0x30, 0x00, 0x1F, 0x07, 0xF1, 0xC7, 0x30, 0x66, 0x0C, 0xC1, 0x9C, 0x71,
  0xFC, 0x3F, 0x8E, 0x3B, 0x01, 0xE0, 0x3C, 0x07, 0x80, 0xD8, 0x33, 0xFE,
  0x1F, 0x00, 0x1F, 0x0F, 0xF1, 0xC7, 0x70, 0x6C, 0x07, 0x80, 0xF0, 0x1E,
  0x07, 0x71, 0xEF, 0xEC, 0x79, 0x80, 0x30, 0x0D, 0x81, 0x98, 0x73, 0xFC,
  0x1E, 0x00, 0xF0, 0x00, 0x03, 0xC0, 0x6C, 0x00, 0x00, 0x00, 0x36, 0xDE,
  0x00, 0x10, 0x0F, 0x03, 0xC1, 0xF0, 0x7C, 0x0E, 0x00, 0xF8, 0x01, 0xF0,
  0x07, 0xC0, 0x1F, 0x00, 0x30, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
  0x0F, 0xFF, 0xFF, 0xF0, 0xC0, 0x0F, 0x00, 0x7C, 0x00, 0xF8, 0x03, 0xE0,
  0x07, 0x01, 0xF0, 0x78, 0x3E, 0x0F, 0x80, 0xC0, 0x00, 0x1F, 0x0F, 0xF9,
  0x87, 0x70, 0x7C, 0x06, 0x00, 0xC0, 0x38, 0x0E, 0x03, 0x80, 0xE0, 0x38,
  0x06, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x60, 0x0C, 0x00, 0x00, 0xFF, 0x00,
  0x0F, 0xFF, 0x00, 0xF8, 0x1F, 0x07, 0x00, 0x1E, 0x38, 0x78, 0xB9, 0xC3,
  0xF6, 0x76, 0x1C, 0x78, 0xD8, 0xC0, 0xE3, 0xC3, 0x03, 0x8F, 0x18, 0x0E,
  0x3C, 0x60, 0x38, 0xF1, 0x80, 0xC7, 0xC6, 0x07, 0x1B, 0x18, 0x1C, 0x6E,
  0x31, 0xF3, 0x18, 0xFD, 0xF8, 0x71, 0xE3, 0xC0, 0xE0, 0x00, 0x31, 0xC0,
  0x03, 0x83, 0xC0, 0x3C, 0x07, 0xFF, 0xE0, 0x03, 0xFC, 0x00, 0x01, 0x80,
  0x03, 0xC0, 0x03, 0xC0, 0x07, 0xC0, 0x06, 0x60, 0x06, 0x60, 0x0C, 0x30,
  0x0C, 0x30, 0x1C, 0x30, 0x18, 0x18, 0x1F, 0xF8, 0x3F, 0xFC, 0x30, 0x0C,
  0x70, 0x0C, 0x60, 0x06, 0x60, 0x06, 0xC0, 0x07, 0xFF, 0x0F, 0xFC, 0xC0,
  0xEC, 0x06, 0xC0, 0x6C, 0x06, 0xC0, 0xCF, 0xF8, 0xFF, 0xCC, 0x06, 0xC0,
  0x3C, 0x03, 0xC0, 0x3C, 0x03, 0xC0, 0x6F, 0xFE, 0xFF, 0x80, 0x07, 0xE0,
  0x3F, 0xE0, 0xE0, 0xE3, 0x80, 0xE6, 0x00, 0xD8, 0x00, 0x30, 0x00, 0x60,
  0x00, 0xC0, 0x01, 0x80, 0x03, 0x00, 0x06, 0x00, 0x76, 0x00, 0xEE, 0x03,
  0x8E, 0x0E, 0x0F, 0xF8, 0x07, 0xE0, 0xFF, 0x83, 0xFF, 0x8C, 0x07, 0x30,
  0x06, 0xC0, 0x1B, 0x00, 0x3C, 0x00, 0xF0, 0x03, 0xC0, 0x0F, 0x00, 0x3C,
  0x00, 0xF0, 0x03, 0xC0, 0x1B, 0x00, 0xEC, 0x07, 0x3F, 0xF8, 0xFF, 0x80,
  0xFF, 0xFF, 0xFF, 0xC0, 0x0C, 0x00, 0xC0, 0x0C, 0x00, 0xC0, 0x0F, 0xFE,
  0xFF, 0xEC, 0x00, 0xC0, 0x0C, 0x00, 0xC0, 0x0C, 0x00, 0xC0, 0x0F, 0xFF,
  0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0x00, 0x60, 0x0C, 0x01, 0x80, 0x30, 0x07,
  0xFE, 0xFF, 0xD8, 0x03, 0x00, 0x60, 0x0C, 0x01, 0x80, 0x30, 0x06, 0x00,
  0xC0, 0x00, 0x07, 0xF0, 0x1F, 0xFC, 0x3C, 0x1E, 0x70, 0x06, 0x60, 0x03,
  0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x00, 0xC0, 0x7F, 0xC0, 0x7F, 0xC0, 0x03,
  0xE0, 0x03, 0x60, 0x03, 0x70, 0x03, 0x3C, 0x0F, 0x1F, 0xFC, 0x07, 0xF0,
  0xC0, 0x1E, 0x00, 0xF0, 0x07, 0x80, 0x3C, 0x01, 0xE0, 0x0F, 0x00, 0x7F,
  0xFF, 0xFF, 0xFE, 0x00, 0xF0, 0x07, 0x80, 0x3C, 0x01, 0xE0, 0x0F, 0x00,
  0x78, 0x03, 0xC0, 0x18, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x01, 0x80, 0xC0,
  0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x80, 0xC0, 0x60, 0x3C, 0x1E,
  0x0F, 0x8E, 0xFE, 0x3E, 0x00, 0xC0, 0x3B, 0x01, 0xCC, 0x0E, 0x30, 0x70,
  0xC3, 0x83, 0x1C, 0x0C, 0xE0, 0x37, 0x80, 0xFF, 0x03, 0xCC, 0x0E, 0x38,
  0x30, 0x70, 0xC0, 0xE3, 0x03, 0x8C, 0x07, 0x30, 0x0E, 0xC0, 0x18, 0xC0,
  0x30, 0x0C, 0x03, 0x00, 0xC0, 0x30, 0x0C, 0x03, 0x00, 0xC0, 0x30, 0x0C,
  0x03, 0x00, 0xC0, 0x30, 0x0C, 0x03, 0xFF, 0xFF, 0xC0, 0xF0, 0x07, 0xF0,
  0x0F, 0xF0, 0x0F, 0xF8, 0x0F, 0xD8, 0x1B, 0xD8, 0x1B, 0xDC, 0x1B, 0xCC,
  0x33, 0xCC, 0x33, 0xCE, 0x33, 0xC6, 0x63, 0xC6, 0x63, 0xC7, 0x63, 0xC3,
  0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0xC1, 0x83, 0xE0, 0x1F, 0x00, 0xFC, 0x07,
  0xE0, 0x3D, 0x81, 0xEE, 0x0F, 0x30, 0x78, 0xC3, 0xC7, 0x1E, 0x18, 0xF0,
  0x67, 0x83, 0xBC, 0x0D, 0xE0, 0x3F, 0x01, 0xF8, 0x07, 0xC0, 0x18, 0x07,
  0xE0, 0x1F, 0xF8, 0x38, 0x1C, 0x70, 0x0E, 0x60, 0x06, 0xE0, 0x07, 0xC0,
  0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x07, 0x60,
  0x06, 0x70, 0x0E, 0x38, 0x1C, 0x1F, 0xF8, 0x07, 0xE0, 0xFF, 0xC7, 0xFF,
  0xB0, 0x0D, 0x80, 0x3C, 0x01, 0xE0, 0x0F, 0x00, 0x78, 0x0E, 0xFF, 0xF7,
  0xFE, 0x30, 0x01, 0x80, 0x0C, 0x00, 0x60, 0x03, 0x00, 0x18, 0x00, 0xC0,
  0x00, 0x07, 0xE0, 0x1F, 0xF8, 0x38, 0x1C, 0x70, 0x0E, 0x60, 0x06, 0xC0,
  0x07, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0,
  0x07, 0x60, 0xC6, 0x70, 0xFE, 0x38, 0x3C, 0x1F, 0xFC, 0x07, 0xE7, 0x00,
  0x01, 0xFF, 0xE1, 0xFF, 0xE3, 0x00, 0xE6, 0x00, 0xCC, 0x01, 0x98, 0x03,
  0x30, 0x0E, 0x7F, 0xF8, 0xFF, 0xC1, 0x83, 0x83, 0x03, 0x86, 0x03, 0x0C,
  0x07, 0x18, 0x07, 0x30, 0x06, 0x60, 0x0E, 0xC0, 0x0E, 0x0F, 0xC1, 0xFF,
  0x1C, 0x1C, 0xC0, 0x76, 0x01, 0xB0, 0x01, 0xE0, 0x07, 0xF0, 0x0F, 0xE0,
  0x07, 0x80, 0x0F, 0x80, 0x3C, 0x01, 0xF0, 0x0D, 0xC0, 0xC7, 0xFE, 0x0F,
  0xC0, 0xFF, 0xFF, 0xFF, 0x06, 0x00, 0x60, 0x06, 0x00, 0x60, 0x06, 0x00,
  0x60, 0x06, 0x00, 0x60, 0x06, 0x00, 0x60, 0x06, 0x00, 0x60, 0x06, 0x00,
  0x60, 0x06, 0x00, 0xC0, 0x1E, 0x00, 0xF0, 0x07, 0x80, 0x3C, 0x01, 0xE0,
  0x0F, 0x00, 0x78, 0x03, 0xC0, 0x1E, 0x00, 0xF0, 0x07, 0x80, 0x3C, 0x01,
  0xB0, 0x19, 0xC1, 0xC7, 0xFC, 0x1F, 0xC0, 0xE0, 0x06, 0x60, 0x06, 0x60,
  0x06, 0x70, 0x0C, 0x30, 0x0C, 0x38, 0x1C, 0x18, 0x18, 0x18, 0x18, 0x1C,
  0x30, 0x0C, 0x30, 0x0C, 0x30, 0x06, 0x60, 0x06, 0x60, 0x07, 0xC0, 0x03,
  0xC0, 0x03, 0xC0, 0x01, 0x80, 0xE0, 0x38, 0x0D, 0x80, 0xE0, 0x36, 0x07,
  0xC1, 0x9C, 0x1B, 0x06, 0x30, 0x6C, 0x18, 0xC3, 0xB0, 0x63, 0x0C, 0x63,
  0x0E, 0x31, 0x8C, 0x18, 0xC6, 0x30, 0x66, 0x18, 0xC1, 0x98, 0x36, 0x07,
  0x60, 0xD8, 0x0F, 0x83, 0x60, 0x3C, 0x07, 0x80, 0xF0, 0x1C, 0x01, 0xC0,
  0x70, 0x07, 0x01, 0xC0, 0x70, 0x06, 0x38, 0x0C, 0x18, 0x1C, 0x1C, 0x38,
  0x0E, 0x70, 0x07, 0x60, 0x03, 0xE0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0,
  0x07, 0xE0, 0x0E, 0x70, 0x1C, 0x38, 0x18, 0x18, 0x38, 0x1C, 0x70, 0x0E,
  0xE0, 0x07, 0xE0, 0x06, 0x60, 0x0E, 0x30, 0x1C, 0x38, 0x18, 0x18, 0x38,
  0x0C, 0x70, 0x0E, 0x60, 0x06, 0xE0, 0x03, 0xC0, 0x03, 0x80, 0x01, 0x80,
  0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80,
  0xFF, 0xFF, 0xFF, 0xC0, 0x0C, 0x00, 0xC0, 0x0E, 0x00, 0xE0, 0x0E, 0x00,
  0x60, 0x06, 0x00, 0x70, 0x07, 0x00, 0x70, 0x07, 0x00, 0x30, 0x03, 0x00,
  0x1F, 0xFF, 0xFF, 0xF8, 0xFF, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
  0xCC, 0xCC, 0xFF, 0xC3, 0x06, 0x18, 0x61, 0x83, 0x0C, 0x30, 0xC3, 0x06,
  0x18, 0x61, 0x83, 0x0C, 0xFF, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
  0x33, 0x33, 0xFF, 0x0C, 0x07, 0x81, 0xE0, 0xC8, 0x33, 0x1C, 0xC6, 0x1B,
  0x86, 0xC0, 0xC0, 0xFF, 0xFF, 0xFF, 0xF0, 0xE6, 0x30, 0x1F, 0x87, 0xFC,
  0x60, 0xE0, 0x06, 0x01, 0xE3, 0xFE, 0x7E, 0x6E, 0x06, 0xC0, 0x6C, 0x0E,
  0xE1, 0xE7, 0xF6, 0x3E, 0x60, 0xC0, 0x30, 0x0C, 0x03, 0x00, 0xDE, 0x3F,
  0xCE, 0x1B, 0x87, 0xC0, 0xF0, 0x3C, 0x0F, 0x03, 0xC0, 0xF8, 0x6E, 0x3B,
  0xFC, 0xDE, 0x00, 0x1F, 0x07, 0xF9, 0xC3, 0x70, 0x3C, 0x01, 0x80, 0x30,
  0x06, 0x00, 0xC0, 0x7C, 0x0D, 0xC3, 0x1F, 0xE1, 0xF0, 0x00, 0x60, 0x0C,
  0x01, 0x80, 0x31, 0xE6, 0x7F, 0xDC, 0x7F, 0x07, 0xC0, 0x78, 0x0F, 0x01,
  0xE0, 0x3C, 0x06, 0xC1, 0xDC, 0x79, 0xFF, 0x1E, 0x60, 0x1F, 0x07, 0xF1,
  0xC3, 0x70, 0x3C, 0x07, 0xFF, 0xFF, 0xFE, 0x00, 0xC0, 0x1C, 0x0D, 0xC3,
  0x9F, 0xE1, 0xF0, 0x1E, 0x7C, 0xC1, 0x8F, 0xDF, 0x8C, 0x18, 0x30, 0x60,
  0xC1, 0x83, 0x06, 0x0C, 0x18, 0x30, 0x1E, 0x67, 0xFD, 0xC7, 0xF0, 0x7C,
  0x07, 0x80, 0xF0, 0x1E, 0x03, 0xC0, 0x7C, 0x1D, 0xC7, 0x9F, 0xF1, 0xE6,
  0x00, 0xF0, 0x3F, 0x06, 0x7F, 0xC7, 0xE0, 0xC0, 0x30, 0x0C, 0x03, 0x00,
  0xCF, 0x3F, 0xEE, 0x1F, 0x83, 0xC0, 0xF0, 0x3C, 0x0F, 0x03, 0xC0, 0xF0,
  0x3C, 0x0F, 0x03, 0xC0, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xC0, 0x18, 0xC0,
  0x01, 0x8C, 0x63, 0x18, 0xC6, 0x31, 0x8C, 0x63, 0x18, 0xC6, 0x37, 0xF8,
  0xC0, 0x30, 0x0C, 0x03, 0x00, 0xC3, 0xB1, 0xCC, 0xE3, 0x70, 0xF8, 0x3E,
  0x0F, 0x83, 0x30, 0xCE, 0x31, 0x8C, 0x33, 0x0E, 0xC1, 0x80, 0xFF, 0xFF,
  0xFF, 0xFF, 0xC0, 0xDE, 0x3C, 0xFF, 0x7E, 0xE3, 0xC7, 0xC1, 0x83, 0xC1,
  0x83, 0xC1, 0x83, 0xC1, 0x83, 0xC1, 0x83, 0xC1, 0x83, 0xC1, 0x83, 0xC1,
  0x83, 0xC1, 0x83, 0xC1, 0x83, 0xCF, 0x3F, 0xEE, 0x1F, 0x83, 0xC0, 0xF0,
  0x3C, 0x0F, 0x03, 0xC0, 0xF0, 0x3C, 0x0F, 0x03, 0xC0, 0xC0, 0x1F, 0x07,
  0xF1, 0xC7, 0x70, 0x6C, 0x07, 0x80, 0xF0, 0x1E, 0x03, 0xC0, 0x6C, 0x19,
  0xC7, 0x1F, 0xC1, 0xF0, 0xDE, 0x3F, 0xEE, 0x3B, 0x87, 0xC0, 0xF0, 0x3C,
  0x0F, 0x03, 0xC0, 0xF0, 0x6E, 0x3B, 0xFC, 0xDE, 0x30, 0x0C, 0x03, 0x00,
  0xC0, 0x30, 0x00, 0x1E, 0x67, 0xFD, 0xC7, 0xF0, 0x7C, 0x07, 0x80, 0xF0,
  0x1E, 0x03, 0xC0, 0x6C, 0x1D, 0xC7, 0x9F, 0xF1, 0xF6, 0x00, 0xC0, 0x18,
  0x03, 0x00, 0x60, 0x0C, 0xDF, 0xFB, 0x86, 0x0C, 0x18, 0x30, 0x60, 0xC1,
  0x83, 0x06, 0x0C, 0x00, 0x3F, 0x1F, 0xEE, 0x1F, 0x03, 0xE0, 0x3E, 0x03,
  0xF8, 0x3F, 0x00, 0xF0, 0x3E, 0x1D, 0xFE, 0x3F, 0x00, 0x23, 0x18, 0xCF,
  0xFD, 0x8C, 0x63, 0x18, 0xC6, 0x31, 0x8F, 0x38, 0xC1, 0xE0, 0xF0, 0x78,
  0x3C, 0x1E, 0x0F, 0x07, 0x83, 0xC1, 0xE0, 0xF8, 0xEF, 0xF3, 0xD8, 0xE0,
  0x66, 0x06, 0x60, 0x63, 0x0C, 0x30, 0xC3, 0x0C, 0x19, 0x81, 0x98, 0x19,
  0x80, 0xF0, 0x0F, 0x00, 0xF0, 0x06, 0x00, 0xC1, 0xC1, 0xF0, 0xE0, 0xD8,
  0x70, 0xCC, 0x3C, 0x66, 0x36, 0x31, 0x9B, 0x10, 0xCD, 0x98, 0x66, 0x6C,
  0x36, 0x36, 0x0F, 0x1E, 0x07, 0x8F, 0x03, 0xC3, 0x81, 0xC1, 0x80, 0x60,
  0x63, 0x0C, 0x39, 0xC1, 0x98, 0x0F, 0x00, 0xF0, 0x06, 0x00, 0xF0, 0x1F,
  0x81, 0x98, 0x30, 0xC7, 0x0E, 0x60, 0x70, 0x60, 0x76, 0x06, 0x70, 0x63,
  0x0C, 0x30, 0xC3, 0x8C, 0x19, 0x81, 0x98, 0x0D, 0x80, 0xF0, 0x0F, 0x00,
  0x60, 0x06, 0x00, 0x60, 0x0C, 0x00, 0xC0, 0x78, 0x07, 0x00, 0x7F, 0xEF,
  0xFC, 0x03, 0x00, 0xE0, 0x38, 0x0E, 0x03, 0x80, 0xE0, 0x18, 0x06, 0x01,
  0x80, 0x7F, 0xFF, 0xFE, 0x0E, 0x3C, 0x60, 0xC1, 0x83, 0x06, 0x0C, 0x38,
  0x63, 0x87, 0x03, 0x07, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC1, 0xE1, 0xC0,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xE1, 0xE0, 0xC1, 0x83, 0x06, 0x0C,
  0x18, 0x30, 0x30, 0x38, 0x71, 0x87, 0x0C, 0x18, 0x30, 0x60, 0xC1, 0x8F,
  0x1C, 0x00, 0x78, 0x0F, 0xE3, 0xC7, 0xF0, 0x1E };

constexpr GFXglyph Garuda12pt7bGlyphs[] = {
  {     0,   0,   0,   8,    0,    1 },   // 0x20 ' '
  {     0,   2,  17,   7,    2,  -16 },   // 0x21 '!'
  {     5,   6,   6,   9,    1,  -16 },   // 0x22 '"'
  {    10,  13,  17,  13,    0,  -16 },   // 0x23 '#'
  {    38,  11,  20,  13,    1,  -18 },   // 0x24 '$'
  {    66,  19,  18,  21,    1,  -16 },   // 0x25 '%'
  {   109,  14,  17,  16,    1,  -16 },   // 0x26 '&'
  {   139,   2,   6,   5,    1,  -16 },   // 0x27 '''
  {   141,   5,  22,   8,    1,  -16 },   // 0x28 '('
  {   155,   5,  22,   8,    2,  -16 },   // 0x29 ')'
  {   169,   8,   7,   9,    1,  -16 },   // 0x2A '*'
  {   176,  10,  10,  14,    2,  -13 },   // 0x2B '+'
  {   189,   3,   5,   6,    1,   -1 },   // 0x2C ','
  {   191,   6,   2,   8,    1,   -6 },   // 0x2D '-'
  {   193,   2,   2,   7,    2,   -1 },   // 0x2E '.'
  {   194,   6,  17,   7,    0,  -16 },   // 0x2F '/'
  {   207,  11,  17,  13,    1,  -16 },   // 0x30 '0'
  {   231,   6,  17,  13,    3,  -16 },   // 0x31 '1'
  {   244,  11,  17,  13,    1,  -16 },   // 0x32 '2'
  {   268,  11,  17,  13,    1,  -16 },   // 0x33 '3'
  {   292,  11,  17,  13,    1,  -16 },   // 0x34 '4'
  {   316,  11,  17,  13,    1,  -16 },   // 0x35 '5'
  {   340,  11,  17,  13,    1,  -16 },   // 0x36 '6'
  {   364,  10,  17,  13,    2,  -16 },   // 0x37 '7'
  {   386,  11,  17,  13,    1,  -16 },   // 0x38 '8'
  {   410,  11,  17,  13,    1,  -16 },   // 0x39 '9'
  {   434,   2,  13,   7,    2,  -12 },   // 0x3A ':'
  {   438,   3,  16,   6,    1,  -12 },   // 0x3B ';'
  {   444,  12,  11,  14,    1,  -13 },   // 0x3C '<'
  {   461,  12,   7,  14,    1,  -11 },   // 0x3D '='
  {   472,  12,  11,  14,    1,  -13 },   // 0x3E '>'
  {   489,  11,  17,  13,    1,  -16 },   // 0x3F '?'
  {   513,  22,  22,  24,    1,  -16 },   // 0x40 '@'
  {   574,  16,  17,  16,    0,  -16 },   // 0x41 'A'
  {   608,  12,  17,  16,    2,  -16 },   // 0x42 'B'
  {   634,  15,  17,  17,    1,  -16 },   // 0x43 'C'
  {   666,  14,  17,  17,    2,  -16 },   // 0x44 'D'
  {   696,  12,  17,  16,    2,  -16 },   // 0x45 'E'
  {   722,  11,  17,  15,    2,  -16 },   // 0x46 'F'
  {   746,  16,  17,  19,    1,  -16 },   // 0x47 'G'
  {   780,  13,  17,  17,    2,  -16 },   // 0x48 'H'
  {   808,   2,  17,   7,    2,  -16 },   // 0x49 'I'
  {   813,   9,  17,  12,    1,  -16 },   // 0x4A 'J'
  {   833,  14,  17,  16,    2,  -16 },   // 0x4B 'K'
  {   863,  10,  17,  13,    2,  -16 },   // 0x4C 'L'
  {   885,  16,  17,  20,    2,  -16 },   // 0x4D 'M'
  {   919,  13,  17,  17,    2,  -16 },   // 0x4E 'N'
  {   947,  16,  17,  19,    1,  -16 },   // 0x4F 'O'
  {   981,  13,  17,  16,    2,  -16 },   // 0x50 'P'
  {  1009,  16,  18,  19,    1,  -16 },   // 0x51 'Q'
  {  1045,  15,  17,  17,    2,  -16 },   // 0x52 'R'
  {  1077,  13,  17,  16,    1,  -16 },   // 0x53 'S'
  {  1105,  12,  17,  15,    1,  -16 },   // 0x54 'T'
  {  1131,  13,  17,  17,    2,  -16 },   // 0x55 'U'
  {  1159,  16,  17,  16,    0,  -16 },   // 0x56 'V'
  {  1193,  22,  17,  23,    0,  -16 },   // 0x57 'W'
  {  1240,  16,  17,  16,    0,  -16 },   // 0x58 'X'
  {  1274,  16,  17,  16,    0,  -16 },   // 0x59 'Y'
  {  1308,  13,  17,  15,    1,  -16 },   // 0x5A 'Z'
  {  1336,   4,  22,   7,    2,  -16 },   // 0x5B '['
  {  1347,   6,  17,   7,    0,  -16 },   // 0x5C '\'
  {  1360,   4,  22,   7,    0,  -16 },   // 0x5D ']'
  {  1371,  10,   9,  11,    1,  -16 },   // 0x5E '^'
  {  1383,  14,   2,  13,    0,    4 },   // 0x5F '_'
  {  1387,   4,   3,   8,    1,  -16 },   // 0x60 '`'
  {  1389,  12,  13,  13,    1,  -12 },   // 0x61 'a'
  {  1409,  10,  17,  13,    2,  -16 },   // 0x62 'b'
  {  1431,  11,  13,  12,    1,  -12 },   // 0x63 'c'
  {  1449,  11,  17,  13,    1,  -16 },   // 0x64 'd'
  {  1473,  11,  13,  13,    1,  -12 },   // 0x65 'e'
  {  1491,   7,  17,   7,    0,  -16 },   // 0x66 'f'
  {  1506,  11,  18,  13,    1,  -12 },   // 0x67 'g'
  {  1531,  10,  17,  13,    2,  -16 },   // 0x68 'h'
  {  1553,   2,  17,   5,    2,  -16 },   // 0x69 'i'
  {  1558,   5,  22,   5,   -1,  -16 },   // 0x6A 'j'
  {  1572,  10,  17,  12,    2,  -16 },   // 0x6B 'k'
  {  1594,   2,  17,   5,    2,  -16 },   // 0x6C 'l'
  {  1599,  16,  13,  20,    2,  -12 },   // 0x6D 'm'
  {  1625,  10,  13,  13,    2,  -12 },   // 0x6E 'n'
  {  1642,  11,  13,  13,    1,  -12 },   // 0x6F 'o'
  {  1660,  10,  18,  13,    2,  -12 },   // 0x70 'p'
  {  1683,  11,  18,  13,    1,  -12 },   // 0x71 'q'
  {  1708,   7,  13,   8,    2,  -12 },   // 0x72 'r'
  {  1720,  10,  13,  12,    1,  -12 },   // 0x73 's'
  {  1737,   5,  17,   7,    1,  -16 },   // 0x74 't'
  {  1748,   9,  13,  13,    2,  -12 },   // 0x75 'u'
  {  1763,  12,  13,  12,    0,  -12 },   // 0x76 'v'
  {  1783,  17,  13,  17,    0,  -12 },   // 0x77 'w'
  {  1811,  12,  13,  12,    0,  -12 },   // 0x78 'x'
  {  1831,  12,  18,  12,    0,  -12 },   // 0x79 'y'
  {  1858,  11,  13,  12,    0,  -12 },   // 0x7A 'z'
  {  1876,   7,  22,   8,    0,  -16 },   // 0x7B '{'
  {  1896,   2,  22,   6,    2,  -16 },   // 0x7C '|'
  {  1902,   7,  22,   8,    1,  -16 },   // 0x7D '}'
  {  1922,  12,   4,  14,    1,  -10 } }; // 0x7E '~'

constexpr GFXfont Garuda12pt7b = {
  Garuda12pt7bBitmaps,
  Garuda12pt7bGlyphs,
  0x20, 0x7E, 45 };

GFXfont const * get_fontGaruda12pt7b()
{ return &Garuda12pt7b;}

// Approx. 2600 bytes