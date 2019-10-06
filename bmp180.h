#ifndef __BMP180_H
#define BMP180_H

/* variables to store BMP180 Calibration */
static int16_t AC1;
static int16_t AC2;
static int16_t AC3;
static uint16_t AC4;
static uint16_t AC5;
static uint16_t AC6;
static int16_t VB1;
static int16_t VB2;
static int16_t MB;
static int16_t MC;
static int16_t MD;

static double c3;
static double c4;
static double b1;
static double c5;
static double c6;
static double mc;
static double md;
static double x0;
static double x1;
static double x2;
static double y0;
static double y1;
static double y2;
static double p0;
static double p1;
static double p2;

/* Buffer to store read i2c values */
uint8_t data[2];
#define BMP180_ADDR 0x77 /* 7-bit address */
/* BMP180 Register Mappings */
#define	BMP180_REG_CONTROL 0xF4
#define	BMP180_REG_RESULT 0xF6

#define	BMP180_COMMAND_TEMPERATURE 0x2E
#define	BMP180_COMMAND_PRESSURE0 0x34
#define	BMP180_COMMAND_PRESSURE1 0x74
#define	BMP180_COMMAND_PRESSURE2 0xB4
#define	BMP180_COMMAND_PRESSURE3 0xF4

#endif