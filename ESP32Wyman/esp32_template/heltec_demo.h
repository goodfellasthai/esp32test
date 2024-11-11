#ifndef HELTEC_DEMO_H
#define HELTEC_DEMO_H

#include <heltec_unofficial.h> // Your interface header now manages dependencies
// Include custom images
#include "images.h"

#define DEMO_DURATION 3000
typedef void (*Demo)(void);

// Declare global variables with extern
extern int demoMode;
extern int counter;
extern Demo demos[];
extern int demoLength;
extern long timeSinceLastModeSwitch;  // Add missing extern declarations

// Function prototypes
void heltec_Init(void);
void drawFontFaceDemo(void);
void drawTextFlowDemo(void);
void drawTextAlignmentDemo(void);
void drawRectDemo(void);
void drawCircleDemo(void);
void drawProgressBarDemo(void);
void drawImageDemo(void);
void heltec_loopCode(void);

#endif // HELTEC_DEMO_H