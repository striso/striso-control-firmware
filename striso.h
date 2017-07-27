#ifndef _STRISO_H
#define _STRISO_H

// Striso protocol source ID's
#define ID_DIS 0      // buttons on dis side
#define ID_BAS 1      // buttons on bas side
#define ID_CONTROL 2  // other controls
#define ID_ACCEL 3    // motion sensor
#define ID_SYS 4      // system, status, debug

// control numbers
#define IDC_PORTAMENTO 0
#define IDC_OCT_UP 1
#define IDC_OCT_DOWN 2
#define IDC_ALT 3
#define IDC_SLD_NPRESS 8
#define IDC_SLD_SLIDE 9
#define IDC_SLD_SLIDEZOOM 10
#define IDC_SLD_ONE_TAP 3
#define IDC_SLD_TWO_TAP 4

// system numbers
#define ID_SYS_MSGQUE_OVERFLOW_BB 1
#define ID_SYS_MSGQUE_OVERFLOW_SYNTH 2
#define ID_SYS_MSG_TOO_SHORT_SYNTH 3
#define ID_SYS_MSG_TOO_LONG_SYNTH 4

#endif
