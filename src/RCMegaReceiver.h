/******************************************************************************/
/**
 * \file
 * \brief Declaration of Radio Control Receiver to Arduino Mega only
 *        All code here is adapted from Multiwii 2.4 - RX.cpp file
 */
/******************************************************************************/

#ifndef RCMEGARECEIVER_H
#define RCMEGARECEIVER_H

void configureReceiver();
int16_t *getRcData();

#endif // RCMEGARECEIVER_H