// ***** MODIFIED VERSION ***** //
/*
Library for King-Meter displays

Copyright © 2015 Michael Fabry (Michael@Fabry.de)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


// Includes
#include "display_kingmeter.h"

#if (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)

// Definitions
#define RXSTATE_STARTCODE   0
#define RXSTATE_SENDTXMSG   1
#define RXSTATE_MSGBODY     2
#define RXSTATE_DONE        3


// Hashtable used for handshaking in 901U protocol
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
const uint8_t KM_901U_HANDSHAKE[64] =
{
    201, 107,  13, 229, 241, 198, 108, 230, 186,  67,  39,  92, 217, 140, 177,  36,
     22,  71, 174,  39, 161, 151,   7, 140, 107, 155, 189, 195, 209, 106,  63, 191,
    218,  47, 221,  46, 135, 145,  98,  82,  35,  42,  85,  99,  35,  43, 180,  12,
      3, 126,  94, 103, 198,  10, 182, 249, 253,  86, 105, 196, 217, 183, 195, 115
};
#endif


// Local function prototypes
#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
static void KM_618U_Service(KINGMETER_t* KM_ctx);
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
static void KM_901U_Service(KINGMETER_t* KM_ctx);
#endif



/* Public functions (Prototypes declared by display_kingmeter.h) */

/****************************************************************************************************
 * KingMeter_Init() - Initializes the display object
 *
 ****************************************************************************************************/
#if HARDWARE_REV < 20
void KingMeter_Init (KINGMETER_t* KM_ctx, SoftwareSerial* DisplaySerial)
#else
void KingMeter_Init (KINGMETER_t* KM_ctx, HardwareSerial* DisplaySerial)
#endif
{
    uint8_t i;


    KM_ctx->SerialPort                      = DisplaySerial;            // Store serial port to use

    KM_ctx->RxState                         = RXSTATE_STARTCODE;
    KM_ctx->LastRx                          = millis();

    for(i=0; i<KM_MAX_TXBUFF; i++)
    {
        KM_ctx->RxBuff[i]                   = 0x00;
    }

    KM_ctx->RxCnt                           = 0;

    // Settings received from display:
    KM_ctx->Settings.PAS_RUN_Direction      = KM_PASDIR_FORWARD;
    KM_ctx->Settings.PAS_SCN_Tolerance      = (uint8_t) pas_tolerance;
    KM_ctx->Settings.PAS_N_Ratio            = 255;
    KM_ctx->Settings.HND_HL_ThrParam        = KM_HND_HL_NO;
    KM_ctx->Settings.HND_HF_ThrParam        = KM_HND_HF_NO;
    KM_ctx->Settings.SYS_SSP_SlowStart      = 1;
    KM_ctx->Settings.SPS_SpdMagnets         = (uint8_t) wheel_magnets;
    KM_ctx->Settings.VOL_1_UnderVolt_x10    = (uint16_t) (vcutoff * 10);
    KM_ctx->Settings.WheelSize_mm           = (uint16_t) (wheel_circumference * 1000);

    // Parameters received from display in operation mode:
    KM_ctx->Rx.AssistLevel                  = 0;
    KM_ctx->Rx.Headlight                    = KM_HEADLIGHT_OFF;
    KM_ctx->Rx.Battery                      = KM_BATTERY_NORMAL;
    KM_ctx->Rx.PushAssist                   = KM_PUSHASSIST_OFF;
    KM_ctx->Rx.PowerAssist                  = KM_POWERASSIST_ON;
    KM_ctx->Rx.Throttle                     = KM_THROTTLE_ON;
    KM_ctx->Rx.CruiseControl                = KM_CRUISE_OFF;
    KM_ctx->Rx.OverSpeed                    = KM_OVERSPEED_NO;
    KM_ctx->Rx.SPEEDMAX_Limit_x10           = (uint16_t) (spd_max1 * 10);
    KM_ctx->Rx.CUR_Limit_x10                = 150;

    // Parameters to be send to display in operation mode:
    KM_ctx->Tx.Battery                      = KM_BATTERY_NORMAL;
    KM_ctx->Tx.Wheeltime_ms                 = KM_MAX_WHEELTIME;
    KM_ctx->Tx.Error                        = KM_ERROR_NONE;
    KM_ctx->Tx.Current_x10                  = 0;


    DisplaySerial->begin(9600);
}



/****************************************************************************************************
 * KingMeter_Service() - Communicates data from and to the display
 *
 ***************************************************************************************************/
void KingMeter_Service(KINGMETER_t* KM_ctx)
{
    #if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
    KM_618U_Service(KM_ctx);
    #endif

    #if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
    KM_901U_Service(KM_ctx);
    #endif
}



/****************************************************************************************************
 * hearthBeatEBiCS() - Wrapper for KingMeter_Service (swapped Rx <-> Tx)
 *
 ***************************************************************************************************/
void hearthBeatEBiCS(KINGMETER_t* KM_ctx)
{
    static bool isSending = 0;
    if (!isSending && (millis() - KM_ctx->LastRx >= 500))
    {
        isSending = 1;

        // Message received completely
        KM_ctx->RxState = RXSTATE_SENDTXMSG;
        KM_ctx->LastRx = millis();

        #if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
        KM_ctx->RxBuff[3] = 0x52;         // Operation mode
        #endif

        // send hearthBeat to EBiCS Controller every 500 ms
        KingMeter_Service(KM_ctx);

        isSending = 0;
    }
}




/* Local functions */

#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_618U)
/****************************************************************************************************
 * KM_618U_Service() - Communication protocol of 618U firmware (swapped Rx <-> Tx)
 *
 ***************************************************************************************************/
static void KM_618U_Service(KINGMETER_t* KM_ctx)
{
    // BEWARE: this function is re-written to emulate Display (swapped Rx <-> Tx)

    uint8_t i;
    uint8_t CheckSum;
    uint8_t TxBuff[KM_MAX_RXBUFF];


    // Message received completely
    if(KM_ctx->RxState == RXSTATE_SENDTXMSG)
    {
        KM_ctx->RxState = RXSTATE_STARTCODE;

        // Prepare Tx message
        TxBuff[0] = 0X46;                                               // StartCode

        // encode PAS level - Display sets PAS-level to 0 when overspeed detected!
        TxBuff[1] = (map(KM_ctx->Rx.AssistLevel, 0, 255, 0, 5)       & 0x07) |  // 0..255
                    ((KM_ctx->Rx.Headlight                     << 7) & 0x80) |  // KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON
                    ((KM_ctx->Rx.PushAssist                    << 4) & 0x10);   // KM_PUSHASSIST_OFF / KM_PUSHASSIST_ON
        TxBuff[2] = ((KM_ctx->Rx.SPEEDMAX_Limit_x10 / 10 - 10) << 3) & 0xF8;    // Unit: 0.1km/h

        // encode Wheelsize by hashtable
        for (i = 0; i < 8; i++)
        {
            if (KM_ctx->Settings.WheelSize_mm == KM_WHEELSIZE[i])               // Unit: 1mm
            {
                TxBuff[2] = (TxBuff[2] & 0xF8) | (i & 0x07);
                break;
            }
        }

        TxBuff[3] = 0x00;                                                       // (unused)


        // Send prepared message
        TxBuff[4] = 0x00;

        KM_ctx->SerialPort->write(TxBuff[0]);                           // Send StartCode

        for(i=1; i<4; i++)
        {
            KM_ctx->SerialPort->write(TxBuff[i]);                       // Send TxBuff[1..3]
            TxBuff[4] = TxBuff[4] ^ TxBuff[i];                          // Calculate XOR CheckSum
        }

        KM_ctx->SerialPort->write(TxBuff[4]);                           // Send XOR CheckSum
    }


    // Search for Start Code
    if(KM_ctx->RxState == RXSTATE_STARTCODE)
    {
        if(KM_ctx->SerialPort->available())
        {
            KM_ctx->LastRx = millis();

            if(KM_ctx->SerialPort->read() == 0x46)
            {
                KM_ctx->RxBuff[0] = 0x46;
                KM_ctx->RxCnt = 1;
                KM_ctx->RxState = RXSTATE_MSGBODY;
            }
            else
            {
                return;                                                 // No need to continue
            }
        }
    }

    // Receive Message body
    if(KM_ctx->RxState == RXSTATE_MSGBODY)
    {
        while(KM_ctx->SerialPort->available())
        {
            KM_ctx->RxBuff[KM_ctx->RxCnt] = KM_ctx->SerialPort->read();
            KM_ctx->RxCnt++;

            if(KM_ctx->RxCnt > 7)                                       // Check for reception of complete message
            {
                CheckSum = 0x00;
                for(i=1; i<7; i++)
                {
                    CheckSum ^= KM_ctx->RxBuff[i];                      // Calculate XOR CheckSum
                }

                // Verify XOR CheckSum
                if(KM_ctx->RxBuff[7] == CheckSum)
                {
                    KM_ctx->RxState = RXSTATE_DONE;
                }
                else
                {
                    KM_ctx->RxState = RXSTATE_STARTCODE;
                }
                break;
            }
        }
    }

    // Message received completely
    if(KM_ctx->RxState == RXSTATE_DONE)
    {
        KM_ctx->RxState = RXSTATE_SENDTXMSG;

        if(KM_ctx->RxBuff[1] == 0x00)
        {
            KM_ctx->Tx.Battery = KM_BATTERY_LOW;                        // If none of Bit[0..2] is set, display blinks
        }
        else
        {
            KM_ctx->Tx.Battery = KM_BATTERY_NORMAL;
        }

        KM_ctx->Tx.Current_x10        = (uint16_t)(KM_ctx->RxBuff[2] * 10 / 3); // Current unit: 1/3A
        KM_ctx->Tx.Wheeltime_ms       = ((uint16_t)KM_ctx->RxBuff[3] << 8) |
                                                   KM_ctx->RxBuff[4];
//        KM_ctx->Settings.WheelSize_mm = KM_WHEELSIZE[KM_ctx->RxBuff[5] & 0x07]; // Reply with WheelSize 26" / Maxspeed 25km/h (no influence on display)
        KM_ctx->Tx.Error              = KM_ctx->RxBuff[6];
    }
}
#endif




#if (DISPLAY_TYPE == DISPLAY_TYPE_KINGMETER_901U)
/****************************************************************************************************
 * KM_901U_Service() - Communication protocol of 901U firmware (swapped Rx <-> Tx)
 *
 ***************************************************************************************************/
static void KM_901U_Service(KINGMETER_t* KM_ctx)
{
    // BEWARE: this function is re-written to emulate Display (swapped Rx <-> Tx)

    uint8_t  i;
    uint16_t CheckSum;
    uint8_t  TxBuff[KM_MAX_RXBUFF];
    uint8_t  TxCnt;


    // Message received completely
    if(KM_ctx->RxState == RXSTATE_SENDTXMSG)
    {
        KM_ctx->RxState = RXSTATE_STARTCODE;

        switch(KM_ctx->RxBuff[3])
        {
            case 0x52:      // Operation mode

                // Prepare Tx message
                TxBuff[0]  = 0X3A;                                      // StartCode
                TxBuff[1]  = 0x1A;                                      // SrcAdd:  Controller
                TxBuff[2]  = 0X28;                                      // DestAdd: LCD
                TxBuff[3]  = 0x52;                                      // CmdCode
                TxBuff[4]  = 0x06;                                      // DataSize

                TxBuff[5]  =  KM_ctx->Rx.AssistLevel;                               // 0..255
                TxBuff[6]  = (KM_ctx->Rx.Headlight     << 6) |                      // KM_HEADLIGHT_OFF / KM_HEADLIGHT_ON / KM_HEADLIGHT_LOW / KM_HEADLIGHT_HIGH
                             (KM_ctx->Rx.Battery       << 5) |                      // KM_BATTERY_NORMAL / KM_BATTERY_LOW
                             (KM_ctx->Rx.PushAssist    << 4) |                      // KM_PUSHASSIST_OFF / KM_PUSHASSIST_ON
                             (KM_ctx->Rx.PowerAssist   << 3) |                      // KM_THROTTLE_OFF / KM_THROTTLE_ON
                             (KM_ctx->Rx.Throttle      << 2) |                      // KM_CRUISE_OFF / KM_CRUISE_ON
                             (KM_ctx->Rx.CruiseControl << 1) |                      // KM_OVERSPEED_NO / KM_OVERSPEED_YES
                              KM_ctx->Rx.OverSpeed;                                 // KM_OVERSPEED_NO / KM_OVERSPEED_YES
                TxBuff[7]  =  KM_ctx->Rx.SPEEDMAX_Limit_x10       & 0xFF;           // Unit: 0.1km/h
                TxBuff[8]  = (KM_ctx->Rx.SPEEDMAX_Limit_x10 >> 8) & 0xFF;
                TxBuff[9]  =  KM_ctx->Rx.CUR_Limit_x10            & 0xFF;           // Unit: 0.1A
                TxBuff[10] = (KM_ctx->Rx.CUR_Limit_x10      >> 8) & 0xFF;

                TxCnt = 11;
                break;


            case 0x53:      // Settings mode

                // Prepare Tx message with handshake code
                TxBuff[0] = 0X3A;                                       // StartCode
                TxBuff[1] = 0x1A;                                       // SrcAdd:  Controller
                TxBuff[2] = 0X28;                                       // DestAdd: LCD
                TxBuff[3] = 0x53;                                       // CmdCode
                TxBuff[4] = 0x01;                                       // DataSize

                TxBuff[5]  =  (KM_ctx->Settings.PAS_RUN_Direction   << 7) & 0x80;       // KM_PASDIR_FORWARD / KM_PASDIR_BACKWARD
                TxBuff[6]  =   KM_ctx->Settings.PAS_SCN_Tolerance;                      // 2..9 Number of PAS signals to start the motor
                TxBuff[7]  =   KM_ctx->Settings.PAS_N_Ratio;                            // 0..255 PAS ratio
                TxBuff[8]  = ((KM_ctx->Settings.HND_HL_ThrParam     << 7) & 0x80) |     // KM_HND_HL_NO / KM_HND_HL_YES
                             ((KM_ctx->Settings.HND_HF_ThrParam     << 6) & 0x40);      // KM_HND_HF_NO / KM_HND_HF_YES
                TxBuff[9]  =   KM_ctx->Settings.SYS_SSP_SlowStart;                      // 1..4 Level of soft ramping at start
                TxBuff[10] =   KM_ctx->Settings.SPS_SpdMagnets;                         // Number of magnets of speedsensor
                TxBuff[11] =   KM_ctx->Settings.VOL_1_UnderVolt_x10       & 0xFF;       // Unit: 0.1V
                TxBuff[12] =  (KM_ctx->Settings.VOL_1_UnderVolt_x10 >> 8) & 0xFF;
                TxBuff[13] =   KM_ctx->Settings.WheelSize_mm              & 0xFF;       // Unit: 1mm
                TxBuff[14] =  (KM_ctx->Settings.WheelSize_mm        >> 8) & 0xFF;

                TxBuff[15] = random(0, sizeof(KM_901U_HANDSHAKE)-1);                    // Handshake

                TxCnt = 16;
                break;


            default:
                TxCnt = 0;
        }


        // Send prepared message
        if(TxCnt != 0)
        {
            CheckSum = 0x0000;

            KM_ctx->SerialPort->write(TxBuff[0]);                       // Send StartCode

            for(i=1; i<TxCnt; i++)
            {
                KM_ctx->SerialPort->write(TxBuff[i]);                   // Send TxBuff[1..x]
                CheckSum = CheckSum + TxBuff[i];                        // Calculate CheckSum
            }

            KM_ctx->SerialPort->write(lowByte(CheckSum));               // Send CheckSum low
            KM_ctx->SerialPort->write(highByte(CheckSum));              // Send CheckSum high

            KM_ctx->SerialPort->write(0x0D);                            // Send CR
            KM_ctx->SerialPort->write(0x0A);                            // Send LF
        }
    }


    // Search for Start Code
    if(KM_ctx->RxState == RXSTATE_STARTCODE)
    {
        if(KM_ctx->SerialPort->available())
        {
            KM_ctx->LastRx = millis();

            if(KM_ctx->SerialPort->read() == 0x3A)
            {
                KM_ctx->RxBuff[0] = 0x3A;
                KM_ctx->RxCnt = 1;
                KM_ctx->RxState = RXSTATE_MSGBODY;
            }
            else
            {
                return;                                                 // No need to continue
            }
        }
    }

    // Receive Message body
    if(KM_ctx->RxState == RXSTATE_MSGBODY)
    {
        while(KM_ctx->SerialPort->available())
        {
            KM_ctx->RxBuff[KM_ctx->RxCnt] = KM_ctx->SerialPort->read();
            KM_ctx->RxCnt++;

            if(KM_ctx->RxCnt == 5)                                      // Range check of Data size
            {
                if(KM_ctx->RxBuff[4] > (KM_MAX_RXBUFF-5-4))
                {
                    KM_ctx->RxState = RXSTATE_STARTCODE;                // Invalid Data size, cancel reception
                    break;
                }
            }


            if(KM_ctx->RxCnt == (5 + KM_ctx->RxBuff[4] + 4))            // Check for reception of complete message
            {
                // Verify CheckSum
                CheckSum = 0x0000;
                for(i=1; i<(5+KM_ctx->RxBuff[4]); i++)
                {
                    CheckSum = CheckSum + KM_ctx->RxBuff[i];            // Calculate CheckSum
                }

                if((lowByte(CheckSum) == KM_ctx->RxBuff[i]) && (highByte(CheckSum) == KM_ctx->RxBuff[i+1]))
                {
                    KM_ctx->RxState = RXSTATE_DONE;
                }
                else
                {
                    KM_ctx->RxState = RXSTATE_STARTCODE;                // Invalid CheckSum, ignore message
                }

                break;
            }
        }
    }


    // Message received completely
    if(KM_ctx->RxState == RXSTATE_DONE)
    {
        KM_ctx->RxState = RXSTATE_SENDTXMSG;

        switch(KM_ctx->RxBuff[3])
        {
            case 0x52:      // Operation mode

                // Decode Rx message
                if (KM_ctx->RxBuff[5] == 0x40)
                {
                    KM_ctx->Tx.Battery = KM_BATTERY_LOW;                // State data (only UnderVoltage bit has influence on display)
                }
                else
                {
                    KM_ctx->Tx.Battery = KM_BATTERY_NORMAL;             // State data (only UnderVoltage bit has influence on display)
                }
                KM_ctx->Tx.Current_x10        = (KM_ctx->RxBuff[7] << 8) | KM_ctx->RxBuff[6];
                KM_ctx->Tx.Wheeltime_ms       = (KM_ctx->RxBuff[9] << 8) | KM_ctx->RxBuff[8];
                KM_ctx->Tx.Error              =  KM_ctx->RxBuff[10];

                break;


            case 0x53:      // Settings mode
   
                // Decode Rx message with handshake code
                if (KM_ctx->RxBuff[5] == KM_901U_HANDSHAKE[TxBuff[15]])
                {
                    Serial.println("OK: Handshake true");
                }
                else
                {
                    Serial.println("Error: Invalid Handshake");
                }
                break;

            default:
                KM_ctx->RxCnt = 0;
        }
    }
}
#endif

#endif // (DISPLAY_TYPE & DISPLAY_TYPE_KINGMETER)
