#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import spidev

#MCP3208から値を取得するクラス
class MCP3208_Class:

    """コンストラクタ"""
    def __init__(self, ref_volts):
        self.ref_volts = ref_volts
        self.spi = spidev.SpiDev()
        self.spi.open(0,0)


    """電圧取得"""
    def GetVoltage(self,ch):
        raw = self.spi.xfer2([1,(8+ch)<<4,0])
        raw2 = ((raw[1]&3) << 8) + raw[2]
        volts = (raw2 * self.ref_volts ) / float(1023)
        volts = round(volts,4)
        return volts

    """終了処理"""
    def Cleanup(self):
        self.spi.close()

"""メイン関数"""
if __name__ == '__main__':
    ADC = MCP3208_Class(ref_volts=5)
    try:
        while True:
            volts = ADC.GetVoltage(ch=0)
            print("volts: {:8.2f}".format(volts))
            time.sleep(1)

    except KeyboardInterrupt  :         #Ctl+Cが押されたらループを終了
        print("\nCtl+C")
    except Exception as e:
        print(str(e))
    finally:
        ADC.Cleanup()
        print("\nexit program")
