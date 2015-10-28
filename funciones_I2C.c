#include "funciones_I2C.h"
#include "built_in.h"

char data_[256];
//FUNCIONES para los sensores
//rutina de configuracion
void escribirI2C(unsigned short wAddr, unsigned short wData, unsigned short SAddres){
     data_[0] = wAddr;
     data_[1] = wData;
     I2C2_Start();
     I2C2_Write(SAddres,data_,2,END_MODE_STOP);
     delay_ms(100);
}

//rutina de lectura
unsigned short leerI2C(unsigned short rAddr, unsigned short SAddres){
         data_[0] = rAddr;
         I2C2_Start();
         I2C2_Write(SAddres, data_,1, END_MODE_RESTART);
         I2C2_Read(SAddres,data_,1,END_MODE_STOP);
         return data_[0];
}
