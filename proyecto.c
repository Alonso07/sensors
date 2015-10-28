#include "funciones_I2C.h"
#include "funciones_mcu.h"
#include "funciones_sensores.h"

void main() {
initial_mcu();
initial_sensors();
calibrarG();
while(1){
            // interrupcion determinado tiempo
            if(TIM2_SR.UIF == 1){
                  leer_MPU();
                  leer_COMPAS();
                  datos_MPU();
                  datos_COMPAS();
                  enviar_datos_mpu();
                  enviar_datos_compas();
          }
            }
}
