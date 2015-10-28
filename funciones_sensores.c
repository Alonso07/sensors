#include "funciones_I2C.h"
#include "funciones_sensores.h"
#include "built_in.h"

char AX2T[12], AY2T[12], AZ2T[12], MXT[12], MYT[12], MZT[12], P[12], R[12], H[12]; // variables del compas
char AX1T[12], AY1T[12], AZ1T[12], GXT[12], GYT[12], GZT[12];        // variables de la mpu

float XF=0, YF=0, ZF=0;
float A1XF=0, A1YF=0, A1ZF=0;
float GXF=0, GYF=0, GZF=0;
float A2XF=0, A2YF=0, A2ZF=0;
float MXF=0, MYF=0, MZF=0;
float AN_ZX=0, AN_ZY=0, AN_YX=0;
float GX0=0, GY0=0, GZ0=0;
float TX0=0, TY0=0, TZ0=0, TX=0, TY=0, TZ=0;

#define LSM202DLHM (0x3C >> 1)   // DIRECCION FISICA DE EL MAGNETOMETRO DEL COMPAS
#define LSM202DLHA (0x32 >> 1)  // DIRECCION FISICA DEL ACELEROMETRO DEL COMPAS
#define ADDRMPU6050  0x69 //direccion fisica de LA MPU

// REGISTROS DE CONFIGURACION DEL MAGNETOMETRO DEL COMPAS
unsigned short CRA_REG_M = 0x00;
unsigned short CRB_REG_M = 0x01;
unsigned short MR_REG_M = 0x02;

// REGISTROS DE DATOS DEL MAGNETOMETRO
unsigned short OUT_X_H_H = 0x03;
unsigned short OUT_X_L_H = 0x04;

unsigned short OUT_Z_H_H = 0x05;
unsigned short OUT_Z_L_H = 0x06;

unsigned short OUT_Y_H_H = 0x07;
unsigned short OUT_Y_L_H = 0x08;

// REGISTROS DE CONFIGURACION DEL ACELEROMETRO DEL COMPAS
unsigned short CTRL_REG1_A = 0x20;
unsigned short CTRL_REG2_A = 0x21;
unsigned short CTRL_REG3_A = 0x22;
unsigned short CTRL_REG4_A = 0x23;
unsigned short CTRL_REG5_A = 0x24;

// REGISTROS DE DATOS DEL ACELEROMETRO
unsigned short OUT_X_H_A = 0x28;
unsigned short OUT_X_L_A = 0x29;

unsigned short OUT_Y_H_A = 0x2A;
unsigned short OUT_Y_L_A = 0x2B;

unsigned short OUT_Z_H_A = 0x2C;
unsigned short OUT_Z_L_A = 0x2D;

// REGISTROS DE CONFIGURACION DE LA MPU

#define CONFIG  0x1A
//unsigned short CONFIG = 0x1A;    // configuracion del sensor
unsigned short ACCEL_CONFIG = 0x1C;  // accel configuration
unsigned short GYRO_CONFIG = 0X1B;   // configuracion del giroscopio
unsigned short FIFO_1 = 0x23;  //configuracion 2 fifo_enable
unsigned short INT_EN = 0x38;  //configuracion 3  int enable
unsigned short PWR_MGMT_1 = 0x6B;  //configuracion 1 mpu_rm_PWR_MGMT_1

// REGISTROS DE DATOS DE LA MPU
#define ahighx  0x3B
//unsigned short ahighx = 0x3B; //Parte alta x     aceleracion
unsigned short alowx = 0x3C;  //Parte baja x
unsigned short ahighy = 0x3D; //Parte alta y
unsigned short alowy = 0x3E;  //Parte baja y
unsigned short ahighz = 0x3F; //Parte alta z
unsigned short alowz = 0x40;  //Parte baja z

unsigned short hight = 0x41;
unsigned short lowt = 0x42;

unsigned short ghighx = 0x43; //Parte alta x       giroscopio
unsigned short glowx = 0x44;  //Parte baja x
unsigned short ghighy = 0x45; //Parte alta y
unsigned short glowy = 0x46;  //Parte baja y
unsigned short ghighz = 0x47; //Parte alta z
unsigned short glowz = 0x48;  //Parte baja z

// constantes auxiliares
#define PI 3.1415
#define UM 0.075    //umbral para eliminar error en integracion
#define dt  0.035   // dt de integral de giroscopio

//Variables para la calibracion del giroscopio
float sumX=0, sumY=0, sumZ=0;
// orientacion
int ori=0;

float Mag_minx;
float Mag_miny;
float Mag_minz;
float Mag_maxx;
float Mag_maxy;
float Mag_maxz;

int xhm, xlm, yhm, ylm, zhm, zlm;
double xm, ym, zm;
double xa, ya, za;
double ayx, azx, azy;
//int norte;
float Heading;
float Pitch;
float Roll;
float accxnorm, accynorm;
float magxcomp, magycomp;


void initial_sensors(){
/// CONFIGURAR LA MPU
      escribirI2C(CONFIG,0x06,ADDRMPU6050);
      escribirI2C(ACCEL_CONFIG,0x00,ADDRMPU6050); //configuracion de +-2g
      escribirI2C(GYRO_CONFIG,0x10,ADDRMPU6050); //configuracion de +-1000 °/s
      escribirI2C(PWR_MGMT_1,0x00,ADDRMPU6050);  //bits de el administrador de potencia
      delay_ms(100);
       
//CONFIGURAR EL COMPAS
       //se envian las configuraciones del sensor
      escribirI2C(CTRL_REG1_A,0x47,LSM202DLHA);  // configuracion del acelerometro
      escribirI2C(CTRL_REG4_A,0x48,LSM202DLHA);
      delay_ms(100);
      // configuracion del magnetometro
      escribirI2C(CRA_REG_M,0x98,LSM202DLHM);
      escribirI2C(CRB_REG_M,0x20,LSM202DLHM);
      escribirI2C(MR_REG_M,0x00,LSM202DLHM);
      delay_ms(100);
}

void leerSensor(unsigned short initial_address, unsigned short sensor_address){
     unsigned short int lx=0, hx=0, ly=0, hy=0, hz=0, lz=0;
     
              // LEER Y SEPARAR LOS DATOS DE LOS SENSORES
          hx = leerI2C(initial_address    , sensor_address);
          lx = leerI2C(initial_address + 1, sensor_address);
          hy = leerI2C(initial_address + 2, sensor_address);
          ly = leerI2C(initial_address + 3, sensor_address);
          hz = leerI2C(initial_address + 4, sensor_address);
          lz = leerI2C(initial_address + 5, sensor_address);

          // ORDENAR Y COMPENSAR DATOS RESPECTO AL SENSOR USADO
         if(((sensor_address) == (ADDRMPU6050))  && ((initial_address) == (ahighx)))
              {
               A1XF = (((int)hx)<<8 )  |  ((int)lx) - 950;
               A1YF = (((int)hy)<<8 )  |  ((int)ly) + 600;
               A1ZF = (((int)hz)<<8 )  |  ((int)lz) + 200;
          }
          else if(((sensor_address) == (ADDRMPU6050))  && ((initial_address) == (ghighx)))
              {
               GXF = (((int)hx)<<8 )  |  ((int)lx);
               GYF = (((int)hy)<<8 )  |  ((int)ly);
               GZF = (((int)hz)<<8 )  |  ((int)lz);
          }
          else if((sensor_address) == (LSM202DLHA))
              {
              
              A2XF =(((int)hx)<<8 )  |  ((int)lx);
              A2YF =(((int)hy)<<8 )  |  ((int)ly);
              A2ZF =(((int)hz)<<8 )  |  ((int)lz);
          }
          else if((sensor_address) == (LSM202DLHM))
          {
               MXF = (((int)hx)<<8 )  |  ((int)lx);
               MYF = (((int)hz)<<8 )  |  ((int)lz);
               MZF = (((int)hy)<<8 )  |  ((int)ly);
          }
 }
     // CALIBRAR EL GIROSCOPIO
 void calibrarG(){
     int i;
     for (i=0;i<1000;i++){
         leerSensor(ghighx,ADDRMPU6050); // LEER GIROSCOPIO

         sumX = sumX + GXF;
         sumY = sumY + GYF;
         sumZ = sumZ + GZF;
     }       // PROMEDIOS
         sumX = sumX / 1000.0;
         sumY = sumY / 1000.0;
         sumZ = sumZ / 1000.0;
}
// obtener datos de la mpu
void leer_MPU(){
                leerSensor(ahighx,ADDRMPU6050);
                leerSensor(ghighx,ADDRMPU6050);
}
// obtener datos del compas
void leer_COMPAS(){
                   leerSensor(OUT_X_H_A,LSM202DLHA);
                   leerSensor(OUT_X_H_H,LSM202DLHM);

}
// tratamiento digital de lo datos de la mpu
void datos_MPU()  {
                  AN_ZX=(A1XF*90)/(16384.0);
                  AN_ZY=(A1YF*90)/(16384.0);
                  AN_YX=(A1ZF*90)/(16384.0);
// convertir datos del sensor a °/seg
                 GXF=((GXF-sumX)*(2000/65536.0));
                 GYF=-((GYF-sumY)*(2000/65536.0));
                 GZF=((GZF-sumZ)*(2000/65536.0));
   //////////////////////////////////////
   //// condicion para cambiar el estado del grados respecto al giroscopio
   /////////////////////////////////////
                    if (fabs(GXF) > UM){
                        TX = TX0 + (dt*(GX0 + GXF)/2); }
                        else {
                             TX = TX0;
                        }
                     if (fabs(GYF) > UM){
                        TY = TY0 + (dt*(GY0 + GYF)/2);}
                        else {
                              TY = TY0;
                              }
                     if (fabs(GYF) > UM){
                        TZ = TZ0 + (dt*(GZ0 + GZF)/2);}
                         else {
                              TZ = TZ0;
                               }
   //// Sustituir variables para siguiente iteracion

                      GX0 = GXF;
                      GY0 = GYF;
                      GZ0 = GZF;

                      TX0 = TX;
                      TY0 = TY;
                      TZ0 = TZ;
}
// tratamiento digital de lo datos del compas
void datos_COMPAS(){
  Mag_minx = -542;
  Mag_miny = -592;
  Mag_minz = -330;
  Mag_maxx = 366;
  Mag_maxy = 350;
  Mag_maxz = 512;

  MXF = (MXF-Mag_minx)/(Mag_maxx-Mag_minx)*2-1;
  MYF = (MYF-Mag_miny)/(Mag_maxy-Mag_miny)*2-1;
  MZF = (MZF-Mag_minz)/(Mag_maxz-Mag_minz)*2-1;

   accxnorm = A2XF/sqrt(A2XF*A2XF+A2YF*A2YF+A2ZF*A2ZF);
   accynorm = A2YF/sqrt(A2XF*A2XF+A2YF*A2YF+A2ZF*A2ZF);

   // calcular pitch y roll
  Pitch = asin(-accxnorm);
  Roll = asin(accynorm/cos(Pitch));

  magxcomp = MXF*cos(Pitch)+zm*sin(Pitch);
  magycomp = MXF*sin(Roll)*sin(Pitch)+MYF*cos(Roll)-MZF*sin(Roll)*cos(Pitch);

  // calcular heading
  Heading = 180*atan2(magycomp,magxcomp)/PI;

  if (Heading < 0)
      Heading +=360;

  if (ya < 0 && za < 0 )
     Roll=  -(PI + Roll);

  if (ya > 0 && za < 0)
     Roll=  PI - Roll;

  if (xa < 0 && za < 0 )
     Pitch=  -(PI + Pitch);

  if (xa > 0 && za < 0)
     Pitch=  PI - Pitch;

   /*if (0.0<Heading && Heading<180.0){
             Heading=0.0;
             }
     else {
         Heading=1.0;
           }*/
}
// enviar los datos de la mpu al puerto serial
void enviar_datos_mpu(){
          sprintf(GXT, "%.2f", TX);
          sprintf(GYT, "%.2f", TY);
          sprintf(GZT, "%.2f", TZ);
          
          sprintf(AX1T, "%.2f", AN_ZX);
          sprintf(AY1T, "%.2f", AN_ZY);
          sprintf(AZ1T, "%.2f", AN_YX);
          
          /*UART2_Write_Text("AX= ");
          UART2_Write_Text(AX1T);
          UART2_Write(' ');
          UART2_Write_Text("AN1= ");
          UART2_Write_Text(AY1T);
          UART2_Write(' ');
          UART2_Write_Text("AZ= ");
          UART2_Write_Text(AZ1T);
          UART2_Write(' ');*/

         /*UART2_Write_Text("GX= ");
          UART2_Write_Text(GXT);
          UART2_Write(' ');*/
          //UART2_Write('M');
          //UART2_Write_Text("M");
          UART2_Write_Text(GYT);
          UART2_Write('M');
          //UART2_Write(' ');
          /*UART2_Write_Text("GZ= ");
          UART2_Write_Text(GZT);
          UART2_Write(' ');*/

         // UART2_Write('\n');
          //UART2_Write('\r');
          
          //UART2_Write_Text("A");
          //UART2_Write('A');
          UART2_Write_Text(AY1T);
          UART2_Write('A');
          //UART2_Write(' ');
          
}
// enviar los datos del compas al puerto serial
void enviar_datos_compas(){

     // sprintf(P, "%.2f",Pitch*(180/PI));
      sprintf(R, "%.2f",Roll*(180/PI));
      sprintf(H, "%.2f",Heading);

    /*UART2_Write_Text("P= ");
      UART2_Write_Text(P);
      UART2_Write(' ');*/
      //UART2_Write_Text("B");
      //UART2_Write('B');
      UART2_Write_Text(R);
      UART2_Write('B');
      //UART2_Write(' ');
      
      //UART2_Write_Text("C");
      //UART2_Write('C');
      UART2_Write_Text(H);
      UART2_Write('C');
      //UART2_Write(' ');

      UART2_Write('\n');
      UART2_Write('\r');

}
