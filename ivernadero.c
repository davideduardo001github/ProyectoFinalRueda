/* DESCRIPCION DEL PROGRAMA

  Control de Ivernadero

*/


/* LIBRERIAS
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/rom_map.h"

#define ValPLLFREQ0 0X00800060;
#define ValPLLFREQ1 0X00000004;


/* VARIABLES
 *
 */

uint32_t Salida, DATO32BIT, g_ui32SysClock;
uint8_t DATO8BIT;
int a, b, numero_pasos = 0, timeout = 6, SalidaB;


 //** Pendiente cuando estén todas



/* DECLARACION DE FUNCIONES GENERALES
 *
 */

    // LM35 "Sensor de temperatura"
        void EncenderLM35(void)
        {
            // Modificadores de frecuencia de reloj
            SYSCTL_PLLFREQ1_R = ValPLLFREQ1;
            SYSCTL_PLLFREQ0_R = ValPLLFREQ0;
            while( (SYSCTL_PLLSTAT_R & 0x01) == 0);

            // Habilitar reloj para los puertos utilizados
            SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGCGPIO_R4;   // Reloj para puerto de entrada E
            SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;               // Reloj para el adc
            while( (SYSCTL_PLLSTAT_R & SYSCTL_RCGCADC_R0) == 0); // Prueba de encendido

            // Inicializar Puerto E pin 4 para que sea AIN0

            GPIO_PORTE_AHB_AMSEL_R |= 0X10; // Habilita modulo analogico
            GPIO_PORTE_AHB_DEN_R &= ~0X10;  // Desahabilita buffer digital
            GPIO_PORTE_AHB_PDR_R |= 0X10; // Resistencia Pull Down
            //1110 1111 = 1111 1111 & 1110 1111
            // INICIALIZAR ADC

            ADC0_SSPRI_R = 0X00003210 ;   //se queda con las mismas prioridades
            ADC0_PC_R = 0x00;             //1Ms/S
            // * Modifique valor a ver que pasa, estaba en 0x07
            ADC0_ACTSS_R = 0;             //durante la configuracion
            ADC0_EMUX_R = 0X0000;         //inicio de la conversion por software con bit SSn en el ADCPSSI,
            ADC0_SSEMUX3_R = 0;           //obtener entrada del primer grupo de 16 canales (0 al 15)
            ADC0_SSMUX3_R = 9;            //obtener entrada del canal 9
            ADC0_SSCTL3_R |= 2;           //poner bandera para terminar a la 1a muestra
            ADC0_ACTSS_R |= 8;            //habilitar secuenciador 3 del ADC0
            SYSCTL_PLLFREQ0_R = 0;        //DESHABILITA EL PLL
            SYSCTL_PLLFREQ1_R= 0;
        }

        int ReadSensorValue(void)
        {
            int Lectura;
            ADC0_PSSI_R |= 8;             // EMPEZAR SECUENCIA DE CONVERSION 3
            while(ADC0_RIS_R & 8 == 0);   // Espera por conversion completa
            // Conversion a grados Celcius
            Lectura = ADC0_SSFIFO3_R*.0758+1;   // Leer el resultado de la conversion
            return Lectura;
        }

        int SendValue(int ValorADC,int deadband)
        {
            int ValorSalida;
            ADC0_ISC_R = 8;                   // limpiar bandera
            if(ValorADC < a)                  // 0 -> (     X < 32)
            {
                ValorSalida = 0x00;
                a = 32 + deadband;
                b = 35 + deadband;
                return ValorSalida;
            }
            if( (ValorADC >= a) & (ValorADC <= b) )   // 1 -> (32 < X < 35)
            {
                ValorSalida = 0x01;
                a = 32 - deadband;
                b = 35 + deadband;
                return ValorSalida;
            }
            if(ValorADC > b)   // 2 -> (35 < X )
            {
                ValorSalida = 0x02;
                a = 32 - deadband;
                b = 35 - deadband;
                return ValorSalida;
            }
            return ValorSalida = 0;
        }

    // FUNCIONES DE MOTOR Y VÁLVULA
        // Secuencia de motor de pasos 1
        void pasos_1(void)
            {
                GPIO_PORTK_DATA_R =0X1c;
                SysCtlDelay(40000);
                GPIO_PORTK_DATA_R =0X16;
                SysCtlDelay(40000);
                GPIO_PORTK_DATA_R =0X13;
                SysCtlDelay(40000);
                GPIO_PORTK_DATA_R =0X19;
                SysCtlDelay(40000);
            }

        // Secuencia de motor de pasos 2
        void pasos_2(void)
            {
                GPIO_PORTK_DATA_R =0X19;
                SysCtlDelay(40000);
                GPIO_PORTK_DATA_R =0X13;
                SysCtlDelay(40000);
                GPIO_PORTK_DATA_R =0X16;
                SysCtlDelay(40000);
                GPIO_PORTK_DATA_R =0X1c;
                SysCtlDelay(40000);
            }

        // Apagado del motor
        void apagado(void)
            {
                GPIO_PORTK_DATA_R=0x00;
            }

        // Giro de la valvula
        void giro_1(np)
            {
                while(np>numero_pasos)
                {
                    pasos_1();
                    numero_pasos=numero_pasos+1;
                }
                numero_pasos=0;
                apagado();
            }

        void giro_2(np)
            {
                while(np>numero_pasos)
                {
                    pasos_2();
                    numero_pasos=numero_pasos+1;
                }
                numero_pasos=0;
                apagado();
            }

        // Apagado válvula
        void cerrar_valvula()
            {
                giro_1(128);
                apagado();
            }

        // Apertura valvula
        void abrir_valvula()
            {
                giro_2(32);
                GPIO_PORTK_DATA_R =0X10;
                SysCtlDelay(5330000);
                giro_2(32);
                GPIO_PORTK_DATA_R =0X10;
                SysCtlDelay(5330000);
                giro_2(32);
                GPIO_PORTK_DATA_R =0X10;
                SysCtlDelay(5330000);
                giro_2(32);
                GPIO_PORTK_DATA_R =0X10;
                apagado();
            }

    // CONFIGURACION DEL SISTEMA
        // Establecimiento de la frecuencia del reloj CPU
        void setclock (int timeClock) // Entrada de la frecuencia objetivo
           {
               //la funcion SysCtlClockFreqSet establece la frecuencia de operacion del CPU . en este caso solicitaremos
               //que el reloj de 25 MHz sea usado como fuente para el reloj MAIN con el PLL y que
               //sea usado para el VCO, que estar a 480 MHz
               //dandonos una frecuencia final de 50,000,000 Hz. la funcion regresa un resultado
               //asi que siempre podemos saber la frecuencia real a la que opera el reloj.
               g_ui32SysClock = MAP_SysCtlClockFreqSet(( SYSCTL_XTAL_25MHZ |
                                                            SYSCTL_OSC_MAIN |
                                                            SYSCTL_USE_PLL |
                                                            SYSCTL_CFG_VCO_480), timeClock);
           }

        // Configuracion de puertos
        void setports (void)
        {
            //  Habilitar los puertos N,F,J
            //  XQPN MLKJ HGFE DCBA
            //  0001 0001 0010 0000    bits 12,8 y 5
            SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12 | SYSCTL_RCGCGPIO_R8 | SYSCTL_RCGCGPIO_R5;
            while( ( SYSCTL_PRGPIO_R & ( SYSCTL_PRGPIO_R12 | SYSCTL_PRGPIO_R8 | SYSCTL_PRGPIO_R5 ) )!= ( SYSCTL_PRGPIO_R12 | SYSCTL_PRGPIO_R8 | SYSCTL_PRGPIO_R5 ) );
                // CONFIGURACION DE PUERTOS
                    //PORT D
                    //PORT E
                    //PORT K
                    //PORT J
                    //PORT M
        }

        // Configuracion de timers
        void conftmrs (void)
        {
            SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3 | SYSCTL_RCGCTIMER_R4;
            //  xxxx xxxx xxxx xxxx xxxx xxxx 0001 1000
            // Retardo para que el reloj alcance los timers
            while( (SYSCTL_PRTIMER_R & 0X018) != 0X018);

                // CONFIGURACON TIMER 3 -> JOSHUA

                //CONFIGURACION TIMER 4 MODO 32 BITS para cuenta regresiva
                    TIMER4_CTL_R=0X00000000;        // DESHABILITA TIMER EN LA CONFIGURACION
                    TIMER4_CFG_R= 0X00000000;       // CONFIGURAR PARA 32 BITS
                    TIMER4_TAMR_R= 0X00000002;      // CONFIGURAR PARA MODO PERIODICO CUENTA HACIA ABAJO
                    TIMER4_TAILR_R= 1600000000;     // VALOR DE RECARGA A 100 SEGUNDOS, por lo que la cuenta se harà 6 vecesd
                    TIMER4_ICR_R= 0X00000001;       // LIMPIA POSIBLE BANDERA PENDIENTE DE TIMER4
                    TIMER4_IMR_R |= 0X00000001;     // ACTIVA INTERRUPCION DE TIMEOUT
                    NVIC_EN1_R |= 0X80000000;       // HABILITA LA INTERRUPCION DE  TIMER4 BIT 63 GLOBAL
                    TIMER4_CTL_R |= 0X00000001;     // HABILITA TIMER EN LA CONFIGURACION
            }

        // Control para apagar todos los puertos que operan
        void ApagadoTotal (void)
            {
                apagado();
            }

    // INTERRUPCIONES DEL SISTEMA
        void TMR4 (void)
            {
                timeout = timeout - 1;
                if (timeout == 0)
                    {
                        // Apagar motores
                        // Aquí va Joshua Code
                        ApagadoTotal();

                        // Reset del timeout
                        timeout = 6;
                    }
                //LIMPIA BANDERA
                TIMER4_ICR_R= 0X00000001 ; //LIMPIA BANDERA DE TIMER4. Como se usa en modo de 32 bits se usa la parte A del registro
            }

//CONFIGURACION UART0 PARA COMUNICACION RS232 - UART0
Encendido_RS32(void)
    {
        // Inicializacion de sistema

            SYSCTL_RCGCUART_R |=0X0001; //HABILITAR UART0
            SYSCTL_RCGCGPIO_R |=0X0001; //HABILITAR PUERTO A
            DATO32BIT = SYSCTL_RCGCGPIO_R; //RETARDO PARA HABILITAR PERIFERICOS
            //ESTO DEBIDO A QUE EL PUERTO A EN ESPECIAL LOS BITS PA0  Y PA1 SE PUEDEN CONFIGURAR
            //PARA QUE SU FUNCION ALTERNA SE LA DE, PARA PA0 LA TERMINAL Rx Y PARA PA1 LA TERMINAL TX DEL UART0

        // Configuracion del sistema

            // CONFIGURACION UART
                UART0_CTL_R &=~0X0001;  //DESHABILITAR UART para configuracion
                //Se configura la velocidad deseada (115200 Bauds)
                UART0_IBRD_R = 8; //IBDR=int(16000000/16*115200))= int(8.6805) DONDE 115200 es la velocidad a la que quiero transmitir
                //Y 16 MHz es la frecuencia de reloj, se deja la parte entera que en este caso es 8
                //Quiero transmitir datos a esta velocidad ya que es la velocidad que se coloco en la terminal y tanto la terminal
                //como el UART0 deben estar a la misma velocidad
                UART0_FBRD_R =44 ; //FBRD= round(0.6805*64 =44) //Aqui los decimales*64 redondeado al sig mas proximo que es 44
                //Esto es debido a que estas operaciones las especifica el fabricante
                UART0_LCRH_R =0X0060; //8 BITS, DESHABILITAR FIFO
                UART0_CTL_R= 0X0301 ; //HABILITAR RXE, TXE Y UART

            // CONFIGURACION PUERTO A
                GPIO_PORTA_AHB_PCTL_R = (GPIO_PORTA_AHB_PCTL_R & 0XFFFFFF00) + 0X00000011;
                //Aqui indicamos al registro PCTL  la funcion alterna y encendemos la configuracion que van a tener los bits PA0 y PA1, donde si coloco un uno en ambos bits de su registro PCTL
                //estoy indicando que van a ser la terminal de recepcion y transmision del UART0 repectivamente.
                GPIO_PORTA_AHB_AMSEL_R &= ~0X03; //DESHABILITAR FUNCION ANALOGICA EN PA0 y PA1
                GPIO_PORTA_AHB_AFSEL_R |=  0X03; //HABILITAR FUNCION ALTERNA EN PA0 y PA1
                GPIO_PORTA_AHB_DEN_R |= 0X03; //HABILITAR FUNCION I/O DIGITAL en los pines PA0 y PA1
    }

//COMO AUN NO JUNTAMOS CODIGO CON JOSHUA Y DAVID SE PROBARA LA TRANSMISION DE DATOS DE LA TERMINAL  CON LOS
//BITS, PJ0 Y PJ1
INIC_PORTS(void)
{
    SYSCTL_RCGCGPIO_R=SYSCTL_RCGCGPIO_R | 0x0100;
    GPIO_PORTJ_AHB_DATA_R=0x00;
    GPIO_PORTJ_AHB_DIR_R=0x00;
    GPIO_PORTJ_AHB_DEN_R=0x03;
}



//SE COMENTA ESTO DE MOMENTO YA QUE POR AHORA NO ES NECESARIO LEER DATOS DE LA TERMINAL PARA EL FIN DEL PROYECTO
/*char UART_Lee_dato(void)
{
    while((UART0_FR_R&0X0010)!=0); //ESPERAR A QUE RXFE SEA CERO
{
        DATO8BIT=((char)(UART0_DR_R&0xff));
        return((char)(UART0_DR_R&0xff));
}
*/
void UART0_dato(char dato[])
{
    while ( (UART0_FR_R&0X0020)!=0 ); // espera a que TXFF sea cero, nos indica si estamos enviando un dato
    UART0_DR_R= dato;
}

IMP_NOMBRES(void)
{
    while ((UART0_FR_R&0X0020)!=0);
    {

    }
}

UART0_AVISO_TEMP(int Value)
{
    while ((UART0_FR_R&0X0020)!=0); // espera a que TXFF sea cero
    {
        if (Value == 0x00)
        {
            UART0_dato('T');
            UART0_dato('e');
            UART0_dato('m');
            UART0_dato('p');
            UART0_dato('e');
            UART0_dato('r');
            UART0_dato('a');
            UART0_dato('t');
            UART0_dato('u');
            UART0_dato('r');
            UART0_dato('a');
            UART0_dato(' ');
            UART0_dato('b');
            UART0_dato('a');
            UART0_dato('j');
            UART0_dato('a');
            UART0_dato(';');
            UART0_dato(' ');
        }
        if(Value == 0x01)
        {
            UART0_dato('T');
            UART0_dato('e');
            UART0_dato('m');
            UART0_dato('p');
            UART0_dato('e');
            UART0_dato('r');
            UART0_dato('a');
            UART0_dato('t');
            UART0_dato('u');
            UART0_dato('r');
            UART0_dato('a');
            UART0_dato(' ');
            UART0_dato('i');
            UART0_dato('d');
            UART0_dato('e');
            UART0_dato('a');
            UART0_dato('l');
            UART0_dato(';');
            UART0_dato(' ');
        }
        if (Value==0x02)
        {
            UART0_dato('T');
            UART0_dato('e');
            UART0_dato('m');
            UART0_dato('p');
            UART0_dato('e');
            UART0_dato('r');
            UART0_dato('a');
            UART0_dato('t');
            UART0_dato('u');
            UART0_dato('r');
            UART0_dato('a');
            UART0_dato(' ');
            UART0_dato('a');
            UART0_dato('l');
            UART0_dato('t');
            UART0_dato('a');
            UART0_dato(';');
            UART0_dato(' ');
        }
    }
}


/* PROGRAMA PRINCIPAL
 *
 */
    int main(void)
        {
            setclock(16000000);
            EncenderLM35();
            Encendido_RS32();
            INIC_PORTS();
            IMP_NOMBRES();
            // Bucle infinito de operación
            Salida = 0;
            SalidaB = 3;
            while(1)
            {
                Salida = SendValue(ReadSensorValue(),6);  // Salida -> 1   SalidaB -> 1
                if (Salida != SalidaB)
                {
                    UART0_AVISO_TEMP(Salida);
                    SalidaB = Salida;
                }
            }
        }
