/* DESCRIPCION DEL PROGRAMA

  Control de Ivernadero

  Nota: Donde hay "!!" significa que queda algo pendiente
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

uint32_t Lectura, LecturaB, DATO32BIT, g_ui32SysClock, Ventilador, Valvula,Sistema ;
uint8_t DATO8BIT;
int a, b, numero_pasos = 0, timeout = 6, ;


// ****JOSHUA TEAM CODE

    // CONFIGURACON
        void apagado(void)
        {
            GPIO_PORTK_DATA_R=0x00;
        }
        void config_puertos_timers(void)
    {
        SYSCTL_RCGCGPIO_R |=SYSCTL_RCGCGPIO_R9|SYSCTL_RCGCGPIO_R3;
        SYSCTL_RCGCTIMER_R |= 0X08; //HABILITA TIMER 3
        while((SYSCTL_PRGPIO_R & 0x208 )!=0x208)

        //Puerto K (controla el motor a pasos(bits 0:4) )
        GPIO_PORTK_DATA_R |=0B00000000;
        GPIO_PORTK_DIR_R|=0Xff;
        GPIO_PORTK_DEN_R=0Xff;
        //puerto D (motor dc (bits 5:7x)) !! Ver si es importante
        GPIO_PORTD_AHB_DEN_R |= 0x10; //BIT 4 DIGITAL
        GPIO_PORTD_AHB_DIR_R |= 0x10; //bit 4 SALIDA
        GPIO_PORTD_AHB_DATA_R = 0x00; // SALIDA A 0
        GPIO_PORTD_AHB_AFSEL_R = 0x10; //FUNCION ALTERNA EN BIT 4
        GPIO_PORTD_AHB_PCTL_R = 0x00030000; //DIRIGIDO A T3CCP0
        //timer 3 (pwm para controlar la velocidad del motor dc)
        TIMER3_CTL_R=0X00000000; //DESHABILITA TIMER EN LA CONFIGURACION
        TIMER3_CFG_R= 0X00000004; //CONFIGURAR PARA 16 BITS
        TIMER3_TAMR_R= 0X0000000A; //CONFIGURAR PARA MODO PWM, MODO PERIODICO CUENTA HACIA ABAJO
        TIMER3_TAILR_R= 40000; // VALOR DE RECARGA
        TIMER3_TAPR_R=00; // RELOJ 16 MHZ
        TIMER3_TAMATCHR_R =10000; //valor para modificar la velocidad del motor dc
        TIMER3_TAPMR_R =00 ;
    }
    // Control VENTILADOR - MOTOR
        // Encendido Ventilador
        void on_ventilador(void)
            {
                //el valor del registro match es lo que controla la velocidad del motor dc
                TIMER3_CTL_R |= 0X00000041; //HABILITA TIMER A
                //GPIO_PORTK_DATA_R=0x01; // Encendido de puerto hacia un hardwere externo "LED" para verificar cambios en intensidad
                TIMER3_TAMATCHR_R =20000;
                SysCtlDelay(16000000);
                //GPIO_PORTK_DATA_R=0x02;
                TIMER3_TAMATCHR_R =25000;
                SysCtlDelay(16000000);
                //GPIO_PORTK_DATA_R=0x04;
                TIMER3_TAMATCHR_R =49000;
            }
        // Apagado Ventilador
        void off_ventilador(void)
            {
                TIMER3_CTL_R &= 0X00000000;//deshabilita el timer-pwm
                apagado();
            }
    // Control VALVULA - APERTURA Y CERRADO
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
        // Giro de la valvul a
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
                // Aviso de valvula cerrada
                UART0_AVISO_VALVULA(0);
                // Status Valvula OFF
                Valvula = 0;
                // Apagado de LED pin 4
                GPIO_PORTM_DATA_R != 0B00001000;
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
                // Aviso de valvula abierta
                UART0_AVISO_VALVULA(1);
                // Status Valvula ON
                Valvula = 1;
                // Prendido de LED pin 4
                GPIO_PORTM_DATA_R |= 0B00001000;
            }

// ****DAVID & AXEL TEAM

        // LM35 "Sensor de temperatura" -> Axel y David
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
            int ValorLectura;
            ADC0_ISC_R = 8;                   // limpiar bandera
            if(ValorADC < a)                  // 0 -> (     X < 32)
            {
                ValorLectura = 0x00;
                a = 32 + deadband;
                b = 35 + deadband;
                return ValorLectura;
            }
            if( (ValorADC >= a) & (ValorADC <= b) )   // 1 -> (32 < X < 35)
            {
                ValorLectura = 0x01;
                a = 32 - deadband;
                b = 35 + deadband;
                return ValorLectura;
            }
            if(ValorADC > b)   // 2 -> (35 < X )
            {
                ValorLectura = 0x02;
                a = 32 - deadband;
                b = 35 - deadband;
                return ValorLectura;
            }
            return ValorLectura = 0;
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
            //PORTJ Configuration
                // habilita bits 0 y 1 como digital
                // configura como entrada
                // bits 0 y 1 con pull ups
                GPIO_PORTJ_AHB_DATA_R = 0b00; // Clear data, podria omitirse para entradas
                GPIO_PORTJ_AHB_DIR_R  = 0b0000000; // 0 -> Inputs
                GPIO_PORTJ_AHB_IS_R   = 0b0000000; // Edge-Sensitive, duda si esta de mas
                GPIO_PORTJ_AHB_ICR_R  = 0b0000011; // Limpia bandera
                GPIO_PORTJ_AHB_IM_R   = 0b0000011; // Enable interruption per each pin
                GPIO_PORTJ_AHB_DEN_R  = 0b0000011; // Digital enable
                NVIC_EN1_R           |= 0x80000;   // Habilita interrupcion PORTJ -> 51
                GPIO_PORTJ_AHB_PUR_R  = 0b11;
        }

        // Configuracion de timers
        void conftmrs (void)
        {
            SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3 | SYSCTL_RCGCTIMER_R4;
            //  xxxx xxxx xxxx xxxx xxxx xxxx 0001 1000
            // Retardo para que el reloj alcance los timers
            while( (SYSCTL_PRTIMER_R & 0X018) != 0X018);
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

    // SECCIÓN DE CONTROL

    // Control para apagar todos los puertos que operan
        void ApagadoTotal (void) // !! Falta agregar más apagados
            {
                off_ventilador();
                cerrar_valvula();
                Lectura = 0;
                LecturaB = 3;
                Ventilador= 0;
                Valvula = 0;
                Sistema = 1;
            }
        void EncendidoTotal (void) // Recopilación de encendidos
            {
                setclock(16000000);
                EncenderLM35();
                Encendido_RS32();
                setports();
                conftmrs();
                config_puertos_leds();
                config_puertos_timers();
                // Establecimiento de PARAMETROS generales.
                Lectura = 0;
                LecturaB = 3;
                Ventilador= 0;
                Valvula = 0;
                Sistema = 1;
            }

    // INTERRUPCIONES DEL SISTEMA
        void TMR4 (void)  // !!
            {
                timeout = timeout - 1;
                if (timeout == 0)
                    {
                        ApagadoTotal(); // !!Apagado general del sistema
                        // Reset del timeout
                        timeout = 6;
                    }
                //LIMPIA BANDERA
                TIMER4_ICR_R= 0X00000001 ; //LIMPIA BANDERA DE TIMER4. Como se usa en modo de 32 bits se usa la parte A del registro
            }
        void HandlerJ(void)  // atiende interrupciones en J // teclado = 1
        {
                if(GPIO_PORTJ_AHB_RIS_R == 0x01)
                {
                    GPIO_PORTJ_AHB_ICR_R = 0b0000011; // Limpiar bandera
                    // Habilita
                    EncendidoTotal();
                }
                else
                {
                    GPIO_PORTJ_AHB_ICR_R = 0b0000011; // Limpiar bandera
                    // Apagado
                    ApagadoTotal();
                }
            SysCtlDelay(400000);
        }

// **************RICHARD_V01 & ARMANDO CODE *******************
    //CONFIGURACION UART0 PARA COMUNICACION RS232 - UART0 (Richard y Armando)
    void Encendido_RS32(void)
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

    // Envio de dato de caracter a la terminal
    void UART0_dato(char dato[])
    {
        while ( (UART0_FR_R&0X0020)!=0 ); // espera a que TXFF sea cero, nos indica si estamos enviando un dato
        UART0_DR_R= dato;
    }

    void IMP_NOMBRES(void)
    {
        // Retencion para esperar a impresion de caracteres pendieentes
        while ((UART0_FR_R&0X0020)!=0);
        // 0X0020 -> 10 0000
        // 10 1110 & 10 0000 -> 00 0000 != 0 -> Falso
        {
            UART0_dato('C');
            UART0_dato('a');
            UART0_dato('s');
            UART0_dato('t');
            UART0_dato('i');
            UART0_dato('l');
            UART0_dato('l');
            UART0_dato('o');
            UART0_dato(' ');
            UART0_dato('S');
            UART0_dato('a');
            UART0_dato('n');
            UART0_dato('c');
            UART0_dato('h');
            UART0_dato('e');
            UART0_dato('z');
            UART0_dato(' ');
            UART0_dato('A');
            UART0_dato('x');
            UART0_dato('e');
            UART0_dato('l');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('U');
            UART0_dato('r');
            UART0_dato('i');
            UART0_dato('b');
            UART0_dato('e');
            UART0_dato(' ');
            UART0_dato('S');
            UART0_dato('e');
            UART0_dato('r');
            UART0_dato('r');
            UART0_dato('a');
            UART0_dato('l');
            UART0_dato('d');
            UART0_dato('e');
            UART0_dato(' ');
            UART0_dato('A');
            UART0_dato('r');
            UART0_dato('m');
            UART0_dato('a');
            UART0_dato('n');
            UART0_dato('d');
            UART0_dato('o');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('F');
            UART0_dato('l');
            UART0_dato('o');
            UART0_dato('r');
            UART0_dato('e');
            UART0_dato('s');
            UART0_dato(' ');
            UART0_dato('R');
            UART0_dato('o');
            UART0_dato('j');
            UART0_dato('a');
            UART0_dato('s');
            UART0_dato(' ');
            UART0_dato('E');
            UART0_dato('d');
            UART0_dato('u');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato('d');
            UART0_dato('o');
            UART0_dato(' ');
            UART0_dato('D');
            UART0_dato('a');
            UART0_dato('v');
            UART0_dato('i');
            UART0_dato('d');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('L');
            UART0_dato('e');
            UART0_dato('o');
            UART0_dato('v');
            UART0_dato('i');
            UART0_dato('g');
            UART0_dato('i');
            UART0_dato('l');
            UART0_dato('d');
            UART0_dato('o');
            UART0_dato(' ');
            UART0_dato('M');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato('q');
            UART0_dato('u');
            UART0_dato('e');
            UART0_dato('z');
            UART0_dato(' ');
            UART0_dato('A');
            UART0_dato('n');
            UART0_dato('g');
            UART0_dato('e');
            UART0_dato('l');
            UART0_dato(' ');
            UART0_dato('E');
            UART0_dato('d');
            UART0_dato('u');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato('d');
            UART0_dato('o');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('V');
            UART0_dato('i');
            UART0_dato('l');
            UART0_dato('c');
            UART0_dato('h');
            UART0_dato('i');
            UART0_dato('s');
            UART0_dato(' ');
            UART0_dato('T');
            UART0_dato('o');
            UART0_dato('m');
            UART0_dato('a');
            UART0_dato('s');
            UART0_dato(' ');
            UART0_dato('R');
            UART0_dato('i');
            UART0_dato('c');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato('d');
            UART0_dato('o');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('A');
            UART0_dato('v');
            UART0_dato('i');
            UART0_dato('l');
            UART0_dato('e');
            UART0_dato('z');
            UART0_dato(' ');
            UART0_dato('B');
            UART0_dato('a');
            UART0_dato('h');
            UART0_dato('e');
            UART0_dato('n');
            UART0_dato('a');
            UART0_dato(' ');
            UART0_dato('A');
            UART0_dato('l');
            UART0_dato('e');
            UART0_dato('x');
            UART0_dato('a');
            UART0_dato('n');
            UART0_dato('d');
            UART0_dato('e');
            UART0_dato('r');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('B');
            UART0_dato('a');
            UART0_dato('l');
            UART0_dato('c');
            UART0_dato('a');
            UART0_dato('z');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato(' ');
            UART0_dato('A');
            UART0_dato('b');
            UART0_dato('r');
            UART0_dato('i');
            UART0_dato('l');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('A');
            UART0_dato('r');
            UART0_dato('z');
            UART0_dato('a');
            UART0_dato('t');
            UART0_dato('e');
            UART0_dato(' ');
            UART0_dato('J');
            UART0_dato('e');
            UART0_dato('s');
            UART0_dato('u');
            UART0_dato('s');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('L');
            UART0_dato('i');
            UART0_dato('n');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato('e');
            UART0_dato('s');
            UART0_dato(' ');
            UART0_dato('a');
            UART0_dato('l');
            UART0_dato('o');
            UART0_dato('n');
            UART0_dato('s');
            UART0_dato('o');
            UART0_dato(' ');
            UART0_dato('J');
            UART0_dato('o');
            UART0_dato('s');
            UART0_dato('h');
            UART0_dato('u');
            UART0_dato('a');
            UART0_dato(' ');
            UART0_dato('A');
            UART0_dato('d');
            UART0_dato('r');
            UART0_dato('i');
            UART0_dato('a');
            UART0_dato('n');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D);
            UART0_dato('C');
            UART0_dato('a');
            UART0_dato('s');
            UART0_dato('t');
            UART0_dato('i');
            UART0_dato('l');
            UART0_dato('l');
            UART0_dato('o');
            UART0_dato(' ');
            UART0_dato('G');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato('c');
            UART0_dato('i');
            UART0_dato('a');
            UART0_dato(' ');
            UART0_dato('R');
            UART0_dato('i');
            UART0_dato('c');
            UART0_dato('a');
            UART0_dato('r');
            UART0_dato('d');
            UART0_dato('o');
            UART0_dato(';');
            UART0_dato(' ');
            UART0_dato(0x0D)
        }
    }

    UART0_AVISO_VENTILADOR(int Value)
    {
        while ((UART0_FR_R&0X0020)!=0); // espera a que TXFF sea cero
        {
            if (Value == 0x01)
            {
                UART0_dato('V');
                UART0_dato('e');
                UART0_dato('n');
                UART0_dato('t');
                UART0_dato('i');
                UART0_dato('l');
                UART0_dato('a');
                UART0_dato('d');
                UART0_dato('o');
                UART0_dato('r');
                UART0_dato(' ');
                UART0_dato('E');
                UART0_dato('n');
                UART0_dato('c');
                UART0_dato('e');
                UART0_dato('n');
                UART0_dato('d');
                UART0_dato('i');
                UART0_dato('d');
                UART0_dato('o');
                UART0_dato(0x0D);

            }
            if(Value == 0x00)
            {
                UART0_dato('V');
                UART0_dato('e');
                UART0_dato('n');
                UART0_dato('t');
                UART0_dato('i');
                UART0_dato('l');
                UART0_dato('a');
                UART0_dato('d');
                UART0_dato('o');
                UART0_dato('r');
                UART0_dato(' ');
                UART0_dato('A');
                UART0_dato('p');
                UART0_dato('a');
                UART0_dato('g');
                UART0_dato('a');
                UART0_dato('d');
                UART0_dato('o');
                UART0_dato(0x0D);

            }
        }
    }

    UART0_AVISO_VALVULA(int Value)
    {
        while ((UART0_FR_R&0X0020)!=0); // espera a que TXFF sea cero
        {
            if (Value == 0x00)
            {
                UART0_dato('V');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato('v');
                UART0_dato('u');
                UART0_dato('l');
                UART0_dato('a');
                UART0_dato(' ');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato(' ');
                UART0_dato(' ');
                UART0_dato('2');
                UART0_dato('5');
                UART0_dato('%');
                UART0_dato(';');
                UART0_dato(0x0D);
            }
            if(Value == 0x01)
            {
                UART0_dato('V');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato('v');
                UART0_dato('u');
                UART0_dato('l');
                UART0_dato('a');
                UART0_dato(' ');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato(' ');
                UART0_dato(' ');
                UART0_dato('5');
                UART0_dato('0');
                UART0_dato('%');
                UART0_dato(';');
                UART0_dato(0x0D);
            }
            if (Value==0x02)
            {
                UART0_dato('V');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato('v');
                UART0_dato('u');
                UART0_dato('l');
                UART0_dato('a');
                UART0_dato(' ');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato(' ');
                UART0_dato(' ');
                UART0_dato('7');
                UART0_dato('5');
                UART0_dato('%');
                UART0_dato(';');
                UART0_dato(0x0D);
            }
            if (Value==0x03)
            {
                UART0_dato('V');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato('v');
                UART0_dato('u');
                UART0_dato('l');
                UART0_dato('a');
                UART0_dato(' ');
                UART0_dato('a');
                UART0_dato('l');
                UART0_dato(' ');
                UART0_dato('1');
                UART0_dato('0');
                UART0_dato('0');
                UART0_dato('%');
                UART0_dato(';');
                UART0_dato(0x0D);
            }
        }
    }

    // Envia mensaje a terminal con estado de temperatura
    void UART0_AVISO_TEMP(int Value)
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
            UART0_dato(0x0D);
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
            UART0_dato(0x0D);
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
            UART0_dato(0x0D);
        }
    }
}

// ***************ALEXANDER & RICHARD_V02 TEAM CODE*************

    // Configuraicon de puertos LED para indicaciones
    void config_puertos_leds(void)
        {
            GPIO_PORTM_DATA_R|=0B00000000;   // bits a cero
            GPIO_PORTM_DEN_R |=0B00011111;   // bits 5:0 digitales
            GPIO_PORTM_DIR_R |=0B00011111;   // bits 5:0 salidas (escritura)
        }

    // Control y logica del sistema
    void Control_con_leds(int led_Vent,int led_Valv,int led_T, int led_Sistema)// Entrada 4 valiores
    {
        if(led_Sistema==1)
        {
            //Si nuestro sistema esta encendido entonces:
            switch(led_T)
            {
                //Dependiendo del valor de temperatura se decidira que led indicador de temperatura se encendera en conjunto con sus diferentes casos:
                case 0: //Si led_T=0 significa que la temperatura es baja
                    // Encendido Led temperatura baja
                    GPIO_PORTM_DATA_R=0B00000001;
                    if(led_Valv==0 && led_Vent==0)
                        {
                        //Si la valvula y el ventilador estan apagados entonces:
                        // Valvula ON
                        abrir_valvula();
                        }
                    if(led_Valv==1 && led_Vent==0)
                        {//Si la valvula    esta encendida y el ventilador apagado entonces:
                        // No es necesario cambiar nada
                        }
                    if(led_Valv==0 && led_Vent==1)
                        {//Si la valvula esta apagada y el ventilador encendido entonces:
                        // Ventilador OFF
                        off_ventilador();
                        // Ventilador Message
                        // Valvula ON
                        abrir_valvula();
                        // Valvula Message ON
                        UART0_AVISO_VALVULA(1);
                        // Led ON
                        GPIO_PORTM_DATA_R|=0B00000001;
                        // Status Valvula ON
                        Valvula = 1;
                        GPIO_PORTM_DATA_R=0B00010001;//Encendemos el led de la valvula y el led de temperatura baja
                        }
                case 1: //Si led_T=1 significa que la temperatura es estable
                    GPIO_PORTM_DATA_R=0B00000010;//Prendemos el led de temperatura estable

                    if(led_Valv==0 && led_Vent==0)
                    {//Si la valvula y el ventilador estan apagados entonces:
                    }
                    if(led_Valv==1 && led_Vent==0)
                    {//Si el ventilador esta encendido y la valvula apagada entonces:
                        GPIO_PORTM_DATA_R=0B00001010;//Encendemos el led del ventilador y el led de temperatura estable
                    }
                    if(led_Valv==0 && led_Vent==1)
                    {//Si la valvula esta encendida y el ventilador apagado entonces:
                        GPIO_PORTM_DATA_R=0B00010010;//Encendemos el led de la valvula y el led de temperatura estable
                    }
                case 2: //Si led_T=2 significa que la temperatura es alta
                    GPIO_PORTM_DATA_R=0B00000100;//Solo prendemos el led de temperatura alta
                    if(led_Valv==0 && led_Vent==0)
                    {//Si la valvula y el ventilador estan apagados entonces:
                    }
                    if(led_Valv==1 && led_Vent==0)
                    {//Si el ventilador esta encendido y la valvula apagada entonces:
                        GPIO_PORTM_DATA_R=0B00001100;//Encendemos el led del ventilador y el led de temperatura alta
                    }
                    if(led_Valv==0 && led_Vent==1)
                    {//Si la valvula esta encendida y el ventilador apagado entonces:
                        GPIO_PORTM_DATA_R=0B00010100;//Encendemos el led de la valvula y el led de temperatura alta
                    }
                default://Si todo esta apagado y no se recibe lectura del sensor de temperatura, entonces:
                GPIO_PORTM_DATA_R=0B00000000;//Se apagan todos los leds externos
            }
        }
    }


/*                              PROGRAMA PRINCIPAL
 ******************************************************************************************
 */
    int main(void)
        {
            // Inincio del sistema
            EncendidoTotal();
            // Impresion de nombres
            IMP_NOMBRES();
            // Bucle infinito de operación
            while(1)
            {
                // Lectura del sensor
                Lectura = SendValue(ReadSensorValue(),6);  // Lectura -> 1   LecturaB -> 1
                // Bucle de cambio de estado
                if (Lectura != LecturaB)
                {
                    UART0_AVISO_TEMP(Lectura);
                    Control_con_leds(Ventilador,Valvula,Lectura,Sistema)
                    LecturaB = Lectura;
                }
            }
        }