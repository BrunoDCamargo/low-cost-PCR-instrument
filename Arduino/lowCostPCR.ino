//Bibliotecas
#include <PID_v1.h>     //Controle PID
#include <Thermistor.h> //Leitura do NTC
#include <stdarg.h>     //Permite que funções utilizem um número indefinido de argumentos
#include <Modbusino.h>  //Comunicação Arduino + Elipse

//Comunicação
ModbusinoSlave modbusino_slave(2); //Define o slot de comunicação como sendo o número 2
uint16_t tab_reg[25];              //Aloca 25 espaços na memória para comuniocação de dados entra Arduino e Elipse

//Pinos
//Digitais
#define WindPin 2       //Ventilador
#define TopPin 3        //Placa de Cima
#define BotPin 5        //Placa de Baixo
#define Red 6           //LED RGB R
#define Green 7         //LED RGB G
#define Blue 8          //LED RGB B
#define White 9         //White LED
#define printtime 1000  //ms
//Analógicos
#define Diode3Pin A0 //Diodo da Placa de Cima

//Valores de Ajuste Fino:

//Ajuste de Potência
#define PID1maxpwm 80 //ms //Placa de Baixo
#define PID2maxpwm 70 //ms (60 standard) //Placa de Cima

//Controle do sinal de subida
#define temp_increase_PID4_start 10//Valor utilizado para a primeira subida
#define temp_increase_PID4_cicle 2 //Valor utilizado para a subida nos ciclos
#define temp_reset_PID4_start -48  //Não Precisa Mexer
#define temp_reset_PID4_cicle -30  //Não Precisa Mexer

//Controle do Ventilador
#define temp_off_wind_PID2 1 //Desliga o Ventilador em uma temperatura x graus maior do que o valor desejado

//Valores Padrão do PID
//PID 1 - Sinal de Ativação
#define default_PID1_KP 1.75;  //KP1
#define default_PID1_KI 1.50;   //KI1
#define default_PID1_KD 6.75; //KD1
//PID 2 - Sinal de Descida
#define default_PID2_KP 10; //KP2
#define default_PID2_KI 5;  //KI2
#define default_PID2_KD 15; //KD2
//PID 3 - Sinal de Denaturação
#define default_PID3_KP 13.34;  //KP3
#define default_PID3_KI 0.325;   //KI3
#define default_PID3_KD 175.41; //KD3
//PID 4 - Sinal de Subida
#define default_PID4_KP 20; //KP4
#define default_PID4_KI 1;  //KI4
#define default_PID4_KD 8;  //KD4
//PID 5 - Sinal de Transcrição Reversa
#define default_PID5_KP 15;  //15;  //KP5
#define default_PID5_KI 1;   //5;   //KI5
#define default_PID5_KD 8;  //15; //KD5


//Valores Padrão da Ciclagem
//Ativação
#define default_T1 95 //ºC
#define default_t1 5  //min
//Denaturação
#define default_T2 60 //ºC
#define default_t2 60 //s
//Amplificação
#define default_T3 95 //ºC
#define default_t3 15 //s
//Ciclos
#define default_NBCICLE 30

//Temperatura da Placa de Cima
#define default_T_Top 120 //ºC

//Valores padrão da calibração
#define default_calibration_T_start 51    //ºC
#define default_calibration_T_diff 1      //ºC
#define default_calibration_T_interval 5 //min

//Dioco Placa de Cima
//#define diode_var_3 0.4545454 //Parâmetro a da equação linear
//#define vf0_3 573.44          //Parâmetro b da equação linear
//#define t0_3 20.3             
//Os valores acima do servem para transformar o valor de tensão do diodo em temperatura

//Estrutura PID
struct PIDVAL
{
  float PID_K[3];
  float temp;
  int wind;
  int state;
};

//Pino de Leitura do NTC
Thermistor temp(3);  //Analógico 3 NTC calibração
Thermistor tempb(1); //Analógico 1 NTC Placa de Baixo

//Set Pin Numbers:
float pin2 = 0;

//Temperatura do Diodo
int i = 0;
float t;

/*
  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, POn, Direction)
  Input: The variable we're trying to control (double)
  Output: The variable that will be adjusted by the pid (double)
  Setpoint: The value we want to Input to maintain (double)
  Kp, Ki, Kd: Tuning Parameters. these affect how the pid will change the output. (double>=0)
  Direction: Either DIRECT or REVERSE. determines which direction the output will move
  when faced with a given error. DIRECT is most common.
  POn: Either P_ON_E (Default) or P_ON_M. Allows Proportional on Measurement to be specified.
*/
double Setpoint, Input, Output;
double Setpoint2, Input2, Output2;
PID myPID(&Input, &Output, &Setpoint, 0, 0, 0, P_ON_E, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, 17.86, 0.25, 436.81, P_ON_E, DIRECT);

int WindowSize = 250;

//Contadores de tempo de execução do programa.
unsigned long windowStartTime;
unsigned long windowStartTime2;
unsigned long windowStartTime3;
unsigned long windowkStartTime4;

unsigned long windowStartTime_print;

float dtemp, dtemp_avg; //Variação de temperatura, variação média de temperatura e temperatura do diodo de Baixo
double temp_bot;
int ciclenb;         //Número do ciclo atual
float temp_top;      //Temperatura de Cima
float climbtime = 0; //Tempo de Subida
float falltime = 0;  //Tempo de Descida
bool reset_pid = 0;  //Resetar os valores PID para os defaults
int start_ = 0;      //Funciona como um contador para se saber a etapa do programa sendo executada
float temp_NTC;

int ppp = 0;
int pasc = 0;
int start;       
float temp_test; 
PIDVAL PID1;     

//Funções:

float calc_temp(); //Calcular Temperatura do Diodo
void BG(), BY();   //LED RGB
void analog_PID_temp(PIDVAL PID1);
void analog_PID_temp(float PID_KP, float PID_KI, float PID_KD, float temp);
//void digital_PID_temp(float PID_KP, float PID_KI, float PID_KD, float temp);
void cicle_temp(long time1, long time2, long time3, float temp1, float temp2, float temp3); //Função para a ciclagem
void calib_temp_cicle(long duration, float temp_dif);                                       //Função para a calibração
void analog_PID2_temp(float temp);
void resetI_PID();                       //Ciclo para resetar o PID
void executeReset();                     //Resetar todo o Programa
float KP1, KI1, KD1;                     //Constantes do PID 1
float KP2, KI2, KD2;                     //Constantes do PID 2
float KP3, KI3, KD3;                     //Constantes do PID 3
float KP5, KI5, KD5;
float T1, T2, T3, T_Top = default_T_Top; //Definição das temperaturas das etapas do processo e a da placa de cima
long T_start, T_diff;                   //Temperatura inicial da calibração e sua taxa de decréscimo
long t1, t2, t3, t_interval;             //Definição dos tempos dos ciclos e o intervalo de decréscimo da calibração
bool RUN;
int NBCICLE;       //Número de ciclos
bool calib_on = 0; //Flag para o modo de calibração
int print_count = 0;
float T0_, mean;
int x = 0, c = 0;
float z = 0;
long now2 = 0;
long meancounter2 = 0;

void setup() //Roda uma vez e define alguns parâmetros base do programa
{
  modbusino_slave.setup(19200); //Taxa de transmissão em baud rate de 19200 para comunicação serial
  pinMode(A0, INPUT_PULLUP);    //This effectively inverts the behavior of the INPUT mode, where HIGH means the sensor is off, and LOW means the sensor is on.
  pinMode(BotPin, OUTPUT);
  pinMode(WindPin, OUTPUT); //Definindo o pino do ventilador como saida
  pinMode(TopPin, OUTPUT);  //Definindo o pino da placa de cima como saida
  pinMode(Red, OUTPUT);     //Definindo o LED vermelho como saida
  pinMode(Green, OUTPUT);   //Definindo o LED verde como saida
  pinMode(Blue, OUTPUT);    //Definindo o LED azul como saida
  pinMode(White, OUTPUT);   //Definindo o LED branco como saida

  digitalWrite(BotPin, LOW);  //Iniciando a placa de baixo como desligada
  digitalWrite(WindPin, LOW); //Iniciando o ventilador como desligado
  digitalWrite(TopPin, LOW);  //Iniciando a placa de cima como desligada

  //Contadores de tempo de execução do programa em milisegundos.
  windowStartTime = millis();
  windowStartTime2 = millis();
  windowStartTime3 = 0;
  windowStartTime_print = millis();

  myPID.SetOutputLimits(0, PID1maxpwm);  //The PID controller is designed to vary its output within a given range.
  myPID2.SetOutputLimits(0, PID2maxpwm); //By default this range is 0-255: the arduino PWM range.
  Setpoint = 0;
  Setpoint2 = 0;

  //Turning the PID on
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  start = 0;
  PID1.PID_K[0] = 0;
  PID1.PID_K[1] = 0;
  PID1.PID_K[2] = 0;
  PID1.temp = 0;
  PID1.wind = 0;
  PID1.state = 0;

  ciclenb = 0;   //Número do ciclo
  temp_test = 0; //Temperatura de calibração

  digitalWrite(Red, LOW);
  digitalWrite(Green, LOW);
  digitalWrite(Blue, LOW);

  //Valores de controle do PID
  KP1 = default_PID1_KP;
  KI1 = default_PID1_KI;
  KD1 = default_PID1_KD;

  KP2 = default_PID2_KP;
  KI2 = default_PID2_KI;
  KD2 = default_PID2_KD;

  KP3 = default_PID3_KP;
  KI3 = default_PID3_KI;
  KD3 = default_PID3_KD;

  KP5 = default_PID5_KP;
  KI5 = default_PID5_KI;
  KD5 = default_PID5_KD;

  //Valores padrão de temperatura e tempo para ciclagem
  tab_reg[9] = default_T1;
  tab_reg[10] = default_T2;
  tab_reg[16] = default_T3;
  tab_reg[11] = default_t1;
  tab_reg[12] = default_t2;
  tab_reg[13] = default_t3;

  //tab_reg[14] = default_T_Top;
  tab_reg[15] = default_NBCICLE;

  //Valores de temperatura e tempo padrão para calibração
  tab_reg[19] = default_calibration_T_start;
  tab_reg[20] = default_calibration_T_diff;
  tab_reg[18] = default_calibration_T_interval;

  tab_reg[20] = 0;
  //tab_reg[7] = 1;

  tab_reg[0] = 0; //Flag de ininio de programa
  tab_reg[6] = 0; //Flag de inicio de calibração
  modbusino_slave.loop(tab_reg, 25);
}

void loop() //Roda de forma repetida, chamando todas as funções e recebendo/mandando argumentos
{
  if (tab_reg[20] == 1)
  {
    digitalWrite(White, HIGH);
  }
  else
  {
    digitalWrite(White, LOW);
  }

  //digitalWrite(White, HIGH);

  long meancounter = millis();
  if (c == 0) {
    meancounter2 = meancounter;
    c = 1;
  }
  if (meancounter >= (meancounter2 + 250)) {
    mean = mean + temp_bot;
    c = c + 1;
    meancounter2 = meancounter;
    if (c == 21) {
      tab_reg[1] = mean / 20;
      c = 0;
      mean = 0;
      meancounter2 = 0;
    }
  }


  if (tab_reg[6] == 1 && pasc != 2 && x != 1) {
    calib_on = 1;
    x = 1;
    pasc = 0;
  }

  /*if (tab_reg[6] == 1 && tab_reg[0] == 1)
    {
    //RUN = 1;
    delay(100);
    calib_on = 1;
    }*/

  else if (pasc == 2 && x == 1)
  {
    RUN = 1;
    calib_on = 0;
  }

  if (tab_reg[0] == 1 && pasc != 2) //Se receber 1 através do Elipse, inicia o programa
    RUN = 1;
  else if (tab_reg[0] == 0 && RUN == 1 || tab_reg[6] == 0 && calib_on == 1)
  { //Se receber 0 através do Elipse, reseta e para o programa
    executeReset();
    x = 0;
  }

  int xy = 0;
  if (temp_bot > 97 && xy == 0 && tab_reg[5] == 0) {
    digitalWrite(WindPin, HIGH);
    xy = 1;
  }
  else if (temp_bot > 110 && xy == 0 && tab_reg[5] == 1) {
    digitalWrite(WindPin, HIGH);
    xy = 1;
  }
  else if (xy == 1 && temp_bot < 94) {
    digitalWrite(WindPin, LOW);
    xy = 0;
  }


  //Valores de temperatura e tempo para ciclagem, recebidos do Elipse
  T1 = tab_reg[9];
  T2 = tab_reg[10];
  T3 = tab_reg[16];
  t1 = tab_reg[11];
  t2 = tab_reg[12];
  t3 = tab_reg[13];

  T_Top = default_T_Top;

  if (RUN == 0)
  {
    digitalWrite(Red, LOW);
    digitalWrite(Green, LOW);
    digitalWrite(Blue, HIGH);
  }

  /*if (RUN == 0 && tab_reg[7] == 1 && calib_on == 0)
    {
    digitalWrite(Red, LOW);
    digitalWrite(Green, HIGH);
    digitalWrite(Blue, LOW);
    }*/

  if (RUN == 1 && calib_on == 0)
  {
    BG();
  }

  if (tab_reg[6] == 1 && RUN == 0)
  {
    analogWrite(Red, 50);
    analogWrite(Green, 200);
    digitalWrite(Blue, HIGH);
  }

  /*if (calib_on == 1 && RUN == 1)
    {
    BY();
    }*/

  NBCICLE = tab_reg[15]; //Número de ciclos

  //Valores de temperatura e tempo para calibração, recebidos do Elipse
  T_start = tab_reg[19];
  T_diff = tab_reg[20];
  t_interval = tab_reg[18];

  //Calibb
  temp_top = 1.1925 * temp.getTemp() + 18 - 10 ; //Chama a função para calcular o valor de temperatura da Placa de cima

  double y = tempb.getTemp();
  double y2 = 1.1925 * tempb.getTemp() + 2.75; //Recebendo o valor de temperatura do NTC da placa de baixo

  if (y2 < 52) {
    y = 1.1925 * tempb.getTemp() + 4.25;
    y2 = -0.0298 * y * y + 4.351 * y - 93.75; //Recebendo o valor de temperatura                                do NTC da placa de baixo
    temp_bot = y2 + 1.50;
  }
  if (y2 >= 56 && y2 <= 66) {
    temp_bot = 1.1925 * tempb.getTemp() + 7.50;
  }
  if (y2 > 66) {
    temp_bot = 1.1925 * tempb.getTemp() + 13.50;
  }

  //tab_reg[1] = temp_bot;
  unsigned long now = millis(); //Flag que conta o tempo de execução em tempo real quando entra em uma função especifica

  //Ciclagem:

  if (!calib_on)                     //Se a calibração estiver desligada
  {
    if (start_ == 0)                 //Se o contador estiver em 0
    {
      if (temp_bot > tab_reg[19] + 2 && pasc != 2)             //Se a temperatura da placa de baixo for maior que 45 ºC
        digitalWrite(WindPin, HIGH); //Liga-se a ventoinha para se ter uma temepratura menor que 45 ºC
      else                           //Se a temperatura da placa de baixo for menor que 45 ºC
      { //
        start_ = 1;                  //O contador vai para 1
        digitalWrite(WindPin, LOW);  //Desliga-se a ventoinha
        PID1.wind = 0;               //Desliga-se a ventoinha
      }
    }
    if (RUN && calib_on == 0)            //Se o Programa foi Iniciando
    { //
      if (start_ > 0)                    //Se o contador for maior que 1
        analog_PID2_temp(T_Top);         //Coloca a temperatura da placa de cima em T_Top
      if (start_ == 1 && temp_top >= 92) //Após a temperatura da placa de cima alcançar a desejada
        start_ = 2;                     //O contador vai para 2
      if (start_ == 2)                                  //Se o contador for 2
      { //
        cicle_temp((t1 * 60) + t3, t2, t3, T1, T2, T3); //Entra no modo de operação de ciclagem
        analog_PID_temp(PID1);                          //Liga o PID1
      }                                                 //
    }

    //Enviando os valores para o Elipse
    //tab_reg[1] = temp_bot; //Temperatura da placa de baixo
    tab_reg[2] = temp_top; //Temperatura da placa de cima
    tab_reg[3] = temp_NTC; //Temperatura do NTC
    //tab_reg[4] = temp_bot;        //Tempo de descida
    //tab_reg[5] = temp_NTC;        //Tempo de subida
    //tab_reg[17] = PID1.temp; //SetPoint
    tab_reg[9] = T1;  //Temperatura de Ativação
    tab_reg[10] = T2; //Temperatura de Amplificação
    tab_reg[11] = t1; //Tempo de ativação
    tab_reg[12] = t2; //Tempo de amplificação
    tab_reg[13] = t3; //Tempo de denaturação
    //tab_reg[14] = T_Top;   //Valor padrão da placa de cima
    tab_reg[15] = NBCICLE; //Número de ciclos
    tab_reg[16] = T3;      //Temperatura de Denaturação
    //modbusino_slave.loop(tab_reg, 25); //Realoca os 25 espaços para comunicação de dados
  }

  else //Se a calibração estiver ligada
  {
    //Valores para o Elipse
    //tab_reg[1] = temp_bot;
    tab_reg[2] = temp_top;
    //tab_reg[14] = T_Top;
    tab_reg[18] = t_interval;
    tab_reg[19] = T_start;
    tab_reg[20] = T_diff;
    start_ = 0;

    //Enviando as constantes para o PID
    PID1.PID_K[0] = KP5;
    PID1.PID_K[1] = KI5;
    PID1.PID_K[2] = KD5;

    long nb_print = 10; //ms

    if ((start_ != 2 || ((now - windowStartTime) >= ((t_interval * 1000) - ((nb_print) * 1000))) && start_ == 2 && print_count < 10))
    {
      print_count++;
      tab_reg[3] = temp_NTC;
      //tab_reg[1] = temp_bot;
      //tab_reg[17] = PID1.temp;
    }

    analog_PID2_temp(T_Top);           //Aquecendo a placa de cima
    if (start_ == 0 && temp_top > 105) //Quando a temperatura da placa de cima aquecer
    {
      start_ = 1;                      //Contador vai para 1
    }

    if (start_ == 1)             //Se o contador estiver em 1
    { //
      temp_test = T_start;       //Temperatura inicial
      PID1.temp = temp_test;     
      analog_PID_temp(PID1);     
      if (temp_bot >= temp_test) //Se a temperatura da placa de baixo for maior ou igual a temperatura de calibração
      { //
        windowStartTime = now;   //Tempo de exceução realocado
        start_ = 2;              //Contador vai para 2
        print_count = 0;         //Zera o contador de prints
        //tab_reg[6] == 0;
      }                          //
    }

    if (start_ == 2)                        //Se o contador estiver em 2
    { //
      calib_temp_cicle(t_interval, T_diff); //Entra no ciclo de calibração
      analog_PID_temp(PID1);                
    }

    //modbusino_slave.loop(tab_reg, 25);
  }
  //Calibração
  modbusino_slave.loop(tab_reg, 25);
}

//Funções

void analog_PID2_temp(float temp) //Função de controle da temperatura da placa de cima
{
  Input2 = temp_top;            //Define o sinal a ser controlado como valor lido do diodo da placa de cima
  Setpoint2 = temp;             //Define o Setpoint 2 como o valor padrão de temperatura para a placa de cima
  myPID2.Compute();             //Realoca os valores para o PID que faz controle da placa de cima
  analogWrite(TopPin, Output2); //Define que a saida cotrolada pelo PID será escrita no pino TopPin
}

float calc_temp() //Converte a leitura analógica do diodo em temepratura
{
  dtemp_avg = 0;
  for (i = 0; i < 1024; i++)
  {
    float vf = analogRead(A0) * (4976.30 / 1023.000);
    //Serial.println(vf);
    dtemp = (vf - 573.44) * 0.4545454;
    dtemp_avg = dtemp_avg + dtemp;
  }
  t = 20.3 - dtemp_avg / 1024;

  return t;
}

void calib_temp_cicle(long duration, float temp_dif) //Ciclo de calibração
{
  BY();                                                 //Pisca cor amarela
  long now3 = millis();                         //A variavel now vai contar o tempo de execução quando a função começa
  //tab_reg[5] = now2;
  //tab_reg[4] = now3;
  if (pasc == 0 && now2 == 0 && now3 >= (60000 + 0))
  {
    now2 = (duration * 60 * 1000) + now3;
    pasc = 1;
  }
  if (temp_bot > tab_reg[19] + 1 && pasc == 1) {
    digitalWrite(WindPin, HIGH);
  }
  else {
    digitalWrite(WindPin, LOW);
  }

  if ((now3 > now2) && pasc == 1) //Quando se passar a duração definida
  {
    //windowStartTime = now;                              //O contador windowStartTime recebe o tempo atual de execução
    //temp_test -= temp_dif;                              //A temperatura desejada para calibração é decrescida de um valor estabelecido
    //print_count = 0;                                    //
    calib_on = 0;
    //tab_reg[6] = 0;
    tab_reg[0] = 1;
    pasc = 2;
    executeReset();
    //now2 = 0;
  }
  tab_reg[4] = pasc;
  tab_reg[5] = 0;
  //PID1.temp = temp_test; //Novo valor de temperatura para o PID
  modbusino_slave.loop(tab_reg, 25);
}

void analog_PID_temp(float PID_KP, float PID_KI, float PID_KD, float temp) //Essa função chama os PIDs respectivos a cada parte do programa
{
  if (calib_on)
  {
    myPID.SetTunings(PID_KP, PID_KI, PID_KD, P_ON_M);
  }
  else
  {
    if (PID1.state == 1)
    {
      myPID.SetTunings(PID_KP, PID_KI, PID_KD, P_ON_M);
    }
    else
    {
      if (PID1.state == 3)
      {
        myPID.SetTunings(PID_KP, PID_KI, PID_KD, P_ON_E);
      }
      else
      {
        myPID.SetTunings(PID_KP, PID_KI, PID_KD, P_ON_E);
      }
    }
  }

  myPID.SetTunings(PID_KP, PID_KI, PID_KD);
  Input = temp_bot;
  Setpoint = temp;
  myPID.Compute();

  analogWrite(BotPin, Output);
}

void analog_PID_temp(PIDVAL PID1) //Chama e define as constantes e a temperatura para o PID 1
{
  analog_PID_temp(PID1.PID_K[0], PID1.PID_K[1], PID1.PID_K[2], PID1.temp);
}

void cicle_temp(long time1, long time2, long time3, float temp1, float temp2, float temp3) //Ciclo principal
{

  unsigned long now = millis(); //Contador que inicia junto a função cicle_temp

  if (ciclenb < NBCICLE)        //Enquando todos os ciclos não forem executados
  {
    PIDVAL PID_1, PID_2, PID_3, PID_4;   //Definição de todos os PIDs dentro da estrutura PIDVAL
    float temp4;                                //Variavel temp4 como registrador temporário
    if (!start)                                  //Enquanto o programa não iniciar
      temp4 = temp1 + temp_increase_PID4_start; //A temperatura inicial é a temperatura desejada mais um offset que aumenta a velocidade de subida
    else
      temp4 = temp3 + temp_increase_PID4_cicle; //Offset para aumentar a velocidade

    //Valores padrão do primeiro PID (Ativação)
    PID_1.PID_K[0] = KP1;
    PID_1.PID_K[1] = KI1;
    PID_1.PID_K[2] = KD1;
    PID_1.temp = temp1;
    PID_1.state = 1;
    //Valores padrão do segundo PID (Descida)
    PID_2.PID_K[0] = KP2;
    PID_2.PID_K[1] = KI2;
    PID_2.PID_K[2] = KD2;
    PID_2.temp = temp2;
    PID_2.state = 2;
    //Valores padrão do terceiro PID (Denaturação)
    PID_3.PID_K[0] = KP3;
    PID_3.PID_K[1] = KI3;
    PID_3.PID_K[2] = KD3;
    PID_3.temp = temp3;
    PID_3.state = 3;
    //Valores padrão do quarto PID (Subida)
    PID_4.PID_K[0] = default_PID4_KP;
    PID_4.PID_K[1] = default_PID4_KI;
    PID_4.PID_K[2] = default_PID4_KD;
    PID_4.temp = temp4;
    PID_4.state = 4;


    switch (PID1.state) //Troca de PID de acordo com o ciclo em execução do programa
    {
      case 0:
        PID1 = PID_4;
        T0_ = temp_bot;
        reset_pid = 0;
        windowStartTime2 = now;
        break;
      case 1:
        if ((now - windowStartTime2) > (time1 * 1000))
        {
          PID1 = PID_2;
          windowStartTime2 = now;
          if (time2 != 0)
          {
            PID1.temp = 0;
            PID1.wind = 1;
          }
        }
        break;
      case 2:
        tab_reg[5] = 1;
        //pasc = 3;
        if (PID1.temp == 0)
        {
          if (temp_bot < (temp2 + 4.50))
          {
            PID1 = PID_2;
            PID1.wind = 0;
            falltime = (T3 - T2) / ((now - windowStartTime2) / 1000.0);
            //tab_reg[4] = falltime;
            windowStartTime2 = now;
            modbusino_slave.loop(tab_reg, 25);
          }
        }
        else
        {
          if ((now - windowStartTime2) > (time2 * 1000))
          {
            PID1 = PID_4;
            reset_pid = 0;
            windowStartTime2 = now;
            ciclenb++;
          }
        }
        break;
      case 3:
        if ((now - windowStartTime2) > (time3 * 1000))
        {
          PID1 = PID_2;
          windowStartTime2 = now;
          if (time2 != 0)
          {
            PID1.temp = 0;
            PID1.wind = 1;
          }
        }
        break;
      case 4:
        if (!start)
        {
          if (temp_bot >= temp1)
          {
            climbtime = (T1 - T0_) / ((now - windowStartTime2) / 1000.0);
            //tab_reg[5] = climbtime;
            windowStartTime2 = now;
            PID1 = PID_1;
            start = 1;
          }
          if (reset_pid == 0 && temp_bot >= (temp1 + temp_reset_PID4_start))
          {
            resetI_PID();
            reset_pid = 1;
          }
        }
        else
        {
          if (temp_bot >= temp3)
          {
            climbtime = (T3 - T2) / ((now - windowStartTime2) / 1000.0);
            //tab_reg[5] = climbtime;
            windowStartTime2 = now;
            PID1 = PID_3;
          }
          if (reset_pid == 0 && temp_bot >= (temp3 + temp_reset_PID4_cicle))
          {
            resetI_PID();
            reset_pid = 1;
          }
        }
        break;
    }

    if (PID1.wind == 1)
    {
      digitalWrite(WindPin, HIGH);
    }
    else
    {
      digitalWrite(WindPin, LOW);
    }
  }
  else
  {
    digitalWrite(Red, LOW);
    digitalWrite(Green, LOW);
    digitalWrite(Blue, HIGH);
    digitalWrite(WindPin, HIGH);
    delay(5000);
    executeReset();
  }
}

void resetI_PID() //Reseta is limites do PID
{
  myPID.SetOutputLimits(-1, 0);
  myPID.SetOutputLimits(0, -1);
  myPID.SetOutputLimits(0, PID1maxpwm);
}

void executeReset() //Ao desligar o programa, os valores principais e as saidas são colocadas em 0
{
  tab_reg[6] = 0;
  start_ = 0;
  //ciclenb = 0;
  start = 0;
  Setpoint = 0;
  PID1.wind = 0;
  PID1.PID_K[0] = 0;
  PID1.PID_K[1] = 0;
  PID1.PID_K[2] = 0;
  PID1.state = 0;
  PID1.temp = 0;
  analogWrite(TopPin, 0);
  analogWrite(BotPin, 0);
  digitalWrite(WindPin, LOW);
  temp_test = T_start;
  RUN = 0;
  calib_on = 0;
  digitalWrite(Red, LOW);
  digitalWrite(Green, LOW);
  digitalWrite(Blue, LOW);
  now2 = 0;
  pasc = 0;
  tab_reg[5] = 0;
}

void BG()
{
  digitalWrite(Red, LOW);
  digitalWrite(Green, HIGH);
  digitalWrite(Blue, LOW);
  delay(100);
  digitalWrite(Green, LOW);
  delay(100);
}

void BY()
{
  analogWrite(Red, 50);
  analogWrite(Green, 200);
  digitalWrite(Blue, LOW);
  delay(100);
  analogWrite(Red, 0);
  analogWrite(Green, 0);
  delay(100);
}

/*void digital_PID_temp(float PID_KP, float PID_KI, float PID_KD, float temp)
  {
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetTunings(PID_KP, PID_KI, PID_KD);
  Input = temp_bot;
  Setpoint = temp;
  myPID.Compute();
  /************************************************
     turn the output pin on/off based on pid output
   ************************************************/
/*unsigned long now = millis();
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  //  if (Output > now - windowStartTime) digitalWrite(BotPin, HIGH);
  //  else digitalWrite(BotPin, LOW);

  if (Output > now - windowStartTime)
    analogWrite(BotPin, 80);
  else
    analogWrite(BotPin, 80);
  Serial.println(Output);

  myPID.SetOutputLimits(0, 140);
  } */
