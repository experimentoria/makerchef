/*
*
*  SousVide com display 16 x 2 
*
*  Algoritmo de sous vide adaptativo
*
*  See http://www.instructables.com/id/Cheap-and-effective-Sous-Vide-cooker-Arduino-power/ para mais informações
*
*  Author : Etienne Giust - 2013, 2014
*  Adaptação: Gustavo Guimaraes Forster, Gustavo Forster - 2016
*
*  Características
*
*  - Funciona fora da caixa: não precisa alustes ou afinação, o software adapta-se as caracteristicas de sua panela: seja ela grande ou pequena, cheia de agua ou pela metade
*  - se a temperatura amabiente estiver baixa ou alta, ela funciona
*  - Regulação eficiente na faixa de +/- 0.5°C
*  - Soa um alarme quando a temperatura desejada é atingida
*  - Detecção automatica se a tampa for aberta e fechada: a regulação não endoidece quando o sensor de temperatura é trirado da agua(o que é algo que você tem que fazer se desejar por comida na panela)
*  - Características de segurança: 
*     - deslida automaticamente se o aquecimento não alterar a temperatura por 5 minutos
*     - desliga automaticamente após 24 horas de operação
*     - desliga automaticamente se a temperatura atingir 95 °C
*     - permite temperatura de operação apenas na faixa de 50°C a 90°C 
*  - Construção simples e barata
*
*
*  Licença
*  
*  Direitos autorais (C) 2014  Etienne Giust
*
*  Este programa é um software livre: você pode redistribur ou modica-lo
*  sob os termos da GNU Affero General Public License como publicado
*  pela The Free Software Foundation, seja a versão 3 da licença, ou
*  (por opção sua) qualquer versão postrior.
*
*  Este programa é distribuido na esperança de ser útil,
*  mas sem NENHUMA GARANTIA; sem mesmo a garantia implicita de 
*  MERCABILIDADE ou para qualquer PROPÓSITO PARIRTICULAR.  Veja
*  a Licença Púplica da GNU Affero para maiores detalhes.
*
*  Você deve ter recebido uma cópia da GNU Affero General Public License
*  junto com este p´rograma.  Se não, veja em  <http://www.gnu.org/licenses/>.
*
*
*/

// ------------------------- PARTES NECESSÁRIAS

// Placa Arduino 
// Display de 2 linhas de 16 caractéres 
// Botões de pressão x 4
// Elemento Piezo ou autofalante  
// Sensor dital de temperatura a prova d'água DS18B20
// Resistor de 4.7k ohm
// Potenciometro 10k ohm  
// Módulo de relé de 5Vcc carga 125/250Vac  10 
// Panela de arroz

// ------------------------- PINAGEM
//
// Botão + no pino 6 com modo INPUT_PULLUP, outro lado do botão em GND 
// Botão - no pino 5 com modo INPUT_PULLUP, outro lado do botão em GND
// Sensor de temperatura no pino 9 (pino de dados do sensor OneWire)
// Relé no pino 8
// Autofalante (piezo) no pino 13
// LCD pino RS  to digital pin 12
// LCD pino 1 (VSS) em GND
// LCD pino 2 (DDD)em +5Vcc
// LCD pino Enable (E) na entrada digital 11
// LCD pino D4 na entrada digital 5
// LCD pino D5 na entrada digital 4
// LCD pino D6 na entrada digital 3
// LCD pino D7 na entrada digital 2
// Potenciometro 10k ohm pino 1 em +5Vcc 
// Potenciometro 10k ohm pino 2 no pino VO do LCD (pino 3)
// Potenciometro 10k ohm pino 3 em GND
// Resistor 220 ohm no pino 15 do LCD e a outra ponta do resistor em +5Vcc
// LCD pino 16 em GND

// ------------------------- TIMER

unsigned long Watch, _micro, time = micros();
unsigned int Clock = 0, R_clock;
boolean Reset = false, Stop = false, Paused = false, _type;
volatile boolean timeFlag = false;

#define DOWN 0
#define UP 1

// ------------------------- BIBLIOTECAS
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>


// ------------------------- CONSTANTES

// botoões de pressão
#define BT_TEMP_MORE_PIN 6 //INPUT_PULLUP mode
#define BT_TEMP_LESS_PIN 7 //INPUT_PULLUP mode
#define BT_TIME_MORE_PIN A0 //INPUT_PULLUP mode
#define BT_TIME_LESS_PIN A1 //INPUT_PULLUP mode
#define BT_TIMER_PIN A2 //INPUT_PULLUP mode

// piezo
#define PIEZO_PIN 13

// sensor de temperatura
#define ONE_WIRE_BUS 8
#define TEMPERATURE_PRECISION 8
#define SAMPLE_DELAY 5000
#define OUTPUT_TO_SERIAL true

// relé
#define RELAY_OUT_PIN 9


// FPrimeira Rampa
#define FIRST_RAMP_CUTOFF_RATIO 0.65

// Características de segurança
#define MIN_TARGET_TEMP 50   /*suficiente para a maioria das receitas de sous vide*/
#define MAX_TARGET_TEMP 90   /*suficiente para a maioria das receitas de sous vide*/
#define SHUTDOWN_TEMP 95   /*desliga se atingir esta temperatura*/
#define MAX_UPTIME_HOURS 24   /*desliga após 24 hours de operação*/
#define MAX_UPTIME_MINS 10
#define MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES 5   /*detecta quando o sensor não está na agua e previne sobre aquecimento*/

// regulação
#define MIN_SWITCHING_TIME 1500  /*tempo mínimo do elemento de aquecimento ligado*/
#define DROP_DEGREES_FOR_CALC_REGULATION 0.12 /*queda mínima de temperatura em graus usada para calcular os tempos de regulação (deve ser: <0.2 ) */
#define LARGE_TEMP_DIFFERENCE 1  /* para maior que "1" grau, usa ajuste "LARGE" (do contrário o "SMALL")*/

// ------------------------- DEFINIÇÔES E INICIALIZAÇÔES

// botões
int sw_tempMore;
int sw_tempLess;
int sw_timeMore;
int sw_timeLess;
int sw_timer;
bool apertouBotao=false;

// temperaturas
double environmentTemp = 0;
double actualTemp = 0;
double targetTemp = 0;
double storedTargetTemp = 0;
double initialTemp = 0;
double firstRampCutOffTemp = 0;
double maxRegTEmp = 0;
double minRegTEmp = 0;
double tempBeforeDrop = 0;
double tempBeforeHeating = 0;
double parametersRegulationSetForTemp = 0;
double actualTempAtBoostStart = 0;
double expectedTempChange = 0;
double tempPreviousArray[6]= {0, 0, 0, 0, 0, 0};

// derivadas
double currentTempDerivative;
double previousDerivative;

// ganhos
double secondPerDegreeGainRef = 0;
double secondPerDegreeGainLarge = 0;
double secondPerDegreeGainSmall = 0;

// booleanas & estados
bool isNewSample = false;
boolean isWaitingForTempAlert = false;
boolean waitForSuddenRise = false;
boolean isDerivativeReliable = false;
boolean waitingForStabilization = false;
boolean doBackToFirstRampWhenStabilizing = false;
boolean isHeatOn = false;
boolean isCounteracting = false;
enum operatingState { INITIAL_WAIT = 0, TEMP_DROP, TEMP_RISE, FIRST_RAMP, BOOST_TEMP, COUNTER_FALL, WAIT_NATURAL_DROP, REGULATE};
operatingState opState = INITIAL_WAIT;
enum boostTypes {HIGHBOOST = 0, LOWBOOST};
boostTypes boostType = HIGHBOOST;
int warningsBeforeCounterFall;

// temporizações

// VARIAVEIS DO TIMER
unsigned int targetTime = 40;

// =====================================
unsigned long tcurrent = 0;
unsigned long previousMillis = 0;             
unsigned long tprog = 0;
unsigned long tStartFirstRamp = 0;
unsigned long tStartBoostTemp = 0;
unsigned long tStartRealRegulation = 0;
unsigned long tFirstRampCutOff = 0;
unsigned long tEndFirstRamp = 0;
unsigned long tOperationalDelay = 0;
unsigned long burnupTime = 0;
unsigned long tMinReg = 0;
unsigned long tMaxReg = 0;
unsigned long tLastTurnOffRelay = 0;
unsigned long durationOnPulse = 0;
unsigned long durationOffPulse = 0;
unsigned long tGetTemperatureSample  = 0;
unsigned long tCheckStabilize  = 0;
unsigned long tCheckTakeOff = 0;
unsigned long tBackToLow = 0;
unsigned long tBackToHigh = 0;
unsigned long delaytime=100;

// variaveis de segurança
unsigned long  maxUptimeMillis;
unsigned long  tCheckNotHeatingWildly;

// variaveis de sensor de temperatura e display

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Configura uma instância do sensor OneWine e do sensor Dallastemperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);  
// variavel para armazenar o endereço do sensor de temperatura
DeviceAddress tempProbeAddress; 




void setup() {
  
  lcd.begin(16,2); // Inicializa a tela do  LCD 
  
  Serial.begin(9600);
 
  /*
  Initializa os botões
  */
  pinMode(BT_TEMP_MORE_PIN, INPUT_PULLUP);
  pinMode(BT_TEMP_LESS_PIN, INPUT_PULLUP);
  pinMode(BT_TIME_MORE_PIN, INPUT_PULLUP);
  pinMode(BT_TIME_LESS_PIN, INPUT_PULLUP);
  pinMode(BT_TIMER_PIN, INPUT_PULLUP);
  /*
  Initializa o sensor de temperatura
  */
  sensors.begin();
  delay(1000);
  sensors.getAddress(tempProbeAddress, 0);  
  delay(1000);   
  sensors.requestTemperaturesByIndex(0); // envia o comando requisitando a temperatura
  delay(1000);
  /*
  Lê a temperatura
  */
  actualTemp =  sensors.getTempC(tempProbeAddress);
  targetTemp = (long) ((int)actualTemp);


    /*
  Escreve valores iniciais no display
  */
  displayActualTemp(actualTemp);
  displayTargetTemp(targetTemp);
  displayTargetTime(targetTime);
  
  //prepara a porta do relé para escrita
  pinMode(RELAY_OUT_PIN, OUTPUT);  
  digitalWrite(RELAY_OUT_PIN,LOW);

  tcurrent = millis();
  maxUptimeMillis = MAX_UPTIME_HOURS * (unsigned long)3600 * (unsigned long)1000;
  
  // Estado inicial 
  warningsBeforeCounterFall = 3;
  opState = INITIAL_WAIT;

  delay(3000);
}

/**************************************************************************************/
/*                                                                                    */
/*                                      Laço principal                                     */
/*                                                                                    */
/**************************************************************************************/


void loop() {   

  timer();
  
  tcurrent = millis();
     
  // lê a temperatura cada poucos segundos e envia para saída serial se necessário. Alerta quando está dentro da faixa
  GetTemperatureAndEnforceSecurity();
  // computa a derivada da temperatura atul
  SetActualDerivative();
  
  
  switch (opState)
   {
  case INITIAL_WAIT:      
    // espera a estabilidade da temperatura inicial
    if (abs(actualTemp - tempPreviousArray[1] ) < 0.1)
    {   
      if (environmentTemp == 0)
      {       
        // armazena a temperatura inicial, mas não mais que 30 graus
        environmentTemp = min(actualTemp, 30);      
      }
      // verifica se a temperatura desejada esta dentro da faixa aceitável e se estiver muda para primeira rampa
      if(targetTemp >= MIN_TARGET_TEMP)
      {
        StartInitialRamping();        
      }
    }   
    break;
    
  case TEMP_DROP:
    // espera pela estabilização ou aumento repentino
    if (waitForSuddenRise == false && IsStabilizing())
    {
      if (abs(actualTemp - environmentTemp) < abs(actualTemp - tempBeforeDrop))
      {
        // estamos perto da temperatura do ambiente. O sensor está provavelmente fora d'água; espera atá a temperatura subir rapidamente e estabilizar
        waitForSuddenRise = true;
        
        Serial.println("Temperatura caindo : espera subir");
      } else {
        // alguma coisa muito fria foi introduzida na agua; ou não. De qualquer modo, o sensor de temperatura voltou, vamos regular               
        if (doBackToFirstRampWhenStabilizing)
        {
          Serial.println("Temperatura subindo : rampa inicial");
          opState = FIRST_RAMP;
        } 
        else 
        {       
          Serial.println(" Temperatura caindo : frio ! regular");
          EnterRegulateStateOrWaitSmoothLowering();
        }
      }
    }
    WatchForTempFalling();
    break;
    
  case TEMP_RISE:
    // espera pela estabilização, então regula
    if ( IsStabilizingOrDropping() )
    {     
      if (doBackToFirstRampWhenStabilizing)
      {
        Serial.println(" Temperatura aumentando : de volta a rampa inicial");
        opState = FIRST_RAMP;
      } 
      else 
      { 
        Serial.println(" Temperatura subindo : de volta ao normal : regular");
        EnterRegulateStateOrWaitSmoothLowering();
      }
    }
    WatchForTempFalling();
    break;
    
  case FIRST_RAMP:
    PerformFirstRamp();
    break;   

  case COUNTER_FALL:  
    // Condição inicial : temperatura bem abaixo da meta && importante derivada negativa , mas não em queda livre : -0.1 < d < -0.01,  3 vezes em uma fila
  
    // Ligado, até derivada == 0 então corta e espera estabilização
    if (isNewSample) 
    {   
      Serial.println("Verificação Counterfall");
      if (waitingForStabilization == false) 
      {     
        // verifica a derivada
        // se (isDerivativeReliable && currentTempDerivative > -0.005)
        double predicted = predictTemp(tOperationalDelay) ;
        Serial.print(" temperatura prevista : ");
        Serial.println(predicted);
        
        if ( predicted >= (targetTemp - 1)  && isDerivativeReliable && currentTempDerivative > 0.001)  // targetTemp - 1 is to avoid overshoot because prediction is not precise enough
        {
          Serial.println(" desliga relé !");
          turnOffRelay();
          waitingForStabilization = true;
        }         
      } 
      else 
      {
        if ( IsStabilizingOrDropping() )
        {
          Serial.println(" Contador de queda terminado : regular");
          
          //reset counter
          warningsBeforeCounterFall = 3;
          EnterRegulateStateOrWaitSmoothLowering();
        }
        if( isDerivativeReliable && currentTempDerivative < -0.005)
        {
          turnOnRelay();
          waitingForStabilization = false;
        }       
      }   
    }
    break;    
  case BOOST_TEMP:    
    PerformBoostTemp();
    WatchForTempFalling();
    break; 


  case WAIT_NATURAL_DROP:
    if (isNewSample) 
    { 
      // quando a temperatura está próxima o suficiente da desejada, tenta calcular os valores de regulação se eles não estiverem já ajustados
      if (isCounteracting == false && parametersRegulationSetForTemp != targetTemp && abs(actualTemp - targetTemp) < 3 )
      {
        PerformRegulationCalculations();
      } 
      
      // predict temp at t + tOperationalDelay
      double futureTemp = predictTemp(tOperationalDelay); 
      // contra medida para estabilizar perto da temperatura desejada
      if (isCounteracting == false && futureTemp < targetTemp)
      {
        isCounteracting = true;
        HeatForDegrees(actualTemp - futureTemp);
      }
      // verifica estabilização
      if ( ((long) (millis() - tCheckStabilize) >= 0) && isCounteracting )
      {
        if(IsStabilizingOrGrowing())
        { 
          Serial.println("Queda natural terminada: esperando estabilizar");         
          opState = TEMP_RISE; // garante que estamos estabilizados antes de regular novamente
        } 
        
        if(IsAcceleratingFall())
        {
          Serial.println("Queda: tentar de novo!");
          isCounteracting = false;
        }     
      }
      // caiu muito
      if (actualTemp < targetTemp - 0.1)
      {
        StartBoostToTarget(); 
      }
    } 
    WatchForTempFalling();
    break;  
  case REGULATE:
    Regulate();
    WatchForTempFalling();
    break;  
   }
   
   if (opState != FIRST_RAMP && opState != COUNTER_FALL)
   {
    // verifica o tempo que o relé precisa para desligar (exceto durante a rampa initial ou contra medida)
    if ( (long) (millis() - tBackToLow) >= 0)
    {
      turnOffRelay();
    }
   }
   
  // Lê o estado dos botões
  readButtonInputs();
  
  // atualiza display
    
  
  displayActualTemp(actualTemp);
  displayTargetTemp(targetTemp);
  displayTargetTime(targetTime);
   
  // pause loop
  delay(delaytime); 
}




/**************************************************************************************/
/*                                                                                    */
/*                                  FUNÇÔES AUXILIARES                                  */
/*                                                                                    */
/**************************************************************************************/
void timer()

{
  //SetTimer (targetTime*60);
  if (sw_timer == HIGH)
  {
    apertouBotao = true;
    StartTimer();
    SetTimer (targetTime*60); // Define o tempo do timer
  }
  if (apertouBotao)
  {
      delay(1000); // isso faz com que os micros() não comecem a contar logo de cara (esperam x tempo com o delay antes de imprimir na tela)
      Serial.print(apertouBotao);
      CountUpDownTimer(DOWN); // run the timer
      lcd.setCursor(0,1);
      lcd.print("T:");
      lcd.print(ShowHours());
      lcd.print(":");
      (ShowMinutes() < 10) ? lcd.print("0") : NULL;
      lcd.print(ShowMinutes());
      lcd.print(":");
      (ShowSeconds() < 10) ? lcd.print("0") : NULL;
      lcd.print(ShowSeconds());
  }
}

boolean CountUpDownTimer(boolean Type)
{
  _type = Type;
  static unsigned long duration = 1000000; // 1 second
  timeFlag = false;

  if (!Stop && !Paused) // if not Stopped or Paused, run timer
  {
    // check the time difference and see if 1 second has elapsed
    if ((_micro = micros()) - time > duration ) 
    {
      _type == UP? Clock++ : Clock--;
      timeFlag = true;

      if (_type == DOWN && Clock == 0) // check to see if the clock is 0
        Stop = true; // If so, stop the timer

     // check to see if micros() has rolled over, if not,
     // then increment "time" by duration
      _micro < time ? time = _micro : time += duration; 
    }
  }
  return !Stop; // return the state of the timer
}

void ResetTimer()
{
  if(_type) 
    Clock = 0;
  else
    SetTimer(R_clock);

  Stop = false;
}

void StartTimer()
{
  Watch = micros(); // get the initial microseconds at the start of the timer
  Stop = false;
  Paused = false;
  if(_type == UP) 
    Clock = 0;
}

void PauseTimer()
{
  Paused = true;
}

void ResumeTimer() // You can resume the timer if you ever stop it.
{
  Paused = false;
}

void StopTimer()
{
  Stop = true;
}

void SetTimer(unsigned int seconds)
{
 // StartTimer(seconds / 3600, (seconds / 3600) / 60, seconds % 60);
 Clock = seconds;
 R_clock = Clock;
 Stop = false;
}

int ShowHours()
{
  return Clock / 3600;
}

int ShowMinutes()
{
  return (Clock / 60) % 60;
}

int ShowSeconds()
{
  return Clock % 60;
}


boolean TimeHasChanged()
{
  return timeFlag;
}

// output true if timer equals requested time
boolean TimeCheck(unsigned int hours, unsigned int minutes, unsigned int seconds) 
{
  return (hours == ShowHours() && minutes == ShowMinutes() && seconds == ShowSeconds());
  
}

void ResetVariablesForRegulationCalculation()
{
  maxRegTEmp = 0;
  minRegTEmp = 1000;  
}

void EnterRegulateStateOrWaitSmoothLowering()
{

  if (actualTemp < targetTemp + 0.3)
  {
       
    Serial.println("Entrou no estado de regulação !");
    ResetVariablesForRegulationCalculation();
    
    tBackToHigh = 0;  
    // garante que não começa aquecer imediatamente ao entrar em regulação sobre o valor desejado
    if (parametersRegulationSetForTemp == targetTemp && actualTemp > targetTemp )
    { 
      tBackToHigh =   millis() + durationOffPulse;
    } 
    tBackToLow = 0;
    tMinReg = 0;
    tMaxReg = 0;
    tStartRealRegulation = 0;

    opState = REGULATE;
  } 
  else 
  {
    WaitForNaturalDrop();
  }
}

void WaitForNaturalDrop()
{
  opState = WAIT_NATURAL_DROP;
  isCounteracting = false;
  Serial.println("Esperando queda natural!"); 
  ResetVariablesForRegulationCalculation(); 
}

void Regulate()
{ 
   
  if (actualTemp > ( targetTemp + 0.2 ))
  {
    // adapta valores de regulação : eles estaõ muito altos
    if ( IsStabilizing() && parametersRegulationSetForTemp == targetTemp && tStartRealRegulation > 0 && (millis() - tStartRealRegulation) > tOperationalDelay )
    {
      durationOnPulse = durationOnPulse / 1.3;
      while ( durationOnPulse < MIN_SWITCHING_TIME )
      {
        durationOffPulse = durationOffPulse * 1.2;
        durationOnPulse = durationOnPulse * 1.2 ;
      }
      tStartRealRegulation = millis();
      tBackToHigh = millis() + durationOffPulse;
      Serial.print("durationOffPulse = ");
      Serial.print(durationOffPulse);
      Serial.print("durationOnPulse = ");
      Serial.println(durationOnPulse);

      
      WaitForNaturalDrop();
    }
  }
  
  // tenta regular a temperatura quando estamos na temperatura desejada estabilizada

  //Talvez estejamos muito abaixo da meta; hora para um impulso?
  if((targetTemp - actualTemp) >= 0.25)
  {
    //  adapta valores de regulação : eles estaõ muito baixos
    if ( IsStabilizing() && parametersRegulationSetForTemp == targetTemp && (millis() - tStartRealRegulation) > tOperationalDelay )
    {
      durationOffPulse = durationOffPulse / 1.3;
      while ( durationOffPulse < MIN_SWITCHING_TIME )
      {
        durationOffPulse = durationOffPulse * 1.2;
        durationOnPulse = durationOnPulse * 1.2 ;
      }
      Serial.print("Duracao do pulso desligado = ");
      Serial.print(durationOffPulse);
      Serial.print("   duracao do pulso ligado = ");
      Serial.println(durationOnPulse);      
    }   
    StartBoostToTarget();   
  } 
  else 
  {     
    if (parametersRegulationSetForTemp == targetTemp )
    {     
      if (tStartRealRegulation == 0)
      {
        tStartRealRegulation = millis();
        tBackToHigh = 0;
      }
      // Nós ja temos os calculos de duração de liga e desliga
      // performar a regulação
      if (digitalRead(RELAY_OUT_PIN) == LOW) {
        // check if downtime over
        if ( (long) (millis() - tBackToHigh) >= 0)
        {
          turnOnRelay();
          tBackToLow = millis() + durationOnPulse + burnupTime;
          tBackToHigh = millis() + durationOnPulse + burnupTime + durationOffPulse;
        }
      }     
    } 
    else
    {
      if ((targetTemp - actualTemp) >= 0.1)
      {
        //executar um impulso com uma ligeira superação primeiro
        StartBoostToTarget(0.1);  
      } 
      else
      {
        // encontra valores adequados de duração ligado e desligado            
        PerformRegulationCalculations();
      }     
    }
  }  
}

void PerformRegulationCalculations()
{
  if (isNewSample && IsFallingNaturally() && tempPreviousArray[0] != 0 && tempPreviousArray[1] != 0 && tempPreviousArray[2] != 0)
  {
    // calcula a média das 3 últimas amostras
    
    double averageTemp3 = (tempPreviousArray[0] + tempPreviousArray[1] +tempPreviousArray[2]) / 3;
   
    // encontra temperaturas maximas e minimas
    if (averageTemp3 > maxRegTEmp)
    {
      maxRegTEmp = averageTemp3;
      tMaxReg = millis();
    }         
    
    if (averageTemp3 < minRegTEmp)
    {
      minRegTEmp = averageTemp3;
      tMinReg = millis();
    }
    
    Serial.print(" --- TempMédia 3 = ");
    Serial.print(averageTemp3, DEC);
    Serial.print(" --- TempRegMax = ");
    Serial.print(maxRegTEmp, DEC);
    Serial.print(" --- TempRegMin = ");
    Serial.print(minRegTEmp, DEC);
    Serial.print(" --- tMaxReg = ");
    Serial.print(tMaxReg);
    Serial.print(" --- tMinReg = ");
    Serial.println(tMinReg);
    
    
    // espera até perder DROP_DEGREES_FOR_CALC_REGULATION graus
    if (maxRegTEmp > 0 && minRegTEmp > 0 && (((long)(tMinReg - tMaxReg)) > 0) && ((maxRegTEmp - minRegTEmp) > DROP_DEGREES_FOR_CALC_REGULATION))
    {                     
      // Tente subir com durações de pulso (ON e OFF) para neutralizar a perda de temperatura
      SetApproximatePulseDurationsForREgulation(maxRegTEmp - minRegTEmp, tMinReg - tMaxReg);    

      // de volta a temperatura desejada
      StartBoostToTarget();             
    }
  } 
}

bool checkDerivativeReliable()
{
  for(int i = 0; i < 6 ; i++)
  {
    if(tempPreviousArray[i]==0)
    {
      return false;
    }
  }
  return true;
}


void SetActualDerivative()
{
  if (isNewSample)
  {
    isDerivativeReliable = checkDerivativeReliable();   
    Serial.print("d = ");
    Serial.println (MAX_UPTIME_MINS);   
    if (isDerivativeReliable)
    {
      //remover maiores e menores valores (se livrar de irregularidades)
      
      // identificar maiores e menores
      double lowest =  1000;
      double highest =  0;
      int i=0;
      for(i=0;i<6;i++) {
        if(tempPreviousArray[i] > highest)
        highest = tempPreviousArray[i];
        
        if(tempPreviousArray[i] < lowest)
        lowest = tempPreviousArray[i];
      }
      
      double tempTemp[6];
      double filteredValues[4];
      bool isHighestRemoved = false;
      bool isLowestRemoved = false;
      //
      if (currentTempDerivative > 0)
      {
        //tendencia de subida : remove menor valor do final
        for(i=5;i>=0;i--) {
          if(tempPreviousArray[i] == lowest && !isLowestRemoved)
          {
            tempTemp[i] = 0;
            isLowestRemoved = true;
          } else {
            tempTemp[i] = tempPreviousArray[i];
          }         
        }
        // remover valor mais alto para o inicio da matriz
        for(i=0;i<6;i++) {
          if(tempTemp[i] == highest && !isHighestRemoved)
          {
            tempTemp[i] = 0;
            isHighestRemoved = true;
          }         
        }       
      } 
      else      
      {
        //tendencia de descida: remove menor valor do inicio da matriz
        for(i=0;i<6;i++) {
          if(tempPreviousArray[i] == lowest && !isLowestRemoved)
          {
            tempTemp[i] = 0;
            isLowestRemoved = true;
          } else {
            tempTemp[i] = tempPreviousArray[i];
          }         
        }
        // remove o maior valor do final
        for(i=5;i>=0;i--) {
          if(tempTemp[i] == highest && !isHighestRemoved)
          {
            tempTemp[i] = 0;
            isHighestRemoved = true;
          }         
        }
      }
      int j = 0;
      for(i=0;i<6;i++) {
        if(tempTemp[i] != 0)
        {
          filteredValues[j] = tempTemp[i];
          j++;
        }         
      }
      
      double pastValues[2];
      pastValues[0] = ( filteredValues[0] + filteredValues[1] ) / 2;
      pastValues[1] = ( filteredValues[2] + filteredValues[3] ) / 2;
      // calcula a ultima derivada
      previousDerivative = currentTempDerivative;
      currentTempDerivative = ((pastValues[0] - pastValues[1]) / (3* SAMPLE_DELAY/1000));
      Serial.println(currentTempDerivative, DEC);
    } else
    {
      Serial.println("NC!");
      
    }
  } 
}

void GetTemperatureAndEnforceSecurity()
{
  if ( (long) (tcurrent - tGetTemperatureSample) >= 0)
  {
    actualTemp = getTemperature();    
    
    if (opState != TEMP_DROP && (tempPreviousArray[0] - actualTemp > 2))
    {
      //queda repentina na temperatura -> sensor de temperatura fora d'água     
      if(opState == COUNTER_FALL || opState == FIRST_RAMP)
      {
        tBackToLow = 0;
        if (opState == FIRST_RAMP)
        {
          firstRampCutOffTemp = tempPreviousArray[0];
          doBackToFirstRampWhenStabilizing = true;
        }
      }
      
      opState = TEMP_DROP;
      tempBeforeDrop = tempPreviousArray[0];
      waitForSuddenRise = false;  
      Serial.println("Sensor removido!"); 
      
      if (tStartBoostTemp - millis() <= 3 * SAMPLE_DELAY)
      {
        // nós provavelmente impulsionamos a temperatura erradamente já que o sensor de temperatura está fora d'água
        // cancela impulso
        tBackToLow = 0;
      }
      
      
    } 
    if (opState == TEMP_DROP && (actualTemp - tempPreviousArray[0] > 2))
    {
      //aumento repentino na temperatura  -> sensor de temperatura de volata n'água
      opState = TEMP_RISE;
      // apaga valores prévios na historia da temperratura -> previne cáculo de derivada negativa mesmo que estejamos subindo
      tempPreviousArray[1]=0;
      tempPreviousArray[2]=0;
      tempPreviousArray[3]=0;
      tempPreviousArray[4]=0;
      tempPreviousArray[5]=0;
        
      Serial.println("sensor de volta");
    } 
    if (opState == BOOST_TEMP && (actualTemp - tempPreviousArray[0] > 1))
    {
      //subida de temperatura repentina durante BOOST_TEMP -> talvez o sensor tenha sido colocado na água agora      
      if (tStartBoostTemp - millis() <= 3 * SAMPLE_DELAY)
      {
        // nós provavelmente impulsionamos a temperatura erradamente já que o sensor de temperatura está fora d'água
        // cancela impulso
        tBackToLow = 0;
      }
      // apaga valores prévios na historia da temperatura -> previne cáculo de derivada negativa mesmo que estejamos subindo
      tempPreviousArray[1]=0;
      tempPreviousArray[2]=0;
      tempPreviousArray[3]=0;
      tempPreviousArray[4]=0;
      tempPreviousArray[5]=0;
    }
    
    tempPreviousArrayPushValue(actualTemp); 
    isNewSample = true;
    if (OUTPUT_TO_SERIAL) {
      Serial.print(tcurrent/1000, DEC);
      Serial.print(";  ");
      Serial.println(actualTemp, 3);
    }    
    if (actualTemp > targetTemp + 0.15)
    {
      //  força desligar quando não precisa estar ligado(0.15 offset conta para condição de regulação)
      tBackToLow = 0;
    }
    
    alertTemperatureNearlySet();
    checkShutdownConditions();    
  } else {
    isNewSample = false;
  }
}

void WatchForTempFalling()
{
  if (isNewSample)
  {
    // CONDIÇÃO INICIAL: Temeperatura bem abaixo da desejada  && importante derivada negativa , mas não em queda livre : -0.1 < d < -0.007,  3 média de 3 vezes
    if ( (targetTemp - actualTemp) > 1 && IsFalling() )
    {
      // deve acontecer em média 3 vezes
      warningsBeforeCounterFall--;
      if (warningsBeforeCounterFall == 0)
      {
        turnOnRelay();
        waitingForStabilization = false;
        opState = COUNTER_FALL;
      }
    } 
    else 
    {
      warningsBeforeCounterFall = 3;
    } 
  }
}


void StartBoostToTarget()
{
  StartBoostToTarget(0);
}

void StartBoostToTarget(double offset)
{
  // valor previsto em t + tOperationalDelay
  actualTempAtBoostStart = actualTemp;  
  double realTargetTemp = targetTemp + offset;
  if (realTargetTemp > actualTempAtBoostStart)
  { 
    expectedTempChange = realTargetTemp - actualTempAtBoostStart;
    Serial.print("Impulsionar! eMudança de temperatura esperada = ");
    Serial.println(expectedTempChange);
    HeatForDegrees(expectedTempChange); 
    // change state
    opState = BOOST_TEMP;
    storedTargetTemp = targetTemp;
    tStartRealRegulation = 0;
  }
}


double HeatingTimeNeeded(double degreeOffset)
{
  double secondPerDegreeGain;
  if (degreeOffset > LARGE_TEMP_DIFFERENCE)
  {
    secondPerDegreeGain = secondPerDegreeGainLarge;
    boostType = HIGHBOOST;
  } else {
    secondPerDegreeGain = secondPerDegreeGainSmall;
    boostType = LOWBOOST;
  }
  return max(degreeOffset * secondPerDegreeGain * 1000, MIN_SWITCHING_TIME) + burnupTime;
}

void HeatForDegrees(double degrees)
{
  if (degrees > 0)
  {
    
    tBackToLow = 0;
    tCheckStabilize = 0;
    tStartBoostTemp = millis();
    tBackToLow = millis() +  HeatingTimeNeeded(degrees);
    tCheckStabilize = tBackToLow + tOperationalDelay;   

    if ( (long) (millis() - tBackToLow) < 0)
    {  
      turnOnRelay();
      Serial.print("Aquecendo !  = ");
      Serial.println(tBackToLow, DEC);
      Serial.print("estabilizando = ");
      Serial.println(tCheckStabilize);
    }   
  }
}

void PerformBoostTemp()
{   
   if ( (long) (millis() - tBackToLow) >= 0)
   {        
    //verifica se a temperaura desejada muda e apapta temporizações
    if (targetTemp > storedTargetTemp)
    {
      StartBoostToTarget();
    }
     // espera pela estabilização      
         
    // performa calculos seguintes a cada SAMPLE_DELAY quando alcançamos tOperationalDelay desde que o impulso de temperatura iniciou
    if ( ((long) (millis() - tCheckStabilize) >= 0)  && isNewSample && isDerivativeReliable)
    {   
      // verifica se estabilizou
      if  (IsStabilizingOrDropping())
      {     
        Serial.println("Estabilizado !");
        FinishBoostTemp();
      }   
    }    
   } else {  
    // liga o relé e espera por tBackToLow
     if (digitalRead(RELAY_OUT_PIN) == LOW) {
       turnOnRelay();
     }       
     
    //verifica se a temperatura desejada mudou e adapta temporizações
    if (targetTemp != storedTargetTemp)
    {   
      double changeOffset =  targetTemp - storedTargetTemp;
      double newExpectedTempChange = expectedTempChange + changeOffset;
      
      tBackToLow = tStartBoostTemp + HeatingTimeNeeded(newExpectedTempChange);
      tCheckStabilize = tBackToLow + tOperationalDelay;
      storedTargetTemp = targetTemp;
      expectedTempChange = expectedTempChange + changeOffset;
      
      Serial.print("Temperatura desejada mudou, novo tBackToLow = ");
      Serial.println(tBackToLow);
      Serial.print("Temperatura desejada mudou, novo expectedTempChange = ");
      Serial.println(expectedTempChange);
    }
   }
}


void FinishBoostTemp()
{      
  AdaptGain(actualTemp);
   
   Serial.println("Fim de impulso na temperatura !");
   
   // entra no estado regulado
   EnterRegulateStateOrWaitSmoothLowering();
}


double predictTemp(unsigned long horizon)
{
  double horizonSeconds = horizon/1000; 
    
  // computa valor previsto 
  return ((( tempPreviousArray[0] + tempPreviousArray[1] + tempPreviousArray[2] ) / 3 ) + (currentTempDerivative * horizonSeconds));
}

void AdaptGain(double resultingTemp)
{
  // só leva em conta ON_Durations > burnupTime e garante que esperamos tOperationalDelay
  unsigned long boostTempDuration = millis() - tStartBoostTemp;   
  unsigned long boostOnTempDuration = tLastTurnOffRelay - tStartBoostTemp;   
    if ( boostTempDuration > tOperationalDelay && boostOnTempDuration > burnupTime )
  { 
        double gain;
    if (boostType == LOWBOOST)
    {
      gain = secondPerDegreeGainSmall;
    }
    else
    {
      gain = secondPerDegreeGainLarge;
    }
      
      
    double actualTempChange = resultingTemp - actualTempAtBoostStart;
      
    if (actualTempChange < (expectedTempChange / 5) )
    {
      gain = gain * 1.8;
    } 
    else
    {
       if (actualTempChange < (expectedTempChange / 2) )
       {
        gain = gain * 1.4;
       } 
       else
       {
         if (expectedTempChange > 0.2 && actualTempChange > 0.1)
         {
          // expectedTempChange > 0.2 serve para evitar erros grandes devido a mudanças pequenas
          gain = gain * expectedTempChange / actualTempChange;
         }    
       }
    }
    
    // garante que ganho adaptado fique dentro de limites aceitáveis (de secondPerDegreeGainRef/3 a secondPerDegreeGainRef*3)
      
    if (gain > secondPerDegreeGainRef*3)
      gain = secondPerDegreeGainRef*3;      
      
    if (gain < secondPerDegreeGainRef/3)
      gain = secondPerDegreeGainRef/3;
          
    switch (boostType)
    {
    case LOWBOOST:        
      secondPerDegreeGainSmall = gain;
      Serial.print("Ganho pequeno segundo/grau =");
      Serial.println(secondPerDegreeGainSmall);
      break;
    case HIGHBOOST: 
      secondPerDegreeGainLarge = gain;
      Serial.print("Ganho grande segundo/grau =");
      Serial.println(secondPerDegreeGainLarge);
      break;
    }
   }
}


void StartInitialRamping()
{
   // entra no estado primeira rampa
   opState = FIRST_RAMP;

   // armazena  teperatura inicial
   initialTemp = actualTemp;
   tStartFirstRamp = millis();

   setupCutOffTempForInitialRamping();
}


void setupCutOffTempForInitialRamping()
{
  // calcula a temperatura de desligamento
   firstRampCutOffTemp = initialTemp +  (targetTemp - initialTemp) * FIRST_RAMP_CUTOFF_RATIO;
   storedTargetTemp = targetTemp;
   
   Serial.print("Temp de desligamento primeira rampa = ");
   Serial.println(firstRampCutOffTemp, DEC);
}

void PerformFirstRamp()
{
    if (targetTemp != storedTargetTemp)
    {
    // Temeperatura desejada mudou ! Atualiza firstRampCutOffTemp
    setupCutOffTempForInitialRamping();
    }
  
    if (actualTemp > firstRampCutOffTemp) 
    {
    // desliga aquecimento e espera estabilização
       if (digitalRead(RELAY_OUT_PIN) == HIGH) {
         Serial.print("STOP at actualTemp = ");
         Serial.println(actualTemp, DEC);
         turnOffRelay();  
         tFirstRampCutOff = millis();
       }
       
       if ( isNewSample )
       {            
          // vrifica se esta perto do ponto de estabilização
          if  ((abs(actualTemp - initialTemp) > abs(targetTemp - actualTemp)) && IsStabilizingOrDropping())
          {
            FinishInitialRamping();
          }               
       }        
    } else {
      // aquec todo vapor
       if (digitalRead(RELAY_OUT_PIN) == LOW)     turnOnRelay();
       
       // tenta encontrar quantotempo é necessário ao sistema para reagir e aquecer
       if (((long) (millis() - tCheckTakeOff) >= 0) && (tOperationalDelay == 0))
       {         
         tCheckTakeOff = millis() + SAMPLE_DELAY;
         
         // tenta encontrar quantotempo é necessário ao sistema para reagir e aquecer
         if(tempPreviousArray[0] > tempPreviousArray[1] && tempPreviousArray[1] > tempPreviousArray[2] && tempPreviousArray[2] > tempPreviousArray[3] && tempPreviousArray[3] > tempPreviousArray[4])
         {
           tOperationalDelay = (millis() - tStartFirstRamp - 3*SAMPLE_DELAY);
       burnupTime = tOperationalDelay / 20; // arbitrary... to be perfected
           Serial.print("tOperationalDelay = ");
           Serial.println(tOperationalDelay, DEC);
         }          
       }
    }
}


void FinishInitialRamping()
{
  // Retorna ao controle normal quando detectamos a eestabilidade
  tEndFirstRamp = millis();
  
  // encontra temperatura final antes de estabilizar ou cair
  double finalTemp = 0;
  for(int i=0;i<6;i++)
  {
  if (tempPreviousArray[i] > finalTemp)
  {
    finalTemp = tempPreviousArray[i];
  }
  }
  
  secondPerDegreeGainRef = (tFirstRampCutOff - tStartFirstRamp) / (1000*(finalTemp - initialTemp));
  secondPerDegreeGainLarge = secondPerDegreeGainRef;  
  secondPerDegreeGainSmall = secondPerDegreeGainLarge;

   Serial.print("FinishInitialRamping !   tEndFirstRamp = ");
   Serial.println(tEndFirstRamp, DEC);
   Serial.print("secondPerDegreeGainLarge = ");
   Serial.println(secondPerDegreeGainLarge);

   
  // entra no estado regulado
   EnterRegulateStateOrWaitSmoothLowering();
}


void turnOnRelay()
{
  Serial.println("Aquecendo !");
  digitalWrite(RELAY_OUT_PIN,HIGH);
  tCheckNotHeatingWildly = millis() + ((unsigned long)60000 * MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES);
  Serial.println("tCheckNotHeatingWildly =");
  Serial.println(tCheckNotHeatingWildly, DEC);
  tempBeforeHeating = actualTemp;
  isHeatOn = true;
}
    
void turnOffRelay()
{
  digitalWrite(RELAY_OUT_PIN,LOW);
  tLastTurnOffRelay = millis();
  tCheckNotHeatingWildly = 0;
  isHeatOn = false;
}    
    
// verificação de segurança   
void checkShutdownConditions(){
  boolean doShutdown = false;
  
  // verifica longo tempo ligado
  if ( (long) (millis() - maxUptimeMillis) >= 0)
  {
    Serial.println(maxUptimeMillis);
    doShutdown = true;
  }
  
  // verifica temperatura muito alta
  if (actualTemp > SHUTDOWN_TEMP)
  {
    Serial.println(actualTemp);
    doShutdown = true;
  }
  
  // verifica longo tempo aquecendo sem aumento de temperatura(sensor de temperatura não mais confiável, desliga sistema)
  if (tCheckNotHeatingWildly > 0 && isHeatOn && ( (long) (millis() - tCheckNotHeatingWildly) >= 0))
  {
    if (actualTemp <= tempBeforeHeating)
    {
      // temperatura não aumenta mesmo se mantiver aquecedor ligado durante MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES
      Serial.println("Tempo máximo aquecendo sem alterar temperatura !");
      doShutdown = true;
    }
    // plan next check
    tempBeforeHeating = actualTemp;
  tCheckNotHeatingWildly = millis() + ((unsigned long)60000 * MAX_HEATINGTIME_NO_TEMP_CHANGE_MINUTES);
  }
  
  if (doShutdown == true)
  {
    shutdownDevice();
  }
}


void shutdownDevice() 
{
  eraseDisplay();
  displayActualTemp(0);
  displayTargetTemp(0);

    if (OUTPUT_TO_SERIAL) {      
        Serial.println("DESLIGADO");
    }
    // turn off relay !
    digitalWrite(RELAY_OUT_PIN,LOW);
  isHeatOn = false;
    while(1)
    {
      delay(30000);
    }
}

void readButtonInputs()
{ 
  // read buttons
  sw_tempMore = digitalRead(BT_TEMP_MORE_PIN);
  sw_tempLess = digitalRead(BT_TEMP_LESS_PIN);
  sw_timeMore = digitalRead(BT_TIME_MORE_PIN);
  sw_timeLess = digitalRead(BT_TIME_LESS_PIN);
  sw_timer = digitalRead(BT_TIMER_PIN);
 
  

  
  // process inputs
  if (sw_tempMore == HIGH) 
  { 
    targetTemp= min(targetTemp + 0.5, MAX_TARGET_TEMP);    
    if (targetTemp > actualTemp)    isWaitingForTempAlert = true;
  }
  
    if (sw_tempLess == HIGH) 
    {
    targetTemp-=0.5;
    }
    if (sw_timeMore == HIGH)
     {
     targetTime=targetTime+1;
    Serial.println(targetTime);
     }
     if (sw_timeLess == HIGH)
     {
     targetTime = targetTime -1;
     Serial.println(targetTime);
     }
}

  
void SetApproximatePulseDurationsForREgulation(double tempLost, unsigned long regDelay )
{
  // calcula tempode atividade para compensar 
  unsigned long neededUptimeForCompensate = tempLost * secondPerDegreeGainRef * 1000;
  SetPulseDurationsForREgulation(neededUptimeForCompensate, regDelay );
}

void SetPulseDurationsForREgulation(unsigned long neededUptimeForCompensate, unsigned long regDelay )
{ 
  Serial.print(" --- neededUptimeForCompensate = ");
  Serial.println(neededUptimeForCompensate);
  
  // distribuir uniformemente o tempo de atividade necessária           
  if (neededUptimeForCompensate >= regDelay) 
  {
    // precisaríamos tempo ligado total! Chamada para um impulso temporário ao invés de uma ligeira superação
    StartBoostToTarget(0.2);          
  } 
  else 
  {
    // garante que pulsos (períodos ligados e desligados)não violarão MIN_SWITCHING_TIME
    while ( (regDelay / 2) < MIN_SWITCHING_TIME )
    {
      neededUptimeForCompensate = neededUptimeForCompensate * 2;
      regDelay = regDelay *2 ;
    }
    while ( (neededUptimeForCompensate / 2) < MIN_SWITCHING_TIME )
    {
      neededUptimeForCompensate = neededUptimeForCompensate * 2;
      regDelay = regDelay *2 ;
    }
    while ( (regDelay - neededUptimeForCompensate ) < MIN_SWITCHING_TIME )
    {
      neededUptimeForCompensate = neededUptimeForCompensate * 2;
      regDelay = regDelay *2 ;
    }
    
    // 
    int nbOnPulsePerRegPeriod = (int) neededUptimeForCompensate / MIN_SWITCHING_TIME;
    int remainder = (int) neededUptimeForCompensate % MIN_SWITCHING_TIME;
    durationOnPulse = MIN_SWITCHING_TIME + ((unsigned long)(remainder / nbOnPulsePerRegPeriod));
    durationOffPulse = (regDelay - neededUptimeForCompensate) / nbOnPulsePerRegPeriod;
    
    // garantir que o tempo desligado é maior que o minimo tempo de chaveamento
    while ( durationOffPulse < MIN_SWITCHING_TIME )
    {
      durationOffPulse = durationOffPulse * 2;
      durationOnPulse = durationOnPulse *2 ;
    }
    
    // armazena que temos bons parâmetros para esta temperatura
    parametersRegulationSetForTemp = targetTemp;
    
    Serial.print("durationOffPulse = ");
    Serial.print(durationOffPulse);
    Serial.print("durationOnPulse = ");
    Serial.println(durationOnPulse);  
  }
}
// 

/**************************************************************************************/
/*                                                                                    */
/*                                    UTILIDADES                                       */
/*                                                                                    */
/**************************************************************************************/


// ------------------------- UTILIDADES de matriz de temperatura

void tempPreviousArrayPushValue(double val)
{
    tempPreviousArray[5] = tempPreviousArray[4];
    tempPreviousArray[4] = tempPreviousArray[3];
      tempPreviousArray[3] = tempPreviousArray[2];
      tempPreviousArray[2] = tempPreviousArray[1];
      tempPreviousArray[1] = tempPreviousArray[0];
      tempPreviousArray[0] = val;
}

// ------------------------- UTILIDADES de derivadas e tendencias de temperatura

bool IsStabilizingOrDropping()
{
  bool toReturn = false;
  if (isDerivativeReliable && (tempPreviousArray[0] <= tempPreviousArray[1] && tempPreviousArray[1] <= tempPreviousArray[2] && tempPreviousArray[2] <= tempPreviousArray[3] && tempPreviousArray[3] <= tempPreviousArray[4]  && tempPreviousArray[4] <= tempPreviousArray[5])) toReturn = true; 
  //(currentTempDerivative < 0.001)
  return toReturn;
}


bool IsStabilizingOrGrowing()
{
  bool toReturn = false;
  if (isDerivativeReliable && (currentTempDerivative >= 0)) toReturn = true;
  return toReturn;
}

bool IsStabilizing()
{
  bool toReturn = false;
  if (isDerivativeReliable && (abs(currentTempDerivative) <= 0.001)) toReturn = true;
  return toReturn;
}

bool IsFallingNaturally()
{
  bool toReturn = false;
  if (isDerivativeReliable && currentTempDerivative > -0.006 && currentTempDerivative <= 0 ) toReturn = true;
  return toReturn;
}

bool IsFalling()
{
  bool toReturn = false;
  if (isDerivativeReliable && currentTempDerivative > -0.1 && currentTempDerivative < -0.007 ) toReturn = true;
  return toReturn;
}

bool IsAcceleratingFall()
{
  bool toReturn = false;
  if (isDerivativeReliable && currentTempDerivative < previousDerivative &&  previousDerivative < 0 ) toReturn = true;
  return toReturn;
}

void eraseDisplay() 
{       
    // apaga o display
    lcd.clear();
}

void displayTargetTime(int)
{ 
  lcd.setCursor(10,1);
  lcd.print("TP:");
  lcd.print(targetTime);
  lcd.home();
}


void displayActualTemp(float temp)
{ 
  lcd.setCursor(0,0);
  lcd.print("TA:");
  lcd.print(actualTemp);
  
   }
void displayTargetTemp(float temp)
{
  lcd.setCursor(9,0);
  lcd.print("TD:");
  lcd.print(targetTemp);
  lcd.home();
}


// ------------------------- outras UTILIDADES

void soundAlarm()
{
  Serial.println("ALERTA");
  for(int index=0;index<3;index++) {
    tone(PIEZO_PIN, 650, 1000);
    //Serial.println("BIIP");
    delay(2000);
  }  
}

void alertTemperatureNearlySet()
{
  if (isWaitingForTempAlert == true && abs(targetTemp - actualTemp) < 0.3)
  {
    soundAlarm();
    isWaitingForTempAlert = false;
  }  
}

float getTemperature()
{
  // planeja próxima medição
  tGetTemperatureSample = millis() + SAMPLE_DELAY;
  sensors.requestTemperaturesByIndex(0); // Send the command to get temperatures
  return sensors.getTempC(tempProbeAddress);
}
