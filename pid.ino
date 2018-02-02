

#include <LiquidCrystal_I2C.h>  //Biblioteca necessaria para o conversor I2C do display LCD
#include <Wire.h> //Biblioteca para comunicação I2C





#define KpL 43.73 //CONSTANTE PROPORCIOANL DA LAMPADA
#define KiL 0.01 //CONSTANTE INTEGRAL DA LAMPADA
#define KdL 0.01 //CONSTANTE DERIVADA DA LAMPADA
#define KpC 20.78 //CONSTANTE PROPORCIOANL DO COOLER
#define KiC 0.01 //CONSTANTE INTEGRAL DO COOLER
#define KdC 0.01 //CONSTANTE DERIVADA DO COOLER
#define S_Umid A3 //Definição da porta analogica do sensor de umidade do solo
#define S_Temp A2 //Definição da porta analogica do sensor de umidade do solo
#define S_Gas 2 //Definição da porta digital de interrupção para o sensor de gas 
#define S_Lumi 47 //Definição da porta analogica para o sensor de luminosidade
#define S_Presenca 3 //Definição da porta digital de interrupção para o sensor de presença
#define SDA_LCD A4 //Define que será usado o pino 20 para dados da comunicação I2C
#define SCL_LCD A5 //Define que será usado o pino 21 como clock para a comunicação I2C
#define Buzzer 30 //Define o pino que o Buzzer de alarme deve ser ligado
#define Led_Valv 13 //Define o pino que o led que indica se a valvula está acionada
#define Valv 7 //Define o pino que aciona a Valvula
#define Led_Ilumin 33 //Define o pino que o led que indica se a iluminação está acionada
#define Ilumin 29 //Define o pino que aciona a Iluminação
#define Cooler_M 6 //Define o pino que aciona o Cooler_Menor
#define Led_Lamp 43 //Define o pino que o led que indica se a lampada está acionada
#define Lamp 9 //Define o pino que aciona a Lampada
#define Led_Cooler 45 //Define o pino que o led que indica se o Cooler está acionado
#define Cooler 10 //Define o pino que aciona o Cooler
#define Aj_Umid A1 //Define a porta analogica para ajuste da umidade
#define Aj_Temp A0 //Define a porta analogica para ajuste da umidade
#define Aj_Pin 37 //Definição da porta digital de interrupção para ajustar a umidade e temperatura de referencia

//Inicializa o display no endereço 0x20
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
/* lcd.setBacklight(HIGH ou LOW); //Luz de fundo do display
   lcd.setCursor(0,0); //endereço no visor
   lcd.print("Arduino e Cia !!");//imprimir informação */

//Variaveis para a função Sensor_de_Presenca
volatile boolean Status_Ilumin = 0; //ARMAZENA O STATUS DO LED QUE ILUMINA A ESTUFA - ON/OFF
volatile float Intensidade_Lumin = 0;//ARMAZENA O VALOR DO SENSOR DE LUMINOSIADE
//Variaveis para a função Sensor_de_Gas
volatile boolean Status_Gas = 0; //ARMAZENA O STATUS SE HÁ PRESENÇA DE GAS OU NAO NA ESTUFA
volatile boolean Status_Msg = 0; //ARMAZENA SE A MENSAGEM DE ALERTA JA FOI EMITIDA OU NÃO
//Variaveis para função Ajuste_de_Referencia
volatile boolean Status_Aj = 0; //ARMAZENA O MODO DE OPERAÇÃO, SE ESTÁ EM CONFIGURAÇÃO OU RODDANDO O PROGRAMA PRINCIPAL
volatile boolean Reset_Prog = 0;//ARMAZENA STATUS DE DESVIO DE FLUXO DO PROGRAMA PRINCIPAL
float Ref_Temp = 0;//ARMAZENA O VALOR DE TEMPERATURA DESEJADA PELO POTENCIOMETRO
float Ref_Umid = 0;//ARMAZENA O VALOR DE UMIDADE DESEJADA PELO POTENCIOMETRO
int i, Media_Calculada;//VARIAVEL PARA "FOR" E ARMAZENA MEDIA DAS LEITURRAS INSTANTANEA DOS POTENCIOMETROS
//Variaveis para função Sensor_de_Umidade
float Umid_Solo = 0; //ARMAZENA O VALOR LIDO PELO SENSOR DE UMIDADE
int j, Media_Umidade;//VARIAVEL PARA "FOR" E AMARZENA MEDIA DAS LEITURAS INSTANTANEA DO SENSOR DE UMIDADE
//Variaveis para PID
int k, Media_Temp;//VARIAVEL PARA "FOR" E AMARZENA MEDIA DAS LEITURAS INSTANTANEA DO SENSOR DE TEMPERATURA
int Tempo_Anterior = 0;//ARMAZENA TEMPO DE REFERENCIA PASSADO
int Tempo_Atual = 0;//ARMAZENA TEMPO DE REFERENCIA ATUAL
int Tempo_Amostragem = 500;//TEMPO QUE SE DESEJA ATUALIZAR O PID
float Erro_Atual_L = 0; //ARMAZENA O ERRO DO INSTANTE ATUAL DE TEMPERATURA PARA PID DA LAMPADA
float Erro_Atual_C = 0;//ARMAZENA O ERRO DO INSTANTE ATUAL DE TEMPERATURA PARA PID DO COOLER
float Erro_Anterior_L = 0;//ARMAZENA O ERRO DO INSTANTE ANTERIRO DE TEMPERATURA PARA PID DA LAMPADA
float Erro_Anterior_C = 0;//ARMAZENA O ERRO DO INSTANTE ANTERIRO DE TEMPERATURA PARA PID DA COOLER
float I_Anterior_L = 0;//ARMAZENA O CALCULO DA PARTE INTEGRATIVA DO INSTANTE ANTERIOR AO CALCULADO DA LAMPADA
float I_Anterior_C = 0;//ARMAZENA O CALCULO DA PARTE INTEGRATIVA DO INSTANTE ANTERIOR AO CALCULADO DO COOLER
float I_Atual_L = 0;//ARMAZENA O CALCULO DA PARTE INTEGRATIVA DO INSTANTE ATUAL  DA LAMPADA
float I_Atual_C = 0;//ARMAZENA O CALCULO DA PARTE INTEGRATIVA DO INSTANTE ATUAL  DO COOLER
float D_L = 0;//ARMAZENA O CALCULO DA PARTE DERIVATIVA DO INSTANTE ATUAL  DA LAMPADA
float D_C = 0;//ARMAZENA O CALCULO DA PARTE DERIVATIVA DO INSTANTE ATUAL  DO COOLER
float P_L = 0;//ARMAZENA O CALCULO DA PARTE PROPORCIONAL DO INSTANTE ATUAL  DA LAMPADA
float P_C = 0;//ARMAZENA O CALCULO DA PARTE PROPORCIONAL DO INSTANTE ATUAL  DO COOLER
int Lamp_PID = 0;//ARMAZENA VALOR DE TEMPERATURA DE CONTROLE USADO NA LAMPADA
int Cooler_PID = 0;//ARMAZENA VALOR DE TEMPERATURA DE CONTROLE USADO NA LAMPADA
float Temp_Ambiente = 0;//ARMAZENA O VALOR DA TEMPERATURA AMBIENTE
int PWM_Lamp = 0;//ARMAZENA VALOR DE TEMPERATURA DE CONTROLE CONVERTIDO PARA VALOR DE PWM
int PWM_Cooler = 0;//ARMAZENA VALOR DE TEMPERATURA DE CONTROLE CONVERTIDO PARA VALOR DE PWM
void setup() {
  Serial.begin(9600);
  //Inicia o Display LCD
  lcd.begin (16,2); //Define o tamanho do display
  lcd.setBacklight(HIGH);//ATIVA BRILHO DO LCD
  lcd.clear();//LIMPA LCD
  //Configura como saida os Dispositivos
  pinMode(Buzzer, OUTPUT); //Alarme sonoro
  pinMode(Cooler_M, OUTPUT); //Cooler Menor
  pinMode(Led_Lamp, OUTPUT); ////Indicação de funcionamento
  pinMode(Lamp, OUTPUT); //Aquecedor
  pinMode(Led_Cooler, OUTPUT); ////Indicação de funcionamento
  pinMode(Cooler, OUTPUT); //Cooler de refrigereção
  //Configura Interrupção para a função do Ajuste_de_Referencia

  //Configura entrada para ajute de temperatura e umidade
  pinMode(Aj_Umid, INPUT); //Sensor de Luminosidade
  pinMode(Aj_Temp, INPUT); //Sensor de Luminosidade
  //Configura Interrupção para a função do Sensor_de_Presença

  //Configura as Entradas e Saidas para função Sensor_de_Presenca
  pinMode(S_Lumi, INPUT); //Sensor de Luminosidade
  pinMode(Led_Ilumin, OUTPUT); //Indicação de funcionamento
  pinMode(Ilumin, OUTPUT); //Iluminação
  //Configura Interrupção para a função do Sensor_de_Gas

  //Configura as entradas e saidas para controle de umidade
  pinMode(Led_Valv, OUTPUT); //Indicação de funcionamento
  pinMode(Valv, OUTPUT); //Valvula de agua
  pinMode(S_Umid, INPUT); //Sensor de Umidade do solo
  pinMode(S_Temp, INPUT); //Sensor de Umidade do solo
  //PID
  Tempo_Anterior = millis();//ARMAZENA TEMPO INICIAL

  digitalWrite(Cooler_M,LOW);

}

void loop() {
  

  //REFERENCIA PARA TEMPERATURA

  Ref_Temp =  analogRead(Aj_Temp) / 20  ;//ARMAZENA VALOR DE AJUSTE DE TEMPERATURA
  Ref_Umid =  analogRead(Aj_Umid)   ;//ARMAZENA VALOR DE AJUSTE DE UMIDADE
  Umid_Solo = analogRead(S_Umid) ;//LER VALOR DO SENSOR DE UMIDADE
  if(Ref_Umid < 500){
    Ref_Umid = 500; //SENSOR QUANDO IMERSO NA AGUA APRESENTA VALOR DE 450 , LOGO, A REFERENCIA NÃO PODE SER MENOR QUE 450
  }
  if (Umid_Solo > Ref_Umid) { //QUANDO SECO SENSOR APRESENTA 1023, QUANDO MUITO UMIDO , APRESENTA 450
    digitalWrite(Valv, LOW);//LIGA VALVULA , LOGICA INVERSA
    digitalWrite(Led_Valv, HIGH);//LIGA LED DE INDICAÇÃO
  } else {
    digitalWrite(Valv, HIGH);
    digitalWrite(Led_Valv, LOW);
  }

  //Ref_Temp = 30; //SETA PARAMETROS MANUALMENTE PARA TESTE
  //Ref_Umid = 30;
  F_PID();//FUNÇÃO DE CONTROLE
}




void F_PID() {
  Tempo_Atual = millis();//ATUALIZA TEMPO 

  if ((Tempo_Atual - Tempo_Anterior) >= Tempo_Amostragem) {  //CONDIÇÃO PARA COMPUTAR PID DE ACORDO COM O TEMPO DE AMOSTRAGEM

    Media_Temp = 0; //ZERA A VARIAVEL PARA PODER SER USADA
    for (k = 0; k < 100; k++) { //REALIZA UMA MEDIA DAS MEDIÇÕES INSTANTANEAS PARA IGNORAR DISCREPANCIAS 
      Media_Temp += analogRead(S_Temp); //REALIZA MEDIÇÃO
    }
    Temp_Ambiente = (Media_Temp / 100) * 0.00488 / 0.01 ; //CONVERTE PARA TEMPERATURA 
    if (Temp_Ambiente < Ref_Temp ) { //VERIFICA SE IRA REALIZAR O PID DA LAMPADA OU COOLER
      Erro_Atual_L = Ref_Temp - Temp_Ambiente; //CALCULA ERRO ATUAL DO PID
      P_L = KpL * Erro_Atual_L;//CALCULA PARTE PROPORCIONAL 
      I_Atual_L = KiL * ((Erro_Atual_L + Erro_Anterior_L) / 2 ) * Tempo_Amostragem ;//CALCULA PARTE INTEGRATVA ATUAL
      D_L = KdL * (Erro_Atual_L + Erro_Anterior_L) / Tempo_Amostragem ;//CALCULA PARTE DERIVATIVA 
      Lamp_PID = P_L + I_Atual_L + D_L + I_Anterior_L; //CALCULA PID SOMANDO AS PARTES INTEGRATIVAS ANTERIORES

      PWM_Lamp = Lamp_PID  ;//SETA PWM DA LAMPADA 

      if (PWM_Lamp < 0) { 
        PWM_Lamp = 0;
      }
      if (PWM_Lamp > 255) {
        PWM_Lamp = 255;
      }
      analogWrite(Lamp, PWM_Lamp);
      PWM_Cooler = 0;
      analogWrite(Cooler, PWM_Cooler);

      Erro_Anterior_L = Erro_Atual_L; //ATUALIZA O ERRO ANTERIOR
      I_Anterior_L = I_Atual_L; //ATUALIZA A PARTE INTEGRATIVA 
      Tempo_Anterior = Tempo_Atual;//ATUALIZA TEMPO ANTERIRO
    } else if (Temp_Ambiente > Ref_Temp) {

      //PID Cooler

      Erro_Atual_C = Temp_Ambiente - Ref_Temp;
      P_C = KpC * Erro_Atual_C;
      I_Atual_C = KiC * ((Erro_Atual_C + Erro_Anterior_C) / 2 ) * Tempo_Amostragem ;
      D_C = KdC * (Erro_Atual_C + Erro_Anterior_C) / Tempo_Amostragem ;
      Cooler_PID = P_C + I_Atual_C + D_C + I_Anterior_C;

      PWM_Cooler = Cooler_PID / 2;

      if (PWM_Cooler < 0) {
        PWM_Cooler = 0;
      }
      if (PWM_Cooler > 255) {
        PWM_Cooler = 255;
      }

      analogWrite(Cooler, PWM_Cooler);
      PWM_Lamp = 0;
      analogWrite(Lamp, PWM_Lamp);

      Erro_Anterior_C = Erro_Atual_C;
      I_Anterior_C = I_Atual_C;
      Tempo_Anterior = Tempo_Atual;
    }

/*
    Serial.print("temp ajuste="); Serial.println(Ref_Temp);
    Serial.print("Umid ajuste="); Serial.println(Ref_Umid);
    Serial.print("Umidade ="); Serial.println(Umid_Solo);
    Serial.print("temp="); Serial.println(Temp_Ambiente);
    Serial.print("PWM Lamp="); Serial.println(PWM_Lamp);
    Serial.print("PWM Cooler="); Serial.println(PWM_Cooler);   */
    Serial.print(Ref_Temp);
    Serial.print(",");
    Serial.print(Temp_Ambiente);
    Serial.print(",");
    Serial.print(PWM_Lamp);
    Serial.print(",");
    Serial.println(PWM_Cooler);


    

    lcd.clear();//LIMPA LCD
    lcd.setCursor(0,1); //endereço no visor
    lcd.print("T_M = ");lcd.print(Temp_Ambiente);lcd.print("U_M = ");lcd.print(Umid_Solo);//imprimir informação */
    lcd.setCursor(0,0); //endereço no visor
    lcd.print("T_A = ");lcd.print(Ref_Temp);lcd.print("U_A = ");lcd.print(Ref_Umid);//imprimir informação */


  }
}

