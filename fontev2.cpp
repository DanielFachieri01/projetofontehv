//pinos
#define DAC 26
#define ADC 34
#define ledon 18
#define hvon 19
#define ledramp 21

//bibliotecas 
#include "BluetoothSerial.h"
#include <iostream>
#include <vector>
#include <cmath>

//strings de comando
String comando="";
String parametro="";

//variáveis importantes
double pi = 3.14159265358979323846;
float rampdown = 10;
float rampup = 10;
bool hv_on = false;
double v0 = 0;
int tempo = 1000; //em milissegundos

void set_rampup(double valor){
    rampup = valor;
}

void set_rampdown(double valor){
    rampdown = valor;
}

void on(){
    hv_on = true;
    digitalWrite(hvon,HIGH);
}

void setTempo(int valor){
    tempo = valor;
}

std::vector<double> volt_values(double vset, double dv){
    std::vector<double> valores;
    double delta = vset - v0;
    int n_passos = std::max(2, static_cast<int>(std::abs(delta) / dv));
    for (int i=0 ; i < n_passos; i++){
        double t = static_cast<double>(i) / (n_passos - 1);
        double f = std::sin( t * pi / 2);
        valores.push_back(v0 + delta * f);
    }
    v0 = vset;
    return valores;

}

void set_voltage(double vset, double dv = rampup,int tempo = 1000 ){
    if(hv_on){
    //quando ligar, verifica o valor do V_set e faz rampup até chegar nele

    for(auto i : volt_values(vset,dv)){
        dacWrite(DAC,i * 255 / 3000);
        digitalWrite(ledramp,HIGH);
        delay(500);
        digitalWrite(ledramp,LOW);
        delay(500);
    }
}
}

void off(){
    set_voltage(0,rampdown);
    hv_on = false;
    digitalWrite(hvon,LOW);
}


double ADCread(){
    double soma=0;
    double mean=0;
    for(int i=0;i<10;i++){
        soma =+ analogRead(ADC)*1.0;
    }
    mean = soma/10.00;
    delay(50);
    return mean;
}


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


void lerserial() { //funçao que transforma o que e recebido pela serial em uma string
  comando = "";//reset do valor
  parametro="";//reset do valor
  boolean isCommand=true;//determina aonde salvar o caracter
  char carac; //armazena cada caracter
  while(SerialBT.available()>0) { // enquando command receber caracteres
    carac=(char)SerialBT.read();; //grava-lo na variavel carac
    if(carac ==' ') {//se o caracter for espaco
      isCommand=false;//gravar o restante na var parametro
    }
    if((carac !='\n') && (isCommand==true)) {//se o caracter NAO for quebra de linha e for um comando
      comando.concat(carac);//concatena o valor na var comando
    }
    if((carac !='\n') && (isCommand==false)) {//se o caracter NAO for quebra de linha e for um parametro
      parametro.concat(carac);//concatena o valor na var parametro
    }
    delay(50); //delay para nova leitura
  }
}

void interpreter(){
    if(SerialBT.available()) { // se estiver recebendo algo pela serial
    SerialBT.println("Estou recebendo");
    lerserial(); //executar a funçao que transforma os caracteres em string
    SerialBT.print("Comando:"); SerialBT.println(comando);
    // tratar

    if(comando=="dac"){ 
        dacWrite(DAC,parametro.toDouble() * 255.0/ 3.3);
        SerialBT.print("DAC setado para "); SerialBT.print(parametro.toDouble()* 1.0); SerialBT.println(" V");
    }
    if(comando=="adc"){
        SerialBT.print("Tensao medida: "); SerialBT.print(ADCread()*3.3/4095); SerialBT.println(" V");
    }
    if(comando == "vset"){
        SerialBT.print("Vset = "); SerialBT.print(parametro.toDouble()*1.0);SerialBT.println(" V");
        if(hv_on){
        set_voltage(parametro.toDouble()*1.0, rampup);
        }
        else{
        SerialBT.println("HV off! Certifique-se de ligar a HV com 'on' ");
        }
    }
    if(comando == "rampup"){
        SerialBT.print("rampup setada para: ");SerialBT.print(parametro.toDouble()*1.0);SerialBT.println(" V/s");
        rampup = parametro.toDouble();
    }
    if(comando == "rampdown"){
        SerialBT.print("rampdown setada para : "); SerialBT.print(parametro.toDouble()*1.0);SerialBT.println(" V/s");
        rampdown = parametro.toDouble();
    }
    if(comando == "tempo"){
        SerialBT.print("tempo setada para : "); SerialBT.print(parametro.toDouble()*1.0);SerialBT.println(" ms");
        tempo = parametro.toDouble();
    }

    if(comando == "on"){
        on();
        SerialBT.println("High Voltage On!");
    }
    if(comando == "off"){
        off();
        SerialBT.println("High Voltage Off!");
    }

    }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(DAC, OUTPUT);
  pinMode(ADC, INPUT);
  pinMode(ledramp,OUTPUT);
  pinMode(ledon,OUTPUT);
  pinMode(hvon, OUTPUT);
  dacWrite(DAC,0.0);
  //SerialBT.begin(9600);
  digitalWrite(ledon,HIGH);
  SerialBT.begin("remote HVPS"); 

}

void loop() {
  // put your main code here, to run repeatedly:
  interpreter();
}
