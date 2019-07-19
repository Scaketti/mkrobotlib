#include <stdio.h>    
#include <wiringPi.h> 
#include <softPwm.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"

#include <vector>
#include <iostream>

#define nImage 36

//esquerda-dianteiro
#define PINO_E1  12 
#define PINO_E2  16  
#define PWM_ED	 6

//esquerda-traseiro
#define PINO_E3  20  
#define PINO_E4  21  
#define PWM_ET	 5

//direita-dianteiro
#define PINO_D1   4   
#define PINO_D2  17  
#define PWM_DD   19

//direita-traseiro
#define PINO_D3  27
#define PINO_D4  22
#define PWM_DT   13		

//sensores
#define SENSOR_ED 24
#define SENSOR_ET 18
#define SENSOR_DD 26
#define SENSOR_DT 25
#define SENSOR_ED_DIST_POS 0
#define SENSOR_ET_DIST_POS 1
#define SENSOR_DD_DIST_POS 2
#define SENSOR_DT_DIST_POS 3

//num segmentos discos
#define SEG_NUM 20

//pi
#define PI 3.1415

//sentido de rotação dos motores
#define HORARIO         'h'
#define ANTIHORARIO     'a'

#define PWM_VAL 100                                     //velocidade de rotação do motor

int flag[4] = {0, 0, 0, 0};
int seg_cont[4] = {SEG_NUM, SEG_NUM, SEG_NUM, SEG_NUM}; //define o numero de segmentos
float dist_total = 0;                                   //distancia total percorrida pelos motores
float d_roda = 6.5;			                            //diametro da roda
float circuferencia;	                                //calcula circunferencia da roda
float dist_seg;                                         //distancia de cada segmento

VideoCapture rightcap(0);
VideoCapture leftcap(1);

void configuraIOs();

void motorED(char sentido);
void motorET(char sentido);
void motorDD(char sentido);
void motorDT(char sentido);

void frente();
void re();
void esquerda();
void direita();
void para();

void motorControledMove(int sensorID, int sensorID_POS)
int validateMov(int sensorID, int seg_cont);
void moveDistancia(char sentido, int metros);

void motorControledMove(int sensorID, int sensorID_POS)
void viraAngulo(char direcao, int vira);

int getImage(int im);
void calibrarCamera();
void criaDatasetCalib();
void stereo();
std::vector<Point3f> Create3DChessboardCorners(Size boardSize, float squareSize);