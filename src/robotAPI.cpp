#include "robotAPI.hpp"

void configuraIOs() {
    circuferencia = PI * d_roda;
    dist_seg = circuferencia / SEG_NUM;  

    //Esquerda
    pinMode(PINO_E1, OUTPUT);   
    pinMode(PINO_E2, OUTPUT);   
    pinMode(PINO_E3, OUTPUT);   
    pinMode(PINO_E4, OUTPUT); 

    digitalWrite(PINO_E1, 0); 
    digitalWrite(PINO_E2, 0);
    digitalWrite(PINO_E3, 0); 
    digitalWrite(PINO_E4, 0);

    softPwmCreate(PWM_ED, 0, 100);
    softPwmCreate(PWM_ET, 0, 100);

    //Direita
    pinMode(PINO_D1, OUTPUT);   
    pinMode(PINO_D2, OUTPUT);   
    pinMode(PINO_D3, OUTPUT);   
    pinMode(PINO_D4, OUTPUT);   

    digitalWrite(PINO_D1, 0); 
    digitalWrite(PINO_D2, 0);
    digitalWrite(PINO_D3, 0); 
    digitalWrite(PINO_D4, 0); 

    softPwmCreate(PWM_DD, 0, 100);
    softPwmCreate(PWM_DT, 0, 100);

    //seta os valores do PWM dos motores
    softPwmWrite(PWM_DT, PWM_VAL);
    softPwmWrite(PWM_ET, PWM_VAL);
    softPwmWrite(PWM_ED, PWM_VAL);
    softPwmWrite(PWM_DD, PWM_VAL);
}

void motorED(char sentido) {
    switch(sentido){
        //sentido horário
        case 'h':
            digitalWrite(PINO_E1, 1); 
            digitalWrite(PINO_E2, 0);
            break;
        //sentido antihorário
        case 'a':
            digitalWrite(PINO_E1, 0); 
            digitalWrite(PINO_E2, 1);
            break;
        default:
            digitalWrite(PINO_E1, 0); 
            digitalWrite(PINO_E2, 0);
    }
}

void motorET(char sentido) {
    switch(sentido){
        //sentido horário
        case 'h':
            digitalWrite(PINO_E3, 1); 
            digitalWrite(PINO_E4, 0);
            break;
        //sentido antihorário
        case 'a':
            digitalWrite(PINO_E3, 0); 
            digitalWrite(PINO_E4, 1);
            break;
        default:
            digitalWrite(PINO_E3, 0); 
            digitalWrite(PINO_E4, 0);
    }
}

void motorDD(char sentido) {
    switch(sentido){
        //sentido horário
        case 'h':
            digitalWrite(PINO_D3, 1); 
            digitalWrite(PINO_D4, 0);
            break;
       //sentido antihorário
        case 'a':
            digitalWrite(PINO_D3, 0); 
            digitalWrite(PINO_D4, 1);
            break;
        default:
            digitalWrite(PINO_D3, 0); 
            digitalWrite(PINO_D4, 0);
    }
}

void motorDT(char sentido) {
    switch(sentido){
        //sentido horário
        case 'h':
            digitalWrite(PINO_D1, 1); 
            digitalWrite(PINO_D2, 0);
            break;
        //sentido antihorário
        case 'a':
            digitalWrite(PINO_D1, 0); 
            digitalWrite(PINO_D2, 1);
            break;
        default:
            digitalWrite(PINO_D1, 0);
            digitalWrite(PINO_D2, 0);
    }
}

void frente() {
    //Esquerda - sentido horário
    motorED(HORARIO);
    motorET(HORARIO);

    //Direita - sentido horário
    motorDD(HORARIO);
    motorDT(HORARIO);
}

void re() {
    //Esquerda - sentido antihorário
    motorED(ANTIHORARIO);
    motorET(ANTIHORARIO);

    //Direita - sentido antihorário
    motorDD(ANTIHORARIO);
    motorDT(ANTIHORARIO);
}

void esquerda() {
    //Esquerda - sentido antihorário
    motorED(ANTIHORARIO);
    motorET(ANTIHORARIO);

    //Direita - sentido horário
    motorDD(HORARIO);
    motorDT(HORARIO);
}

void direita() {
    //Esquerda - sentido horário
    motorED(HORARIO);
    motorET(HORARIO);

    //Direita - sentido antihorário
    motorDD(ANTIHORARIO);
    motorDT(ANTIHORARIO);

}

void para() {
    //Esquerda - parado
    motorED(' ');
    motorET(' ');

    //Direita - parado
    motorDD(' ');
    motorDT(' ');
}

int validateMov(int sensorID, int seg_cont) {
    int i;
    for(i = 0; i < 4; i++){
        if(seg_cont[sensorID] > seg_cont[i]) return 0;
    }
    return 1;
}

void motorControledMove(int sensorID, int sensorID_POS) {
    usleep(1*1000); //delay para fazer a leitura do sensor
    if(digitalRead(sensorID) == 0 && !flag[sensorID_POS]) flag[sensorID_POS] = 1;

    usleep(1*1000);//delay para fazer a leitura do sensor
    if(digitalRead(sensorID) == 1 && flag[sensorID_POS]){
            //if(!validateMov(sensorID, seg_cont[sensorID_POS])) return;

            seg_cont[sensorID_POS]--;
            dist_total += dist_seg;
            flag[sensorID_POS] = 0;
    }
    
    if(seg_cont[sensorID_POS] == 0) seg_cont[sensorID_POS] = SEG_NUM;
}

void moveDistancia(char sentido, int metros){
    if(sentido == 'f')
        frente();
    else
        re();

    while(dist_total/4 < metros){ //média de distancia percorrida pelos quatro motores
        motorControledMov(SENSOR_ED, SENSOR_ED_DIST_POS);
        motorControledMov(SENSOR_DD, SENSOR_DD_DIST_POS);
        motorControledMov(SENSOR_ET, SENSOR_ET_DIST_POS);
        motorControledMov(SENSOR_DT, SENSOR_DT_DIST_POS);
    }

    dist_total = 0;
    para();
}

void motorControledMove(int sensorID, int sensorID_POS) {
    usleep(1*1000); //delay para fazer a leitura do sensor
    if(digitalRead(sensorID) == 0 && !flag[sensorID_POS]) flag[sensorID_POS] = 1;

    usleep(1*1000);//delay para fazer a leitura do sensor
    if(digitalRead(sensorID) == 1 && flag[sensorID_POS]){
            //if(!validateMov(sensorID, seg_cont[sensorID_POS])) return;

            seg_cont[sensorID_POS]--;
            dist_total += dist_seg;
            flag[sensorID_POS] = 0;
    }
    
    if(seg_cont[sensorID_POS] == 0) seg_cont[sensorID_POS] = SEG_NUM;
}

void vira(char direcao, int vira){
    if(direcao == 'd')
        direita();
    else
        esquerda();
    
    while(dist_total/4 < vira){ //média de distancia percorrida pelos quatro motores
        motorControledRot(SENSOR_ED, SENSOR_ED_DIST_POS);
        motorControledRot(SENSOR_DD, SENSOR_DD_DIST_POS);
        motorControledRot(SENSOR_ET, SENSOR_ET_DIST_POS);
        motorControledRot(SENSOR_DT, SENSOR_DT_DIST_POS);
    }

    dist_total = 0;
    para();
}

int getImage(int im){
    if(!leftcap.isOpened() || !rightcap.isOpened())  // check if we succeeded
        return -1;

    Mat left, right;
	
    if(!(leftcap.grab() && rightcap.grab())) return -1;

    leftcap >> left;
    rightcap >> right;

    if(left.empty() || right.empty())
    {
        std::cerr << "ERRO! Frames não capturados." << std::endl;
        return -1;
    }

    if(right.size() != left.size()) return -1;

    if(waitKey(30) >= 0){
        imwrite("right"+im+".jpg", right);
        imwrite("left"+im+".jpg", left);
        printf("Get image " + im + "\n");
    }    
    return 0;
}

void calibrarCamera(){
    
}