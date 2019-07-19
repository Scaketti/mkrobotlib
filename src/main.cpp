#include "robotAPI.hpp"

int main(int argc, char *argv[]){
	
	if(wiringPiSetupGpio() == -1) return -1;

    configuraIOs();

    int i;

    //for(i = 0; i < 1; i++){
        //moveDistancia('f', 1); //se movimenta para frente por um metro
        //vira('d', 1); //vira para direita por 1 segundos
    //}

	return 0;
}
