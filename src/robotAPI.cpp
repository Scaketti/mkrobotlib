#include "robotAPI.hpp"

int main(int argc, char *argv[]){
	
	if(wiringPiSetupGpio() == -1) return -1;

    configuraIOs();

    int i;
    criaDatasetCalib();
    calibrarCamera();
    
    //for(i = 0; i < 1; i++){
        //moveDistancia('f', 1); //se movimenta para frente por um metro
        //vira('d', 1); //vira para direita por 1 segundos
    //}

	return 0;
}

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

void resetaVariaveis(){
    flag[0] = flag[1] = flag[2] = flag[3] = 0;
    seg_cont[0] = seg_cont[1] = seg_cont[2] = seg_cont[3] = SEG_NUM; 
    dist_total = 0; 
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
        motorControledMove(SENSOR_ED, SENSOR_ED_DIST_POS);
        motorControledMove(SENSOR_DD, SENSOR_DD_DIST_POS);
        motorControledMove(SENSOR_ET, SENSOR_ET_DIST_POS);
        motorControledMove(SENSOR_DT, SENSOR_DT_DIST_POS);
    }

    dist_total = 0;
    para();
    resetaVariaveis();
}

void moveTempo(char sentido, float tempo){
    if(sentido == 'f')
        frente();
    else
        re();

    usleep(tempo*1000000);

    para();
}

void motorControledRot(int sensorID, int sensorID_POS) {
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

void viraAngulo(char sentido, int angulo){
    if(sentido == 'd')
        direita();
    else
        esquerda();

    angulo = angulo * 0.2;
    
    while(dist_total/4 < angulo){ //média de distancia percorrida pelos quatro motores
        //if(sentido == 'd'){
        motorControledRot(SENSOR_ED, SENSOR_ED_DIST_POS);
        motorControledRot(SENSOR_ET, SENSOR_ET_DIST_POS);
        //}else{
        motorControledRot(SENSOR_DD, SENSOR_DD_DIST_POS);
        motorControledRot(SENSOR_DT, SENSOR_DT_DIST_POS);
        
    }

    dist_total = 0;
    para();
    resetaVariaveis();
}

void viraTempo(char sentido, int tempo){
    if(sentido == 'd')
        direita();
    else
        esquerda();
    
    usleep(tempo*1000000);

    para();
}


int getImage(){
    
    cout << "Sistema Inicializado." << endl;

//    VideoCapture leftcap(0);
//    VideoCapture rightcap(2);

//    if(!leftcap.isOpened() || !rightcap.isOpened())  // check if we succeeded
  //      return -1;

    Mat left, right;
	
    //namedWindow( "left", CV_WINDOW_AUTOSIZE ); // Create a window for display.
    //namedWindow( "right", CV_WINDOW_AUTOSIZE ); // Create a window for display.
	//tire os comentários caso queira verificar ao vivo
   // for(;;){
        if(!(leftcap.grab() && rightcap.grab())) return -1;

        leftcap >> left;
        rightcap >> right;

        if(left.empty() || right.empty())
        {
            std::cerr << "ERRO! Frames não capturados." << std::endl;
            return -1;
        }

        if(right.size() != left.size()) return -1;

        
//        imshow( "right", right ); // Show our image inside it.
  //      imshow( "left", left ); // Show our image inside it.

//        if(waitKey(30) >= 0){
            imwrite("right.jpg", right);
            imwrite("left.jpg", left);
  //          break; 
    //    }    
   // }
    return 0;
}

int calibrarCamera(){
    Size boardSize(9,6);

    const float squareSize = 22; //tamanho da célula do alvo

    Mat left_f, right_f;

    std::vector<std::vector<Point2f> > leftImagePoints(nImage);
    std::vector<std::vector<Point2f> > rightImagePoints(nImage);
    std::vector<std::vector<Point3f> > objectPoints(nImage);

    Size imageSize;

    Mat cameraMatrix[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

    Mat distortionCoefficients[2];
    Mat rotationMatrix;
    Mat translationVector;
    Mat essentialMatrix;
    Mat fundamentalMatrix;

    double rms = 0;

    for(int curImg = 0; curImg < nImage; curImg++){

    left_f = imread("calib_dataset/left_" + to_string(curImg) + ".jpg", 0);
    right_f = imread("calib_dataset/right_" + to_string(curImg) + ".jpg", 0);

    if(left_f.empty() || right_f.empty())
    {
        std::cerr << "ERRO! Frames não capturados." << std::endl;
        return -1;
    }

    imageSize = left_f.size();

    if(right_f.size() != imageSize )
    {
        std::cerr << "ERRO! Frames precisam ser da mesma resolução." << std::endl;
        return -1;
    }

    bool left_found = findChessboardCorners(left_f, boardSize, leftImagePoints[curImg]);
    bool right_found = findChessboardCorners(right_f, boardSize, rightImagePoints[curImg]);

    if(!(left_found && right_found))
    {
        std::cerr << "ERRO! Alvo de calibração não encontrado." << std::endl;
        return -1;
    }

    //melhora os pontos encontrados no alvo e os refina
    cv::cornerSubPix(left_f, leftImagePoints[curImg], cv::Size(5, 5), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    cv::cornerSubPix(right_f, rightImagePoints[curImg], cv::Size(5, 5), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    drawChessboardCorners(left_f, boardSize, leftImagePoints[curImg], left_found);
    drawChessboardCorners(right_f, boardSize, rightImagePoints[curImg], right_found);

    objectPoints[curImg] = Create3DChessboardCorners(boardSize, squareSize);
    }

    rms = stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints,
                    cameraMatrix[0], distortionCoefficients[0],
                    cameraMatrix[1], distortionCoefficients[1],
                    imageSize, rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix,
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));

    //armazeno o resultado obtido através da calibração em um arquivo
    cv::FileStorage file("calib.txt", cv::FileStorage::WRITE);
    file << "rms" << rms;

    file << "cameraL" << cameraMatrix[0];
    file << "cameraR" << cameraMatrix[1];

    file << "distCoefL" << distortionCoefficients[0];
    file << "distCoefR" << distortionCoefficients[1];

    file << "rotMatrix" << rotationMatrix;
    file << "translVec" << translationVector;
    file << "essMatrix" << essentialMatrix;
    file << "fundMatrix" << fundamentalMatrix;
}

int criaDatasetCalib(){
    Size boardSize(9,6);
    int curImg = 0;
    int nImages = 0;

    cout << "Digite a quantidade de imagens para a calibração: ";
    cin >> nImages;

    if(nImages == 0) return 0;

    std::vector<std::vector<Point2f>> leftImagePoints(nImages);
    std::vector<std::vector<Point2f>> rightImagePoints(nImages);

    Mat left_f, right_f, left_chess, right_chess;
    cout << "OK\n";
    while(true){
        cout << "OK1\n";
        if(!(leftcap.grab() && rightcap.grab())) return -1; //método "grab" garante que o frame da câmera da esquerda seja capturado
                                                        //ao mesmo tempo que o frame da câmera da direita
cout << "OK2\n";
        leftcap.read(left_f);
        rightcap.read(right_f);
cout << "OK3\n";
        if(left_f.empty() || right_f.empty())
        {
            std::cerr << "ERRO! Frame não capturado." << std::endl;
            return -1;
        }


        if(right_f.size() != left_f.size())
        {
            std::cerr << "ERRO! Imagens não possuem a mesma resolução." << std::endl;
            return -1;
        }
cout << "OK4\n";
        bool left_found = findChessboardCorners(left_f, boardSize, leftImagePoints[curImg]);
        bool right_found = findChessboardCorners(right_f, boardSize, rightImagePoints[curImg]);

       // left_chess = left_f.clone();
       // right_chess = right_f.clone();

       // drawChessboardCorners(left_chess, boardSize, leftImagePoints[curImg], left_found);
      //  drawChessboardCorners(right_chess, boardSize, rightImagePoints[curImg], right_found);

       // imshow("Left View", left_chess);
       // imshow("Right View", right_chess);
cout << "OK5\n";
        //caso o alvo esteja presente nas duas imagens e o usuário pressionou alguma tecla, crie uma imagem para a calibração
        if(/*(waitKey(2) > 0) && */(left_found && right_found)) 
            {
            cout << "Print " << curImg << "!" << endl; 

            imwrite("calib_dataset/left_" + to_string(curImg) + ".jpg", left_f);

            imwrite("calib_dataset/right_" + to_string(curImg) + ".jpg" , right_f);
            curImg++;
            }
        if(curImg > (nImages-1)) break;
        cout << "TENTE TIRAR";
    }

    return 0;
}

int stereo(){
    FileStorage readFile("calib.txt", FileStorage::READ);

    Mat cameraL,
        cameraR,
        distortionCoefficientsL,
        distortionCoefficientsR,
        rotationMatrix,
        translationVector,
        essentialMatrix,
        fundamentalMatrix;

    readFile["cameraL"] >> cameraL;
    readFile["cameraR"] >> cameraR;
    readFile["distCoefL"] >> distortionCoefficientsL;
    readFile["distCoefR"] >> distortionCoefficientsR;
    readFile["rotMatrix"] >> rotationMatrix;
    readFile["translVec"] >> translationVector;
    readFile["essMatrix"] >> essentialMatrix;
    readFile["fundMatrix"] >> fundamentalMatrix;

    //tire os comentários caso queira verificar ao vivo
    //VideoCapture leftcap(0);
    //VideoCapture rightcap(1);

    Mat left, right, disp;

    Rect roi1, roi2;

    Mat rectTransL, rectTransR,
        projMatrixL, projMatrixR, 
        disp_depthMatrix;

    Size newImageSize;

    int frame2_width = rightcap.get(CV_CAP_PROP_FRAME_WIDTH); 
    int frame2_height = rightcap.get(CV_CAP_PROP_FRAME_HEIGHT); 


//    VideoWriter video("mapaDisparidades.avi",CV_FOURCC('M','J','P','G'), rightim.get(CV_CAP_PROP_FPS), Size(frame2_width,frame2_height));


    while(true){

        //tire os comentários caso queira verificar ao vivo
        if(!(leftcap.grab() && rightcap.grab())) return -1;

        leftcap.read(left);
        rightcap.read(right);

        if(left.empty() || right.empty())
        {
            std::cerr << "ERRO! Frames não capturados." << std::endl;
            return -1;
        }

        if(right.size() != left.size()) return -1;

        cvtColor(left, left, COLOR_RGB2GRAY);
        cvtColor(right, right, COLOR_RGB2GRAY);

        //realiza a retificação das imagens
        stereoRectify(cameraL, distortionCoefficientsL,
                        cameraR, distortionCoefficientsR,
                        Size(640,480), rotationMatrix, translationVector,
                        rectTransL, rectTransR,
                        projMatrixL, projMatrixR,
                        disp_depthMatrix, CALIB_ZERO_DISPARITY, -1, newImageSize, &roi1, &roi2);

        Mat newCameraL, newCameraR, left_map1, left_map2, right_map1, right_map2;

        Mat left_und, right_und;

        initUndistortRectifyMap(cameraL, distortionCoefficientsL,
                                rectTransL, projMatrixL, Size(left.cols, left.rows),
                                CV_16SC2, left_map1, left_map2);
        initUndistortRectifyMap(cameraR, distortionCoefficientsR,
                                rectTransR, projMatrixR, Size(right.cols, right.rows),
                                CV_16SC2, right_map1, right_map2);

        remap(left, left_und, left_map1, left_map2, INTER_LINEAR);
        remap(right, right_und, right_map1, left_map2, INTER_LINEAR);


        GaussianBlur(left, left, Size(5,5), 10, 10, BORDER_DEFAULT);
        GaussianBlur(right, right, Size(5,5), 10, 10, BORDER_DEFAULT);

        Ptr<StereoBM> bm = StereoBM::create(0,21);

        //bm->setROI1(roi1);
        //bm->setROI2(roi2);
        bm->setPreFilterCap(31);
        bm->setBlockSize(15);
        bm->setMinDisparity(0);
        bm->setNumDisparities(64);
        bm->setTextureThreshold(1000);
        bm->setUniquenessRatio(15);
        bm->setSpeckleWindowSize(100);
        bm->setSpeckleRange(32);
        bm->setDisp12MaxDiff(10);

        //calcula o mapa de disparidades a partir das imagens retificadas
        bm->compute(left_und, right_und, disp);

        //como o mapa de disparidade está muito saturado, normalizamos a imagem para podermos visualizar o mapa de forma clara
        normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);

        //muda-se a apresentação das intensidades para melhorar a visualização
        applyColorMap(disp, disp, COLORMAP_JET);

//        video.write(disp);

        imshow("Mapa de disparidades", disp);

        if(waitKey(2) > 0) break;
    }
    return 0;
}

std::vector<Point3f> Create3DChessboardCorners(Size boardSize, float squareSize){
  // This function creates the 3D points of your chessboard in its own coordinate system

  std::vector<Point3f> corners;

  for( int i = 0; i < boardSize.height; i++ )
  {
    for( int j = 0; j < boardSize.width; j++ )
    {
      corners.push_back(Point3f(float(j*squareSize),
                                float(i*squareSize), 0));
    }
  }

  return corners;
}

void teste1(){
    moveTempo('f', 1);
    viraTempo('d', 2);
    viraTempo('e', 2);
    moveTempo('r', 1);
}

void teste2(){
    moveDistancia('f', 30);
    viraAngulo('d', 90);
    moveDistancia('r', 10);
    viraAngulo('e', 45);
    moveDistancia('f', 10);
    viraAngulo('e', 45);
}

void teste3(){
    moveDistancia('f', 20);
}

void teste4(){
    moveDistancia('f', 5);
    
    usleep(1000000);
    moveDistancia('f', 5);
    
    usleep(1000000);
    moveDistancia('f', 5);
}
