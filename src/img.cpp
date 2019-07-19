#include <stdio.h>    
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <iostream>

using namespace std;
using namespace cv;

//Programa principal
int main( int argc, char *argv[] )
{
    cout << "Sistema Inicializado." << endl;

    //tire os comentários caso queira verificar ao vivo
    VideoCapture leftcap(0);
    VideoCapture rightcap(1);

    if(!leftcap.isOpened() || !rightcap.isOpened())  // check if we succeeded
        return -1;

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

        
        //imshow( "right", right ); // Show our image inside it.
        //imshow( "left", left ); // Show our image inside it.

//        if(waitKey(30) >= 0){
            imwrite("right.jpg", right);
            imwrite("left.jpg", left);
  //          break; 
    //    }    
   // }                   // Wait for a keystroke in the window
    return 0;
    
}
