#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/gapi/render/render_types.hpp>
#include "lijn.hpp"
#include "motorcontrol.h"
#include "pid.hpp"
#include <iostream>

//grootte van de referentiemarker in CM
float X_VAN_REFMARK = 2.1;
float Y_VAN_REFMARK = 2.1;

float PIX_PER_CM_X;
float PIX_PER_CM_Y;

//hoek waar die de detectiemarkers dienen aan te namen.
int gewensteHoek = 180;

Point2f testSetP(-1,-1);

Point2f REF_PUNT;

float HOEK_REF;

using namespace cv;
using namespace std;

// geef een punt mee uit het camera beeld dat rond het referentiepunt zal roteren. Camerabeeld --> referentiemarker
Point2f rotMatrixRef(Point2f p){
    float x = REF_PUNT.x+(p.x-REF_PUNT.x)*cos(HOEK_REF)-(p.y-REF_PUNT.y)*sin(HOEK_REF);
    float y = REF_PUNT.y+(p.x-REF_PUNT.x)*sin(HOEK_REF)+(p.y-REF_PUNT.y)*cos(HOEK_REF);
    return Point2f(x,y);
}

//as referentiemarker --> camerabeeld
Point2f invRotMatrixRef(Point2f p){
    float x = REF_PUNT.x+(p.x-REF_PUNT.x)*cos(HOEK_REF)+(p.y-REF_PUNT.y)*sin(HOEK_REF);
    float y = REF_PUNT.y-(p.x-REF_PUNT.x)*sin(HOEK_REF)+(p.y-REF_PUNT.y)*cos(HOEK_REF);
    return Point2f(x,y);
}

// sla pixelwaarde van de muisklik op in testSetP
void muis_click(int event,int x,int y,int flags,void *param){
    if (event == EVENT_LBUTTONDOWN){
        cout << "x: " <<x << " y: " <<y <<endl;
        testSetP = Point2f(x,y);
    }
}

Point2f xAsRot;
Point2f yAsRot;
int main(){

    Init_Motor_Control();
    // Maak frame aan

    Mat frame;
    // Lees standaard video uit
    VideoCapture v;
    v.open(0);

    // video naar frame 
    v >> frame;
    Mat outputFrame;
	
    // PIDs aanmaken
    PID pidx = PID(0.033,100,-100,0.5,0.01,0);
    PID pidy = PID(0.033, 100,-100,0.5,0.01,0);
    //PID pidx = PID(0.033,100,-100,0,0.0,0);
    //PID pidy = PID(0.033, 100,-100,0.0,0.0,0);
    PID pidz = PID(0.033, 100,-100,0.5,0.01,0.5);

    // start loop als er een frame beschikbaar is
    while(v.grab()){
	// laad laatste frame
        v >> frame;
        outputFrame =frame.clone();

	// laad aruco bib
        Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
        
	// maak lijst waar ids en hoeken van de gedetecteerde markers worden opgeslaan.
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
	    
	// detecteer de markers en omtrek deze
        aruco::detectMarkers(frame,dictionary,corners,ids);
        aruco::drawDetectedMarkers(outputFrame,corners,ids);

	// Voor elke marker doe...
        for (int i =0;i<corners.size(); i++){	
	    // indien detextiemarker (id=1)
            if (ids[i] == 1){
                // Midpunt van de ref marker bepalen
                Lijn schuin(corners[i][0], corners[i][2]);             
                circle(outputFrame,schuin.getMidPunt(),5,Scalar(0,0,255),4);
                REF_PUNT = schuin.getMidPunt();
		
		// zet setpunt op detectiewaarde
                if (testSetP.x == -1){
                    testSetP = REF_PUNT;
                }

                // bereken hoek
                Lijn onder(corners[i][0],corners[i][3]);
                HOEK_REF = onder.getHoekTovXAs(true);

                // lijn links
                Lijn links(corners[i][0],corners[i][1]);

		// bepaal aantal pixels per CM
                PIX_PER_CM_X = onder.getLengte()/X_VAN_REFMARK;
                PIX_PER_CM_Y = links.getLengte()/Y_VAN_REFMARK;

                //putText(outputFrame,to_string(onder.getHoekTovXAs()),onder.getP2(),1,2,Scalar(0,255,0));
		    
		//teken as van 10cm
                float x_as = REF_PUNT.x + 10*PIX_PER_CM_X;
                float y_as = REF_PUNT.y + 10*PIX_PER_CM_Y;

                //testSetP = Point2f(REF_PUNT.x+50,REF_PUNT.y+50);

                //circle(outputFrame,rotMatrixRef(testSetP),5,Scalar(0,255,0),5);
		
		// roteer de x-en y-as naar het referentieassenstelsel

                xAsRot = rotMatrixRef(Point2f(x_as,REF_PUNT.y));
                yAsRot = rotMatrixRef(Point2f(REF_PUNT.x,y_as));
                //cout << "xRef: " << PIX_PER_CM_Y;

		// teken de punten van het assenstelsel
                line(outputFrame,REF_PUNT, xAsRot, Scalar(0,255,255),2);
                line(outputFrame,REF_PUNT, yAsRot, Scalar(0,255,255),2);
                
            }
		// Bij elke andere gedetecteerde marker (id != 1)
            else{
		//bepaal hoekpunten, middelpunten en rotatie.
                Lijn detectieMarker(corners[i][0], corners[i][2]);             
                circle(outputFrame,detectieMarker.getMidPunt(),5,Scalar(0,255,255),4);

                Lijn detectieMarkerOnder(corners[i][0], corners[i][3]);
                float detectHoek = detectieMarkerOnder.getHoekTovXAs()+180; //-  HOEK_REF*(180/M_PI);

                putText(outputFrame,format("%3.0lf", detectHoek),corners[i][3],1,2,Scalar(255,0,0),2);
                
                //bereken de x en y afstand van de marker tot het setp.
                Point2f dM = invRotMatrixRef(detectieMarker.getMidPunt());
                Point2f sP = invRotMatrixRef(testSetP);
                
                float x = sP.x - dM.x;
                float y = sP.y - dM.y;



                cout << "x: " <<x << " y: " <<y <<endl;
		//
		//
		cout << "detect hoek: " << detectHoek+360;
		    
		// bepalen van de hoekverschillen tussen de detectiehoek en de gewenste hoek telkens rekening houdend metde kortste hoek.
		int error;
		if(abs(gewensteHoek-detectHoek)<180){
			error = gewensteHoek-detectHoek;
		}
		else{
			error = -(360-(abs(gewensteHoek-detectHoek)));
			if (gewensteHoek>detectHoek){
				error = -error;
			}

		}
		// Bereken de stuurwaarde X, Y en Z met de vooraf ingestelde waarde 
                double pX = pidx.calculate(sP.x, dM.x);
                double pY = -pidy.calculate(sP.y, dM.y);
		
		double pZ = pidz.calculate(0,error);

		// vertaal piwelwaarde naar cm
               	double xW, yW,zW;
	       	xW= x/PIX_PER_CM_X;
		yW = y/PIX_PER_CM_Y;
		cout << "waarde ::::::::: XW:"<< xW <<" YW:"<<yW<<endl;
		setZwaartepunt(xW,yW,0.0);
		cout << "x: " <<pX << " y: " <<pY <<endl;

		// x,y en z waarde naar motorcontroller die deze zal vertalen naar motorspecifieke waarde.
                set_robot_speed(pX,pY,pZ);


            }
    	    // Zet motors stil wanneer geen markers of enkel de refmarker wordt gedetecteerd.
            if (ids.size() == 0 || (ids.size() == 1 && ids[0] == 1)){
                    set_motor_speed(1, 0);
                    set_motor_speed(2, 0);
                    set_motor_speed(3, 0);
            }
        }
	// Callback event naar muis_click
        setMouseCallback("live", muis_click);

        // teken assenstelsel volgens 
        line(outputFrame,REF_PUNT, xAsRot, Scalar(0,255,255),2);
        line(outputFrame,REF_PUNT, yAsRot, Scalar(0,255,255),2);
        
        circle(outputFrame,testSetP,5,Scalar(0,255,0),5);


        imshow("live",outputFrame);
	
        
        if(waitKey(1) >= 0) break;
    }

    set_motor_speed(1, 0);
    set_motor_speed(2, 0);
    set_motor_speed(3, 0);

}
