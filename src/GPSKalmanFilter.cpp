/* 
 * GPSKalmanFilter - a Kalman Filter implementation for GPS .
 * Created by Gabriel Lopes, March, 24, 2022.
 * 
 */

#include "Haversine.h"
#include "Arduino.h"
#include "GPSKalmanFilter.h"
#include <math.h>

GPSKF::GPSKF(double dt, double N, double E, double x_N[2][1], double P_N[2][2], double x_E[2][1], double P_E[2][2])
{
    _dt = dt;
    N = N;
    E = E;
    _x_N[2][1] = x_N[2][1];
    _P_N[2][2] = P_N[2][2];
    _x_E[2][1] = x_E[2][1];
    _P_E[2][2] = P_E[2][2];
}

void GPSKF::filterCalculation(double lat_old, double lng_old, double lat_cur, double lng_cur, double velocity, double acceleration, uint8_t nSatelite, double angle, double &latitude_filter, double &longitude_filter){
    

    	double haver = Haversine().calcDistance(lat_old, lng_old, lat_cur, lng_cur);
	//double bear = Haversine().calcBearing(lat_old, lng_old, lat_cur, lng_cur);
	//Serial.println(bear);
	double bear = angle;
	
	//Serial.print(haver);Serial.print(",");Serial.print(bear);Serial.print(",");
	
	//Serial.print(bear);Serial.print(",");Serial.println(angle);
      
	double aN = 0;
	double vN = 0;
	double Ndist = 0;
	
	double aE = 0;
	double vE = 0;
	double Edist = 0;
	
 	if(nSatelite < 3){
		aN = acceleration*cos(bear);
		vN = (aN)*(_dt) + vNold;
		Ndist = (vN - vNold)*(_dt) + NdistOld;
		
		aE = acceleration*sin(bear);
		vE = (aE)*(_dt) + vEold;
		Edist = (vE - vEold)*(_dt) + EdistOld;

	}else{ 
		Ndist = haver*cos(bear); //deslocamento em N entre lat_anterior e lat_atual
		vN = velocity*cos(bear);
		aN = acceleration*cos(bear);
		//NdistOld = Ndist;
		
		Edist = haver*sin(bear); //deslocamento em N entre lng_anterior e lng_atual
		vE = velocity*sin(bear);
		aE = acceleration*sin(bear);
		//EdistOld = Edist;
	}
	aNold = aN;
	vNold = vN;
	NdistOld = Ndist;
	aEold = aE;
	vEold = vE;
	EdistOld = Edist;
	
    	N = N + Ndist;
	//Serial.print(N);//Serial.print(",");

    	E = E + Edist;
	//Serial.print(E);//Serial.print(",");

      
    	//VARIABLES TO CALCULATE THE FILTER
    	//F -> Transition State Matrix
    	double F[2][2] = {{1, _dt},{0,  1}};
	//B -> Update matrix 
	double B[2][1] = {{pow(_dt,2)/2},{_dt}};
    	//H -> Channel Distortion filters the state vector for the wanted variables
    	double H[2][2] = {{1, 0},{0, 1}};
    	//Variance used in the Q matrix
    	double Q_var = 0.1;
    	//R -> Sensor covariance matrix
    	double R[2][2] = {{6.25, 0},{0, 5}};
    	//Q -> process noise matrix (computes the noise)
    	double Q[2][2] = {{0.25 * pow(_dt,4.0) * Q_var, 0.5 * pow(_dt,3.0) * Q_var},{0.5 * pow(_dt,3.0) * Q_var, pow(_dt,2.0) * Q_var}};
    
	double H_T[2][2] = {{0,0},{0,0}};
	double F_T[2][2] = {{0,0},{0,0}};

    	//iniciar modificações da biblioteca
    	for(uint8_t i=0; i<2; i++){
		for(uint8_t j=0; j<2; j++){
	    		F_T[j][i] = F[i][j];}}
  
    	for(uint8_t i=0; i<2; i++){
        	for(uint8_t j=0; j<2; j++){
            		H_T[j][i] = H[i][j];}}

    	for(uint8_t NE = 0; NE < 2; NE++){

		//variables to help with the calculations
		double aux1[2][2] = {{0, 0},{0, 0}};
		double aux2[2][2] = {{0, 0},{0, 0}};
		double aux3[2][2] = {{0, 0},{0, 0}};
		double aux4[2][2] = {{0, 0},{0, 0}};
		double aux5[2][2] = {{0, 0},{0, 0}};
		double aux6[4][2] = {{0, 0},{0, 0},{0, 0},{0, 0}};
		double aux7[2][1] = {{0},{0}};
		double aux8[2][2] = {{0, 0},{0, 0}}; 
		double aux9[2][2] = {{0, 0},{0, 0}};
		double detCalc;
		double auxaux;

		double K[2][2] = {{0,0},{0,0}};
		double y[2][1] = {{0},{0}};
		double auxx[2][1] = {{0},{0}};
		double auxP[2][2] = {{0, 0},{0, 0}};
		double auxx2[2][1] = {{0},{0}};
		double auxx3[2][1] = {{0},{0}};

		if(NE == 0){

		    //calculation for predict *x = F @ x* + B @ u
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    for(uint8_t k=0; k<2; k++){
				auxx[i][j] += F[i][k] * _x_N[k][j];}}}
						
		  //calculation for predict x = F @ x + *B @ u*
		  for(uint8_t i=0; i<2; i++){
		  	for(uint8_t j=0; j<1; j++){
				auxx3[i][j] += B[i][j] * aN;}}
                
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    _x_N[i][j] = auxx[i][j] + auxx3[i][j];}}


		    //calculation for predict P =  F @ P **@ F.T + Q**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux1[i][j] += F[i][k] * _P_N[k][j];}}}
                    
            
		    //calculation for predict P =  F @ P @ F.T **+ Q**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux2[i][j] += aux1[i][k] * F_T[k][j];}}}

		    //calculation for predict P =  F @ P @ F.T + Q
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    auxP[i][j] = aux2[i][j] + Q[i][j];}}


		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    _P_N[i][j] = auxP[i][j];}}

		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    _P_N[i][j] = auxP[i][j];}}


		    //calculation for update S = H @ P **@ H.T + R**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux3[i][j] += H[i][k] * _P_N[k][j];}}}


		    //calculation for update S = H @ P @ H.T **+ R**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
			    aux4[i][j] += aux3[i][k] * H_T[k][j];}}}

		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    aux5[i][j] = aux4[i][j] + R[i][j];}}


		    //calculation for update K = P @ H.T **@ inv(S)
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux6[i][j] += _P_N[i][k] * H_T[k][j];}}}

		    //calculate det aux5
		    detCalc = (1/((aux5[0][0]*aux5[1][1]) - (aux5[0][1]*aux5[1][0])));
		    //Serial.println(detCalc, 6);
		    auxaux = aux5[0][0];
		    aux5[0][0] = detCalc * aux5[1][1];
		    aux5[1][1] = detCalc * auxaux;
		    aux5[0][1] = detCalc * aux5[0][1] * (-1);
		    aux5[1][0] = detCalc * aux5[1][0] * (-1);

		    //calculation for update K
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				K[i][j] += aux6[i][k] * aux5[k][j];}}}

		    //calculation for residual y
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    for(uint8_t k=0; k<2; k++){
				aux7[i][j] += H[i][k] * _x_N[k][j];}}}

		    y[0][0] = N - aux7[0][0];
		    y[1][0] = vN - aux7[1][0];

		    //calculation for x 
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    for(uint8_t k=0; k<2; k++){
				auxx2[i][j] += K[i][k] * y[k][j];}}}

		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    _x_N[i][j] += auxx2[i][j];}}

		    //calculation for P
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux8[i][j] += K[i][k] * H[k][j];}}}

		    //calculation for P
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux9[i][j] += aux8[i][k] * _P_N[k][j];}}}

		    //calculation for P
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    _P_N[i][j] = _P_N[i][j] - aux9[i][j];}}

            	    //N = _x_N[0][0];
		    //Serial.print(_x_N[0][0]);//Serial.print(",");
      
		}else if(NE == 1){
		    //calculation for predict x = F @ x
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    for(uint8_t k=0; k<2; k++){
				auxx[i][j] += F[i][k] * _x_E[k][j];}}}
					
		    //calculation for predict x = F @ x + *B @ u*
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
						auxx3[i][j] += B[i][j] * aE;}}

		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    _x_E[i][j] = auxx[i][j] + auxx3[i][j];}}

		    //calculation for predict P =  F @ P **@ F.T + Q**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux1[i][j] += F[i][k] * _P_E[k][j];}}}


		    //calculation for predict P =  F @ P @ F.T **+ Q**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux2[i][j] += aux1[i][k] * F_T[k][j];}}}

		    //calculation for predict P =  F @ P @ F.T + Q
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    auxP[i][j] = aux2[i][j] + Q[i][j];}}
                
            
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    _P_E[i][j] = auxP[i][j];}}

		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    _P_E[i][j] = auxP[i][j];}}

		    //calculation for update S = H @ P **@ H.T + R**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux3[i][j] += H[i][k] * _P_E[k][j];}}}


		    //calculation for update S = H @ P @ H.T **+ R**
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux4[i][j] += aux3[i][k] * H_T[k][j];}}}

		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    aux5[i][j] = aux4[i][j] + R[i][j];}}

		    //calculation for update K = P @ H.T **@ inv(S)
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux6[i][j] += _P_E[i][k] * H_T[k][j];}}}

		    //calculate det aux5
		    detCalc = (1/((aux5[0][0]*aux5[1][1]) - (aux5[0][1]*aux5[1][0])));
		    //Serial.println(detCalc, 6);
		    auxaux = aux5[0][0];
		    aux5[0][0] = detCalc * aux5[1][1];
		    aux5[1][1] = detCalc * auxaux;
		    aux5[0][1] = detCalc * aux5[0][1] * (-1);
		    aux5[1][0] = detCalc * aux5[1][0] * (-1);

		    //calculation for update K
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				K[i][j] += aux6[i][k] * aux5[k][j];}}}


		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    for(uint8_t k=0; k<2; k++){
				aux7[i][j] += H[i][k] * _x_E[k][j];}}}

		    y[0][0] = E - aux7[0][0];
		    y[1][0] = vE - aux7[1][0];

		    //calculation for x 
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    for(uint8_t k=0; k<2; k++){
				auxx2[i][j] += K[i][k] * y[k][j];}}}

		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<1; j++){
			    _x_E[i][j] += auxx2[i][j];}}

		    //calculation for P
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux8[i][j] += K[i][k] * H[k][j];}}}

		    //calculation for P
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    for(uint8_t k=0; k<2; k++){
				aux9[i][j] += aux8[i][k] * _P_E[k][j];}}}

		    //calculation for P
		    for(uint8_t i=0; i<2; i++){
			for(uint8_t j=0; j<2; j++){
			    _P_E[i][j] = _P_E[i][j] - aux9[i][j];}}

		    //E = _x_E[0][0];
		    //Serial.print(_x_E[0][0]);//Serial.print(",");
			
      }
    }
	
	double rN = _x_N[0][0] - auxN;
    	double rE = _x_E[0][0] - auxE;
    
   	 double br = atan(rE / rN);
    
    	double hv = rN/cos(bear);
    
    	double lat2, lon2;
    	double dist = hv / 6378140;
	lat_old = Haversine().dtor(lat_old);
    	lng_old = Haversine().dtor(lng_old);  
    	lat2 = asin(sin(lat_old) * cos(dist) + cos(lat_old) * sin(dist) * cos(bear));
    	lon2 = lng_old + atan2( sin(bear) * sin(dist) * cos(lat_old), cos(dist) - sin(lat_old) * sin(lat2) );

    	latitude_filter =  Haversine().rtod(lat2);
    	longitude_filter = Haversine().rtod(lon2);
    
    	auxN = _x_N[0][0];
    	auxE = _x_E[0][0];
	
}
