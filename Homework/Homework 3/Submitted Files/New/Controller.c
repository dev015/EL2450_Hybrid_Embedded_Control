// Different Cases
/*
Case 1: Rotational Controller; V = 0; Task 6
Case 2: V = Place Hold Condition; and or Rotation Control; Task 8 & 9
Case 3: Velocity Controller; W = 0; 
Case 4: Hybrid Controller;
*/
ctrlCase = 4;
errorXi = xg - x0;
errorYi = yg - y0;
switch (ctrlCase) {
	case 1:
		thetaGoal = atan2(errorYi,errorXi) * (180/pi);
		errorTheta = thetaGoal - theta;
		W = kPsi*errorTheta;
		V = 0;
		break;
	case 2:
		thetaGoal = atan2(errorYi,errorXi) * (180/pi);
		errorTheta = thetaGoal - theta;
		//W=0;
		W = kPsi*errorTheta;

		errorX0 = x0 - x;
		errorY0 = y0 - y;
		errorV0 = (cos(theta*pi/180)*errorX0) + (sin(theta*pi/180)*errorY0);

		V = kW * errorV0;

		break;
	case 3:
		W = 0;
		errorX = xg - x;
		errorY = yg - y;
		errorV = (cos(theta*pi/180)*errorX) + (sin(theta*pi/180)*errorY);
		V = kW * errorV;
		break;
	case 4:
		errorX = xg - x;
		errorY = yg - y;
		thetaGoal = atan2(errorY,errorX) * (180/pi);
		errorTheta = thetaGoal - theta;
		errorTheta1 = ((x + (p*cos(theta*pi/180)) - x0)*sin(thetaGoal*pi/180)) - ((y + (p*sin(theta*pi/180)) - y0)*cos(thetaGoal*pi/180));
		errorV = (cos(theta*pi/180)*errorX) + (sin(theta*pi/180)*errorY);
		if(errorTheta >= 0.5 || errorTheta <= -0.5) {
			W = kPsi*errorTheta;
			V = 0;
			Serial.print("Loop 1 is executing");
		}
		else if(abs(errorX) > 0.5 && abs(errorY) > 0.5){
			V = kW * errorV;
			W = 0;
			//W = kPsi_new*errorTheta1;
			Serial.print("Loop 2 is executing");
		}
		else {
			V = kW_goal * errorV;
			W = 0;
			Serial.print("Loop 3 is executing");
			if (errorX == 0 && errorY == 0) {
				send_done();
			}
		}
		break;
}
right = ((2*V) + W) / 2;
left = ((2*V) - W) / 2;
