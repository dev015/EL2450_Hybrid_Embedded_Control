left = 0;
right = 0;

Kt = 10;
Kr = 5;
Kp = 10;
p = 20;
// e_Kt=sqrt(((yg -y0)*(yg -y0))+((xg - x0)*(xg - x0)));


dy = (yg -y);
dx = (xg - x);
goal = atan2(dy,dx)*180/pi;

roterror = goal-theta;
//Serial.print(roterror);

//e_Kt = sqrt((dy*dy)+(dx*dx)); 
dg = (cos(goal*pi/180) *(xg -x) + sin(goal*pi/180)*(yg -y));

d0= (cos(theta*pi/180) *(x0 -x) + sin(theta*pi/180)*(y0 -y));

dp = sin(goal*pi/180)*(x+p*cos(theta*pi/180) -x0) - cos(goal*pi/180)*(y+p*sin(theta*pi/180) -y0);


//finalr = Kr*roterror; //q6
//finalt = Kt*d0; //q8

finalt = Kt * dg; //q11
finalr = Kp*dp; //q14

/*if(theta == goal+5 || theta == goal-5)
{
left = left + finalt;
right = right + finalt;
}
else
{
left = left - finalr/2;
right = right + finalr/2;
}*/

if(roterror>-2 || roterror<2)
{
left = left + finalt;
right = right + finalt;
}
else
{
left = left - finalr/2;
right = right + finalr/2;
}



