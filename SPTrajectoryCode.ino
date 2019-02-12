//*********************************************************************STEWART PLATFORM TRAJECTORY TRACKING****************************************************************

#include<Servo.h>
Servo s1,s2,s3,s4,s5,s6;

void setup() {
  // put your setup code here, to run once:

s1.attach(2);
s2.attach(3);
s3.attach(4);
s4.attach(5);
s5.attach(6);
s6.attach(7);

}

void loop() {
  // put your main code here, to run repeatedly:
  
//length=zeros(6,1);

  float x=0.03;
  float y=0.03;
  float vert=0.0;
  float z=0.293+vert; //29.3cm is the distance between two platforms.This length can vary depending on the lengths of the legs of the platform
  float phi=0.0;
  float theta=0.0;
  float psi=0.0;
  
  float gammap1,gammab1,rp,rb,hp;
  
  float pi=3.14159;
  int i,j,k;
  float leg=0.3202;//This is the fixed leg length


gammap1=8; //Look at the figure to know what this angle is 
float gammap[6] = {gammap1,120-gammap1, 120+gammap1, -120-gammap1,-120+gammap1,-gammap1};//These are the angles between the X axis and the position on the platform where
                                                                                         //legs are attached to the platform

gammab1 = 20; // Look at the figure to know what this angle is 
float gammab[6] = {gammab1,120-gammab1, 120+gammab1, -120-gammab1,-120+gammab1,-gammab1};//These are the angles between the X axis and the location of the servos from the 
                                                                                        //origin
rp = 0.10; // radius of platform ,in meter
rb = 0.20;  // radius of base,in meter
hp = 0.01;   // platform nominal height ,in meter


  
float spsi = sin(psi*pi/180 );
float cpsi = cos(psi*pi/180 );
float sthe = sin(theta*pi/180 );
float cthe = cos(theta*pi/180 );
float sphi = sin(phi*pi/180 );
float cphi = cos(phi*pi/180 );




float bRp[3][3];// rotation matrix R
bRp[0][0] = cphi * cthe ;
bRp[0][1] = -sphi * cpsi + cphi * sthe * spsi ;
bRp[0][2] = sphi * spsi + cphi * sthe * cpsi ;
bRp[1][0] = sphi * cthe ;
bRp[1][1] = cphi * cpsi + sphi * sthe * spsi ;
bRp[1][2] = -cphi * spsi + sphi * sthe * cpsi ;
bRp[2][0] = -sthe ;
bRp[2][1] = cthe * spsi ;
bRp[2][2] = cthe * cpsi ;
// fixed vector of pp


float pp[3][6];// Fixed vector of pp-Vectors from platform origin to platform leg attachment points on the platform

  for(j=0;j<6;j++)
  {
    pp[0][j]=rp*cos(gammap[j]*pi/180);
  }

   for(j=0;j<6;j++)
   {
    pp[1][j]=rp*sin(gammap[j]*pi/180);
   }

  for(j=0;j<6;j++)
  {
    pp[2][j]=hp/2;
  }

 
  //Serial.println(pp[1][3],4);
  // Serial.println(pp[2][1],4);
 
 float bb[3][6];//Fixed vector of bb:bb contains the vector from the base origin to the points on the base where servos are kept

for(j=0;j<6;j++)
  {
    bb[0][j]=rb*cos(gammab[j]*pi/180);
  }

   for(j=0;j<6;j++)
   {
    bb[1][j]=rb*sin(gammab[j]*pi/180);
   }

  for(j=0;j<6;j++)
  {
    bb[2][j]=0;
  }







float bdp[3]={x,y,z};//bdp is the vector from the base to platform origin

//bDp = {bdp,bdp,bdp,bdp,bdp,bdp} ;
float bDp[3][6];

for(i=0;i<3;i++)
{
  for(j=0;j<6;j++)
  {
    bDp[i][j]=bdp[i];
  }
}

//bDp = {bdp,bdp,bdp,bdp,bdp,bdp} ;
float qq[3][6];
 for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 6; j++)
        {
           qq[i][j] = 0;
            for (k = 0; k < 3; k++)
            {
               qq[i][j] += bRp[i][k] * pp[k][j];
            }
        }
    }

for(i=0;i<3;i++)
{
  for(j=0;j<6;j++)
  {
    qq[i][j]=qq[i][j]+bDp[i][j];
  }
}

//qq=bRp*pp+bDp;
//bA = bRp*pp +bDp - bb ;

//bA contains the actuator vectors
float bA[3][6];
for(i=0;i<3;i++)
{
  for(j=0;j<6;j++)
  {
    bA[i][j]=0;
  }
}


for(i=0;i<3;i++)
{
  for(j=0;j<6;j++)
  {
    bA[i][j]=bA[i][j]+qq[i][j]-bb[i][j];
  }
}
//bA;

float L[3][6];//L contains the coordinates of the platform attachment points from a reference frame where location of the servo is the origin
              //This is done for ease of calculation: Refer report

float rot[3][3][6];//rot contains all the 6 rotation matrices to transform the base origin to each of the 6 servo origins

for(k=0;k<6;k++)
{
rot[0][0][k]=cos(gammab[k]*pi/180);
rot[0][1][k]=sin(gammab[k]*pi/180);
rot[0][2][k]=0;
rot[1][0][k]=-sin(gammab[k]*pi/180);
rot[1][1][k]=cos(gammab[k]*pi/180);
rot[1][2][k]=0;
rot[2][0][k]=0;
rot[2][1][k]=0;
rot[2][2][k]=1;
}


int m=0;
for(m=0;m<6;m++)
{
 for (i = 0; i < 3; i++)
    {
       
           L[i][m] = 0;
            for (k = 0; k < 3; k++)
            {
               L[i][m] += rot[i][k][m] * bA[k][m];
            }
        
    }
}


float r[3]={rb,0,0};
for(i=0;i<6;i++)
{
  for(j=0;j<3;j++)
  {
    L[j][i]-=r[j];
  }
}

 float len [6];
 float temp=0;
for(i=0;i<6;i++)
{
  temp=0.0;
  for(j=0;j<3;j++)
  {
   temp=temp+(bA[j][i]*bA[j][i]);
  }
  len[i]=sqrt(temp);
}
// Serial.println(len[1],5); 


float servoangt[6][2];//Temporary servoangles before correcting for offset
float offset1[6]={10.2575,18.9046,10.2575,18.9046,10.2575,18.9046};// These are the offsets to be corrected for
float offset2[6]={-180,-180,-180,-180,-180,-180};
float a,b,c; 
float s=0.045; //This is the servo arm length 
for (i=0;i<6;i++)
{
    a=2*s*L[1][i];
    b=2*s*L[2][i];
    c=leg*leg-(len[i]*len[i]);  
      
     servoangt[i][0]=(acos(-c/(sqrt(a*a+b*b)))+atan2(b,a))*180/pi+offset2[i]-10.2575;
 
    servoangt[i][1]=(-acos(-c/(sqrt(a*a+b*b)))+atan2(b,a))*(180/pi)+offset1[i];
}


float servoang[6]={0,0,0,0,0,0};// Initialising the servo angles
float offset[6]={-8.1635,8.1635,-8.1635,8.1635,-8.1635,8.1635};
for (i=0;i<6;i++)
{
    if((i+1)%2==0)
        
    servoang[i]=servoangt[i][0]+offset[i];
    
    else
      servoang[i]=servoangt[i][1]+offset[i];
    
    
}


s1.write(60);
s2.write(60);
s3.write(60);
s4.write(60);
s5.write(60);
s6.write(60);
unsigned long time;
time=millis(); 
   float freq=0.5;

// Giving inputs to all the 6 servos

while(millis()<10000)
{
 
    
    
    s1.write(60+(servoang[0]*sin((2*pi*freq*millis()*0.001))));
    s2.write(60+(servoang[1]*sin((2*pi*freq*millis()*0.001))));
    s3.write(60+(servoang[2]*sin((2*pi*freq*millis()*0.001))));
    s4.write(60+(servoang[3]*sin((2*pi*freq*millis()*0.001))));
    s5.write(60+(servoang[4]*sin((2*pi*freq*millis()*0.001))));
    s6.write(60+(servoang[5]*sin((2*pi*freq*millis()*0.001))));
      
  
}
//All the servos will go back to the home position after the motion

s1.write(60);
s2.write(60);
s3.write(60);
s4.write(60);
s5.write(60);
s6.write(60);

while(1)
{
  ;
}
  
}




