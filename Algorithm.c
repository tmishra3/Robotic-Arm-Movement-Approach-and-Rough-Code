#include <math.h>
#include <Servo.h>

Servo baseServo; //Motor that pivots entire arm. [2]
Servo servo1; //First joint motor. [3]
Servo servo2; //Second joint motor. [7]
Servo servo3; //Third joint/hand motor. [8]
Servo servo4; //Hand swivel motor. [9]
Servo servo5; //Claw open/close motor. [10]

double fin_ang1 = 0.0; //Stores value of actual angle of motor 1 from algorithm calculation.
double fin_ang2 = 0.0; //Stores value of actual angle of motor 2 from algoirthm calculation.
double fin_ang3 = 0.0; //Stores value of actual angle of motor 3 from algoirthm calculation.
double fin_ang2_math = 0.0; //Stores value of angle of motor 2 from algoirthm calculation (This is relative to vertical NOT actual motor angle).
double fin_ang3_math = 0.0;//Stores value of angle of motor 3 from algoirthm calculation (This is relative to vertical NOT actual motor angle).

int parallel_algo = 0; //When moveArm function is called, 0 represents parallel calculation, 1 represents values pulled from algorithm calculation.

//Stores the feedback values for each motor.
int feedback_1;
int feedback_2;
int feedback_3;
int feedback_4;
int feedback_5;
int feedback_0; //Base motor.

//Feedback pin assignments.
int feedback_pin1 = 1;
int feedback_pin2 = 2;
int feedback_pin3 = 3;
int feedback_pin4 = 4;
int feedback_pin5 = 5;
int feedback_pin0 = 0; //Base motor.

//Feedback pin angles calculated from best fit linear regressions.
double feedback_angle1;
double feedback_angle2;
double feedback_angle3;
double feedback_angle4;
double feedback_angle5;
double feedback_angle0;

//Stores lengths of each arm (in cm).
double D1 = 5.0;
double D2 = 6.0;
double D3 = 19.0;

//Stores the values of the angles.
double ang1 = 90.0;//Stores the actual value of the angle. This goes to the motor.
//This also stores the vertical value as it's independent of other joints.
double ang2 = 0.0;//Stores the value of angle 2 according to the diagram.
double ang2r = 0.0;//Stores the real value of angle 2. This goes to the motor.
double ang3 = 0.0;//Stores the value of angle 3 according to the diagram.
double ang3r = 0.0;//Stores the real value of angl3 3. This goes to the motor.
double angBase = 0.0;//Stores the angle of the base.
double angHand = 5.0; //Default value (5 degrees).
double angWrist = 0.0; //Angle of the wrist.
double oldAng1 = 90; //Stores old value of angle1.
double oldang2r = ang2r; //Stores old value of actual angle2.
double oldang3r = ang3r; //Stores old value of actual angle3.
double oldbaseAngle = 0.0; //Stores old value of the actual base angle.

//Xdn and Ydn are the coordinates of the location
//of the base of the arm. Dx and Dy store the location of point
//D which is located at (0,0) and represents the endpoint of
//the arm where it terminates at the origin. Ax and Ay store the
//new location of the base of the arm.
double Xdn = 0.0;
double Ydn = 0.0;

//Stores the CURRENT location of the base of the arm.
double Xd = 0.0;
double Yd = 0.0;

//Stores the value of the new coordinates to move to.
double newX, newY, newZ;
double Axn, Ayn;
double loopcode = 2; //Since I2c hasn't yet been implemented, if loopcode is 2, the loop() section of the code
//will function only when loopcode is 2.

double theta2;
double theta1;
double theta3;

void setup()
{

  Serial.begin(9600); //Allowing there to be serial monitor output.

  /***Pin Assignments***/
  baseServo.attach(2); //Base Servo
  servo1.attach(3); //motor 1
  servo2.attach(7); //motor 2
  servo3.attach(8); //motor 3
  servo4.attach(9); //swivel
  servo5.attach(10); //claw

  OpenCloseHand(0); //if 0, goes to 5 degrees. If 1, goes to 60 degrees.

  do
  {
    moveToDefault();
  }while (compareAngles(40,75,10,90)==false);
  
  Serial.print("\n**Asking for input**\n");

  calculateCurrentBaseLocation();

  //do
  //{
    Serial.print("In Do loop.\n"); 
    enterCoordinates();

 // } while (inRange() == false);

  calculateXYProjection();

  algorithm(Xdn, Ydn);

  parallel_algo = 1;
  moveArm();
}

void loop() //Runs over and over. This is why I have loopcode
//set to 2 initially and then set to a different value so it won't run over and over.
{
  /*
    while (loopcode == 2)
    {

      do
      {
        enterCoordinates();

      } while (inRange() == false);

        calculateXYProjection();

      algorithm(Xdn, Ydn);

      parallel_algo = 1;
      moveArm();

      loopcode = 1;
    }
  */
}

bool inRange()
{
  double d1x, d1y;
  d1x = D1 * sin(toR(theta1)) + Xd;
  d1y = D1 * cos(toR(theta1)) + Yd;

  double a, b, c;

  a = (pow(d1y, 2) / pow(d1x, 2)) + 1;
  b = ((pow(d1x, 2) + pow(d1y, 2) - pow(D2, 2) + pow(D3, 2)) * -d1y) / pow(d1x, 2);
  c = pow((pow(d1x, 2) + pow(d1y, 2) - pow(D2, 2) + pow(D3, 2)) / (2 * D1), 2) - pow(D3, 2);

  double b24ac = pow(b, 2) - (4 * a * c);

  if ((b24ac >= 0) && ((Yd - 11) < newY))
  {
    return true;
  }
  else
  {
    false;
  }

  //Still need to check to prevent arm from hitting the floor.
}
void enterCoordinates()
{
  Serial.print("Enter Coordinates for new location:\n");
  //Serial.print("X: ");
 // newX = Serial.read();

  Serial.print("Y: ");
  //newY = Serial.read();

  Serial.print("Z: ");
  //newZ = Serial.read();
}

void calculateXYProjection()
{
  Axn = sqrt(pow(Xdn - newX, 2) + pow(newZ, 2)) + Xdn;
  Ayn = newY;//Converting the new coordinate to XY plane.

  Xdn = Xd - Axn; //Axn is the "new" X location of the base.
  Ydn = Yd - Ayn; //Ayn is the "new" Y location of the base.
}

//This function simply returns the angles relative to angle 0 of a given X and Y coordinate.
double returnAngle(double X, double Y)
{
  double Ang_deg = toD(atan(Y / X)) + (X < 0) * (180);
  return Ang_deg;
}

//Math functions don't take degrees so if value returned in radians, must be converted to degrees.
double toD(double myRad)
{
  return myRad * (180.0 / PI);
}

//Math functions take values in radians so all degree values must be converted to radians.
double toR(double myDeg)
{
  return myDeg * (PI / 180.0);
}

void moveToDefault()
{
  ang1 = 40;
  ang2r = 75;
  ang3r = 10;
  angBase = 90;

  parallel_algo = 2;

  moveArm();
}

bool compareAngles(double myAng1, double myAng2, double myAng3, double base_ang)
{
  feedback();
  
  double error_range = 3;

  if ((abs(myAng2 - feedback_angle2) <= error_range) && (abs(myAng3 - feedback_angle3) <= error_range) && (abs(base_ang - feedback_angle0) <= error_range))
  {
    Serial.print("At Default Position\n\n");
    return true;
  }
  else
  {
    return false;
  }
}

void calculateCurrentBaseLocation()
{
  feedback();

  theta2 = feedback_angle2;
  theta1 = feedback_angle1;
  theta3 = feedback_angle3;
  
  Serial.print("Theta 1: ");
  Serial.print(theta1);
  Serial.print("\n");
  Serial.print("Theta 2: ");
  Serial.print(theta2);
  Serial.print("\n");
  Serial.print("Theta 3: ");  
  Serial.print(theta3);
  Serial.print("\n");
  
  Yd = D3 * sin(toR(90 + theta1 - theta2 - theta3)) - D1 * cos(toR(theta1)) - D2 * sin(toR(theta2 - theta1));
  Xd = -D1 * sin(toR(theta1)) - D2 * cos(toR(theta2 - theta1)) - D3 * cos(toR(90 + theta1 - theta2 - theta3));
  
  Serial.print("Xd: ");
  Serial.print(Xd);
  Serial.print("\n");
  Serial.print("Yd: ");
  Serial.print(Yd);
  Serial.print("\n\n");
}

//If parallel, calculates the location of the base.
void calculateLocation()
{
  ang2r = ang2 + ang1 - 90;
  ang3 = 180 - ang1;
  ang3r = ang3 + ang1 - ang2r;

  Xdn = -1 * ((D1 * sin(toR(ang1))) + (D2 * sin(toR(ang2))) + (D3 * sin(toR(ang3))));
  Ydn = -1 * ((D1 * cos(toR(ang1))) + (D2 * cos(toR(ang2))) - (D3 * cos(toR(ang3))));
}

//If algorithm is used, calculates the location of the base with those values.
void calculateLocationOfAlgorithm()
{
  ang2r = fin_ang2_math + fin_ang1 - 90;
  ang3 = fin_ang3_math;
  ang3r = fin_ang3;
  ang1 = fin_ang1;

  Xdn = -1 * ((D1 * sin(toR(ang1))) + (D2 * sin(toR(ang2))) + (D3 * sin(toR(ang3))));
  Ydn = -1 * ((D1 * cos(toR(ang1))) + (D2 * cos(toR(ang2))) - (D3 * cos(toR(ang3))));

}

//Calculates distance between 2 points.
double calculateDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

//This function moves the arm.
void moveArm()
{
  feedback(); //Calculating feedback.

  //From feedback values, set feedback angles to old values.
  oldang2r = feedback_angle2;
  oldAng1 = feedback_angle1;
  oldang3r = feedback_angle3;
  oldbaseAngle = feedback_angle0;

  //If parallel, set values of angles TO move to.
  if (parallel_algo == 0) //Parallel angles.
  {
    ang2r = ang2 + ang1 - 90; //Real value of angle of joint 2.
    ang3 = 180 - ang1;
    ang3r = ang3 + ang1 - ang2r; //Real value of angle of joint 3.
  }
  //If non parallel values and algorithm used, set values of angles TO move to.
  if (parallel_algo == 1) //Non parallel angles. Values pulled from algorithm.
  {
    ang2r = fin_ang2;
    ang1 = fin_ang1;
    ang3r = fin_ang3;
  }
  //If non parallel values and algorithm not used, values are already set from setup or loop function.
  if (parallel_algo == 2) //Non parallel angles. Values pulled directly from manually setting
    //the values of the angles from the setup() or loop() functions.
  {
  }

  //Set incremental movement of servo. Still need to make this more efficient.
  double servo1Dir = abs(oldAng1 - ang1) / 100;
  double servo2Dir = abs(oldang2r - ang2r) / 100;
  double servo3Dir = abs(oldang3r - ang3r) / 100;
  double baseDir = abs(oldbaseAngle - angBase) / 100;

  //If movement of angles is in negative direction, set them to be -1 x their value.
  if (ang1 < oldAng1)
  {
    servo1Dir *= -1;
  }
  if (ang2r < oldang2r)
  {
    servo2Dir *= -1;
  }
  if (ang3r < oldang3r)
  {
    servo3Dir *= -1;
  }
  if (angBase < oldbaseAngle)
  {
    baseDir *= -1;
  }
  /*
    Serial.print("**Moving servo 1**\n");
    Serial.print("Moving from ");
    Serial.print(oldAng1);
    Serial.print(" to ");
    Serial.print(ang1);
    Serial.print("\n");

    Serial.print("**Moving servo 2**\n");
    Serial.print("Moving from ");
    Serial.print(oldang2r);
    Serial.print(" to ");
    Serial.print(ang2r);
    Serial.print("\n");

    Serial.print("**Moving servo 3**\n");
    Serial.print("Moving from ");
    Serial.print(oldang3r);
    Serial.print(" to ");
    Serial.print(ang3r);
    Serial.print("\n\n");
  */
  double i = oldAng1, j = oldang2r, k = oldang3r, l = oldbaseAngle;

  //Begin process of moving by checking conditions of if any of the motors are not within .001 of their desired end point, it will keep moving until
  //they all are. May be possible point of contention for the code to where it "spazzes" because it may fall outside of the range and continue moving.

  while ((abs(i - ang1) > .001) || (abs(j - ang2r) > .001) || (abs(k - ang3r) > .001) || (abs(l - angBase) > .001))
  {
    if (abs(i - ang1) > .001)
    {
      servo1.write(i);
      i += servo1Dir;
    }

    if (abs(j - ang2r) > .001)
    {
      servo2.write(j);
      j += servo2Dir;
    }

    if (abs(k - ang3r) > .001)
    {
      servo3.write(k);
      k += servo3Dir;
    }

    if (abs(l - angBase) > .001)
    {
      baseServo.write(l);
      l += baseDir;
    }
    delay(30);
  }

  feedback(); //Diagnostic purposes.
}

void OpenCloseHand(int i)
{
  if (i == 0)
  {
    servo5.write(5);
  }
  if (i == 1)
  {
    servo5.write(60);
  }
}

void rotateWrist(double myAng)
{
  feedback();

  double inc = 10;
  double delay_value = 100;
  double dirMove = abs(myAng - feedback_angle4) / 20;
  angWrist = feedback_angle4;

  if (dirMove <= 1)
  {
    delay_value /= .5;
  }

  Serial.print("**Wrist Rotation**\n");
  Serial.print("Rotating to angle ");
  Serial.print(myAng);
  Serial.print(" from angle ");
  Serial.print(feedback_angle4);
  Serial.print("\nMoving by ");
  Serial.print(dirMove);
  Serial.println("");

  if ((myAng <= 180) && (myAng >= 0))
  {

    if (myAng < angWrist)
    {
      for (int i = angWrist; i > myAng; i -= dirMove)
      {
        Serial.print(i);
        Serial.print(" ");
        servo4.write(i);
        delay(delay_value);
      }
    }
    if (myAng > angWrist)
    {
      for (int i = angWrist; i < myAng; i += dirMove)
      {
        Serial.print(i);
        servo4.write(i);
        delay(delay_value);
      }
    }

    Serial.print("**Done Moving**\n\n");
    feedback();
    angWrist = feedback_angle4;

  }
  else
  {
    Serial.println("Error. Not in range.");
  }
}

void feedback()
{
  double fb0_sample = 0.0;
  double fb1_sample = 0.0;
  double fb2_sample = 0.0;
  double fb3_sample = 0.0;
  double fb4_sample = 0.0;
  double fb5_sample = 0.0;  
  
  for (int i = 0; i <= 10000; i++)
  {
    if (i>100)
    {
      fb0_sample += analogRead(feedback_pin0);
      fb1_sample += analogRead(feedback_pin1);
      fb2_sample += analogRead(feedback_pin2);
      fb3_sample += analogRead(feedback_pin3);
      fb4_sample += analogRead(feedback_pin4);
      fb5_sample += analogRead(feedback_pin5);
    }
  }
  feedback_1 = fb1_sample/9900;
  feedback_2 = fb2_sample/9900;
  feedback_3 = fb3_sample/9900;
  feedback_4 = fb4_sample/9900;
  feedback_5 = fb5_sample/9900;
  feedback_0 = fb0_sample/9900;

  feedback_angle1 = (feedback_1 - 115.295) / 2.29407;
  feedback_angle2 = (feedback_2 - 64.8643) / 1.50566;
  feedback_angle3 = (feedback_3 - 71.66) / 1.616335;
  feedback_angle4 = (feedback_4 - 69.04884) / 1.58367;
  feedback_angle5 = (feedback_5 - 66.526) / 1.5379157;
  feedback_angle0 = (feedback_0 - 113.2628) / 2.25834;

/*
  Serial.print("Feedback Angle 0: ");
  Serial.print(feedback_angle0);
  Serial.print("\n");
 
  Serial.print("Feedback Angle 1: ");
  Serial.print(feedback_angle1);
  Serial.print("\n");

  Serial.print("Feedback Angle 2: ");
  Serial.print(feedback_angle2);
  Serial.print("\n");

  Serial.print("Feedback Angle 3: ");
  Serial.print(feedback_angle3);
  Serial.print("\n");

  Serial.print("Feedback Angle 4: ");
  Serial.print(feedback_angle4);
  Serial.print("\n");

  Serial.print("Feedback Angle 5: ");
  Serial.print(feedback_angle5);
  Serial.print("\n");
  Serial.print("\n");
  */
}

void algorithm(double Xd, double Yd)
{
  double d1x, d1y, d3x, d3y;
  double angD1D2, angD2D3, distBetweenD1D3;
  double m1, m2, m3;

  double ang1slide, ang3slide;

  double distance[121];
  double d1d2_ang[121];
  double d2d3_ang[121];
  double ang1array[121];
  double ang3array[121];
  int indexArray[121];

  double final_ang1 = 0.0;
  double final_ang2 = 0.0;
  double final_ang3 = 0.0;
  double final_ang2_math = 0.0;
  double final_ang3_math = 0.0;

  double angOfSecondArm = 0.0;

  int counter = 0;
  int solutionFound = 0;

  //The following code will scan for angles

  for (ang1slide = 6.0; (ang1slide <= 96.0); ang1slide += 9.0) //Scanning angles for theta 1.
  {
    for (ang3slide = 10.0; (ang3slide <= 170.0); ang3slide += 16.0) //Scanning angles for theta 3.
    {
      d1x = D1 * sin(toR(ang1slide)) + Xd;
      d1y = D1 * cos(toR(ang1slide)) + Yd;
      d3x = -D3 * sin(toR(ang3slide));
      d3y = D3 * cos(toR(ang3slide));

      m1 = (d1y - Yd) / (d1x - Xd);
      m2 = (d3y - d1y) / (d3x - d1x);

      angD1D2 = 90 + toD(atan(abs((m1 - m2) / (1 + m1 * m2))));

      m3 = (d3y - d1y) / (d3x - d1x);

      angD2D3 = toD(atan(abs(m3))) + ang3slide;

      distBetweenD1D3 = (sqrt(pow(d1x - d3x, 2) + pow(d1y - d3y, 2))) - D2;

      distance[counter] = distBetweenD1D3;
      d1d2_ang[counter] = angD1D2;
      d2d3_ang[counter] = angD2D3;
      ang1array[counter] = ang1slide;
      ang3array[counter] = ang3slide;
      indexArray[counter] = counter;

      counter++;
    }
  }


  //Sorting Arrays in ascending order using Bubble Sort.

  int v , b;
  double dist_temp;
  double d1d2_temp;
  double d2d3_temp;
  double ang1_temp;
  double ang3_temp;
  double index_temp;


  for (b = 0; (b < 120); b++)
  {
    for (v = 0; (v < (120 - b)); v++)
    {
      if (distance[v] > distance[v + 1])
      {
        dist_temp =  distance[v];
        distance[v] = distance[v + 1];
        distance[v + 1] = dist_temp;

        d1d2_temp = d1d2_ang[v];
        d1d2_ang[v] = d1d2_ang[v + 1];
        d1d2_ang[v + 1] = d1d2_temp;

        d2d3_temp = d2d3_ang[v];
        d2d3_ang[v] = d2d3_ang[v + 1];
        d2d3_ang[v + 1] = d2d3_temp;

        ang1_temp = ang1array[v];
        ang1array[v] = ang1array[v + 1];
        ang1array[v + 1] = ang1_temp;

        ang3_temp = ang3array[v];
        ang3array[v] = ang3array[v + 1];
        ang3array[v + 1] = ang3_temp;

        index_temp = indexArray[v];
        indexArray[v] = indexArray[v + 1];
        indexArray[v + 1] = index_temp;
      }
    }
  }

  int index = 0;

  final_ang2 = 0.0;
  final_ang3 = 0.0;
  final_ang2_math = 0.0;
  final_ang3_math = 0.0;

  double isXd, isYd;

  int k;

  for (index = 0; ((index < 121) && (solutionFound == 0)); index++)
  {

    //cout << "Scanning around: " << endl << "Angle 1: " << ang1array[index] << endl << "Angle 2: " << d1d2_ang[index]
    //<< endl << "Angle 3: " << d2d3_ang[index] << endl << endl;

    for (double k = (ang1array[index] - 20); k <= (ang1array[index] + 20); k += .1)
    {
      for (double j = (ang3array[index] - 20); ((j <= (ang3array[index] + 20)) && (solutionFound == 0)); j += .01)
      {
        d1x = D1 * sin(toR(k)) + Xd;
        d1y = D1 * cos(toR(k)) + Yd;
        d3x = -D3 * sin(toR(j));
        d3y = D3 * cos(toR(j));

        m1 = (d1y - Yd) / (d1x - Xd);
        m2 = (d3y - d1y) / (d3x - d1x);

        angD1D2 = 90 + toD(atan(abs((m1 - m2) / (1 + m1 * m2))));

        m3 = (d3y - d1y) / (d3x - d1x);

        angD2D3 = toD(atan(abs(m3))) + j;

        distBetweenD1D3 = (sqrt(pow(d1x - d3x, 2) + pow(d1y - d3y, 2))) - D2;

        if ((abs(distBetweenD1D3) < .1) && (angD1D2 >= 30.0) && (angD1D2 <= 160.0) && (angD2D3 >= 10.0) && (angD2D3 <= 170.0) && (k >= 6.0) && (k <= 96.0))
        {
          final_ang1 = k;
          final_ang3_math = toD(atan(d3y / d3x)) + 90;
          final_ang2_math = 90 - toD(atan((d3y - d1y) / (d3x - d1x)));

          isXd = -1 * ((D1 * sin(toR(final_ang1))) + (D2 * sin(toR(final_ang2_math))) + (D3 * sin(toR(final_ang3_math))));
          isYd = -1 * ((D1 * cos(toR(final_ang1))) + (D2 * cos(toR(final_ang2_math))) - (D3 * cos(toR(final_ang3_math))));

          if ((abs(isXd - Xd) < .3) && (abs(isYd - Yd) < .3))
          {
            final_ang1 = k;
            final_ang2 = angD1D2;
            final_ang3 = angD2D3;
            final_ang3_math = toD(atan(d3y / d3x)) + 90;
            final_ang2_math = 90 - toD(atan((d3y - d1y) / (d3x - d1x)));
            //cout << "Solution Found." << endl;
            solutionFound = 1;
            break;
          }
        }

      }
    }
  }

  if (solutionFound == 1)
  {

    //cout << "***Calculation Complete***" << endl << "Theta 1: " << final_ang1;
    //cout << endl << "Theta 2: " << final_ang2 << endl;
    //cout << "Theta 3: " << final_ang3 << endl;

    //cout << endl << "Approximate Location of Xd: " << -1 * ((D1 * sin(toR(final_ang1))) + (D2 * sin(toR(final_ang2_math))) + (D3 * sin(toR(final_ang3_math))));
    //cout << endl << "Approximate Location of Yd: " << -1 * ((D1 * cos(toR(final_ang1))) + (D2 * cos(toR(final_ang2_math))) - (D3 * cos(toR(final_ang3_math)))) << endl << endl;
  }
  else
  {
    //cout << "No Solution Found around the location." << endl;
  }
}
