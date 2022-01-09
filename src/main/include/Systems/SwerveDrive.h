// Swerve Drive routine ver 1.3
// 02/05/2020

#include<math.h>
#include<algorithm>

// Swerve Drive Geometry
const static float Length = 25;     // wheel base
const static float Width = 20;      // track width
const static float R = sqrt(Length*Length + Width*Width);

// Max Motor RPM
const static float maxRPM = 60.0;

// Conversion factor radians to degrees - 180/pi
const static float rad2deg = 57.2957795;
const static float deg2rad = 0.01745329251;

float FaceAngle = 0;
float wheelSpeed[4] = {0};
float wheelPreviousAngle[4] = {0};
float wheelAngle[4] = {0};
float throttle[4] = {0};
float steer[4] = {0};

float gainP = 0.01;
float gainI = 0.0000;
float gainD = 0.00000;

float bodyError = 0;
float bodyPreviousError = 0;
float bodyIntError = 0;
float bodyDiffError = 0;

void OptimizeSteerTurning3(void)
{
    float targetAngle,targetStraightAngle, currentAngle, A, B, b0, x0, x1;
    int t = 0;
    
    while (t < 4)
    {
        currentAngle = wheelPreviousAngle[t];
        targetAngle = wheelAngle[t];

        x0 = abs(currentAngle);
        x1 = abs(targetAngle);

        // Calculate Straight/Opposite Target Angle
        if(targetAngle > 0)
        {
            targetStraightAngle = targetAngle - 180;
        }
        else
        {
            targetStraightAngle = 180 + targetAngle;
        }

        // Calculate A & B (refer to Excel spreadsheet)
        A = abs(currentAngle - targetAngle);
        b0 = abs(currentAngle - targetStraightAngle);
        if (b0 > 180)
        {
            B = 360 - b0;
        }
        else
        {
            B = b0;
        }

        // Pick which angle position
        if (A <= B)
        {            
            wheelAngle[t] = targetAngle;
        }
        else
        {
            wheelAngle[t] = targetStraightAngle;   
            throttle[t] = -throttle[t];
        }      

        t++;
    }   
}

void ZeroSwerveDrive2(void)
{
    int loopcount = 0;
	int t;
    int a, b, c, d, e;
    float rotationSpeed[]={0.4,0.4,0.3,0.4};
	
	navxGyro->ZeroYaw(); 

    while (loopcount < 1)
    {
		
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;

        steerMotor0.Set(ControlMode::PercentOutput, rotationSpeed[0]);      // front right
		steerMotor1.Set(ControlMode::PercentOutput, rotationSpeed[1]);      // front left
		steerMotor2.Set(ControlMode::PercentOutput, rotationSpeed[2]);      // rear left
		steerMotor3.Set(ControlMode::PercentOutput, rotationSpeed[3]);      // rear right
				
		while (e < 4)
		{
			// frc::SmartDashboard::PutBoolean("Front Right IRSensor", IRSensor2->Get());
            // frc::SmartDashboard::PutBoolean("Front Left IRSensor", IRSensor3->Get());
            // frc::SmartDashboard::PutBoolean("Rear Left IRSensor", IRSensor0->Get());
            // frc::SmartDashboard::PutBoolean("Rear Right IRSensor", IRSensor1->Get());
            
            if(IRSensor0->Get() == IRSensorClosed)
			{
				steerMotor0.Set(ControlMode::PercentOutput, 0);					
                if (a == 0)
                    {a = 1;rotationSpeed[0] = -rotationSpeed[0]*0.8;}			
			}
			
			if(IRSensor1->Get() == IRSensorClosed)
			{
				steerMotor1.Set(ControlMode::PercentOutput, 0);			
                if (b == 0)
                    {b = 1;rotationSpeed[1] = -rotationSpeed[1]*0.8;}				
			}
			
			if(IRSensor2->Get() == IRSensorClosed)
			{
				steerMotor2.Set(ControlMode::PercentOutput, 0);				
                if (c == 0)
                    {c = 1;rotationSpeed[2] = -rotationSpeed[2]*0.8;}				
			}
			
			
			if(IRSensor3->Get() == IRSensorClosed)
			{
				steerMotor3.Set(ControlMode::PercentOutput, 0);				
                if (d == 0)
                    {d = 1;rotationSpeed[3] = -rotationSpeed[3]*0.8;}				
			}	

            e = a + b + c + d;	
		}       

        loopcount++;
    } 
       
    FaceAngle = navxGyro->GetYaw()*deg2rad;
    SetSteerMotorInitialPosition();   
}

void ZeroWheelsOnly(void)
{
    int loopcount = 0;
	int t;
    int a, b, c, d, e;
    float rotationSpeed[]={0.4,0.4,0.3,0.4};

    while (loopcount < 3)
    {
		
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;

        steerMotor0.Set(ControlMode::PercentOutput, rotationSpeed[0]);      // front right
		steerMotor1.Set(ControlMode::PercentOutput, rotationSpeed[1]);      // front left
		steerMotor2.Set(ControlMode::PercentOutput, rotationSpeed[2]);      // rear left
		steerMotor3.Set(ControlMode::PercentOutput, rotationSpeed[3]);      // rear right
				
		while (e < 4)
		{
            
            if(IRSensor0->Get() == IRSensorClosed)
			{
				steerMotor0.Set(ControlMode::PercentOutput, 0);					
                if (a == 0)
                    {a = 1;rotationSpeed[0] = -rotationSpeed[0]*0.8;}			
			}
			
			if(IRSensor1->Get() == IRSensorClosed)
			{
				steerMotor1.Set(ControlMode::PercentOutput, 0);			
                if (b == 0)
                    {b = 1;rotationSpeed[1] = -rotationSpeed[1]*0.8;}				
			}
			
			if(IRSensor2->Get() == IRSensorClosed)
			{
				steerMotor2.Set(ControlMode::PercentOutput, 0);				
                if (c == 0)
                    {c = 1;rotationSpeed[2] = -rotationSpeed[2]*0.8;}				
			}
			
			
			if(IRSensor3->Get() == IRSensorClosed)
			{
				steerMotor3.Set(ControlMode::PercentOutput, 0);				
                if (d == 0)
                    {d = 1;rotationSpeed[3] = -rotationSpeed[3]*0.8;}				
			}	

            e = a + b + c + d;	
		}       

        loopcount++;
    }
    
    SetSteerMotorInitialPosition();   
}

void SwerveDriveMe(float Forward, float Strafe, float Rotate, float BodyAngle, bool isSetFaceAngle, float setFaceAngle)
{
    int t;
    float tempval, errorAngle; 

    // Record previous steer wheel angles  
    t = 0;
    while (t < 4)
    {
        wheelPreviousAngle[t] = wheelAngle[t];
        t++;
    }

    float magnitude = sqrt(Forward*Forward + Strafe*Strafe);

    // frc::SmartDashboard::PutNumber("Joystick X", Strafe);
    // frc::SmartDashboard::PutNumber("Joystick Y", Forward);
    // frc::SmartDashboard::PutNumber("Rotation", Rotate);
    // frc::SmartDashboard::PutNumber("Body Angle", BodyAngle);
    // frc::SmartDashboard::PutNumber("Translation Magnitude", magnitude);
    // frc::SmartDashboard::PutNumber("Face Angle", FaceAngle);
    
    // Check if translation and rotation is above the threshold to start moving
    if (magnitude < 0.05)
    {
        Forward = 0;
        Strafe = 0;
    }

    if (abs(Rotate) < 0.05)     // if very small input from rotation axis, either during pure translation or auton
    {
        if (magnitude < 0.05)   // do not rotate if joystick input is also very small i.e ~zero inputs on both rotation and translation 
        {
            Rotate = 0;         // not moving at all
        }
        else    // this is for when ~zero input from rotation, but translation magnitude is > 5%
        {
            if (isSetFaceAngle)
            {
                FaceAngle = setFaceAngle;
            }
            
            if (abs(BodyAngle - FaceAngle) > 180)
            {
               if (BodyAngle < 0)
               {
                   BodyAngle = 360 + BodyAngle;
               }
               else
               {
                   BodyAngle = BodyAngle - 360;
               }              
            }
            
            // Rotate = -(BodyAngle - FaceAngle)*0.01;
			//Rotate = (BodyAngle - FaceAngle)*0.009;	// this is the one last used before implementing PID
 			bodyPreviousError = bodyError;
			bodyError = BodyAngle - FaceAngle;
			bodyIntError = bodyIntError + bodyError;
			bodyDiffError = bodyError - bodyPreviousError;
			
            Rotate = bodyError*gainP + bodyIntError*gainI + bodyDiffError*gainD;
        }        
    }
    else
    {
        FaceAngle = BodyAngle;
    }      

    BodyAngle = BodyAngle*deg2rad;
    float Vx = Forward*cos(BodyAngle) + Strafe*sin(BodyAngle);
    float Vy = -Forward*sin(BodyAngle) + Strafe*cos(BodyAngle);    

    float Rx = -Rotate*Width/R;
    float Ry = -Rotate*Length/R;    

    float V0x = Vx + Rx;
    float V1x = Vx - Rx;
    float V2x = Vx - Rx;
    float V3x = Vx + Rx;

    float V0y = -Vy + Ry;
    float V1y = -Vy + Ry;
    float V2y = -Vy - Ry;
    float V3y = -Vy - Ry;

    // Calculate Wheel Speeds
    wheelSpeed[0] = sqrt(V0x*V0x + V0y*V0y);      // front right corner
    wheelSpeed[1] = sqrt(V1x*V1x + V1y*V1y);      // front left corner
    wheelSpeed[2] = sqrt(V2x*V2x + V2y*V2y);      // rear left corner
    wheelSpeed[3] = sqrt(V3x*V3x + V3y*V3y);      // rear right corner

    // Normalize Wheel Speeds if maximum wheelSpeed > 1
    tempval = 0;
    for(t=0;t<4;t++)
    {
        if (wheelSpeed[t] > tempval)
        {
            tempval = wheelSpeed[t];
        }
    }

    if (tempval > 1)
    {
        for (t=0;t<4;t++)
        {
            wheelSpeed[t] = wheelSpeed[t]/tempval;
        }
    }

    // Convert to throttle RPM
    for(t=0;t<4;t++)
    {
        throttle[t] = wheelSpeed[t]*maxRPM;
    }

    if (magnitude >= 0.15 ||abs(Rotate) >= 0.15)  // do not adjust the steering wheel angles if we are letting go of the joystick 
    {
        // Calculate Wheel Angles
        wheelAngle[0] = -atan2(V0y, V0x)*rad2deg;   // front right corner
        wheelAngle[1] = -atan2(V1y, V1x)*rad2deg;   // front left corner
        wheelAngle[2] = -atan2(V2y, V2x)*rad2deg;   // rear left corner
        wheelAngle[3] = -atan2(V3y, V3x)*rad2deg;   // rear right corner 
    }

    OptimizeSteerTurning3();

    // frc::SmartDashboard::PutNumber("Front Right Wheel Angle", wheelAngle[0]);
    // frc::SmartDashboard::PutNumber("Front Left Wheel Angle", wheelAngle[1]);
    // frc::SmartDashboard::PutNumber("Rear Left Wheel Angle", wheelAngle[2]);
    // frc::SmartDashboard::PutNumber("Rear Right Wheel Angle", wheelAngle[3]);

    // frc::SmartDashboard::PutNumber("Front Right Wheel RPM", throttle[0]);
    // frc::SmartDashboard::PutNumber("Front Left Wheel RPM", throttle[1]);
    // frc::SmartDashboard::PutNumber("Rear Left Wheel RPM", throttle[2]);
    // frc::SmartDashboard::PutNumber("Rear Right Wheel RPM", throttle[3]);    

}
