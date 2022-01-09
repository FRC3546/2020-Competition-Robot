void AutonMove(float X, float Y, float W, bool isSetFaceAngle, float faceAngle, int loopCount)   // Just move for a specified amount of loopcounts at a specific throttle speed
{
    float A, waittime;
    int t;
    int y;
    
    y = 0;
    
    // gainP = frc::SmartDashboard::GetNumber("Auton Gain P", 0.009); 
    // gainI = frc::SmartDashboard::GetNumber("Auton Gain I", 0); 
    // gainD = frc::SmartDashboard::GetNumber("Auton Gain D", 0); 
    waittime = frc::SmartDashboard::GetNumber("Auton Loop Wait Time", 0.002);

    while(y <= loopCount)
    {
        A = navxGyro->GetYaw();   
   
        SwerveDriveMe(-Y*0.6,-X*0.6, W*0.4, A, isSetFaceAngle, faceAngle);
        
        if (abs(throttle[0]) > 0) // Front Right Corner
        {
            throttleMotor0PIDController.SetReference(throttle[0]*maxRPM*throttleMotorDirection[0], rev::ControlType::kVelocity); 
        }
        else
        {
            throttleMotor0PIDController.SetReference(0, rev::ControlType::kVoltage);
        }
        
        if (abs(throttle[1]) > 0) // Front Left Corner
        {
            throttleMotor1PIDController.SetReference(throttle[1]*maxRPM*throttleMotorDirection[1], rev::ControlType::kVelocity);
        }
        else
        {
            throttleMotor1PIDController.SetReference(0, rev::ControlType::kVoltage);
        }
        
        if (abs(throttle[2]) > 0) // Rear Left Corner
        {
            throttleMotor2PIDController.SetReference(throttle[2]*maxRPM*throttleMotorDirection[2], rev::ControlType::kVelocity);
        }
        else
        {
            throttleMotor2PIDController.SetReference(0, rev::ControlType::kVoltage);
        }
        
        if (abs(throttle[3]) > 0) // Rear Left Corner
        {
            throttleMotor3PIDController.SetReference(throttle[3]*maxRPM*throttleMotorDirection[3], rev::ControlType::kVelocity);
        }
        else
        {
            throttleMotor3PIDController.SetReference(0, rev::ControlType::kVoltage);
        }   
        //---------------------------------------------------------------------------------------------------------------------
        t = 0;
        while (t < 4)
        {
        if (wheelAngle[t] > 0 && wheelPreviousAngle[t] < 0  && abs(wheelAngle[t]) > 90)
        {
            wheelAngle[t] = wheelAngle[t] - 360;      
        }
        else if (wheelAngle[t] < 0 && wheelPreviousAngle[t] > 0  && abs(wheelAngle[t]) > 90)
        {
            wheelAngle[t] = 360 + wheelAngle[t];      
        }
        t++;
        }

        steerMotorCurrentPosition[0] = steerMotor0.GetSelectedSensorPosition(0);
        steerMotor0.Set(ControlMode::PercentOutput, steerMotorDirection[0]*((steerMotorCurrentPosition[0] - steerMotorInitialPosition[0])-(int)(wheelAngle[0]*18.7277777778))*0.0005);
    
        steerMotorCurrentPosition[1] = steerMotor1.GetSelectedSensorPosition(0);
        steerMotor1.Set(ControlMode::PercentOutput, steerMotorDirection[1]*((steerMotorCurrentPosition[1] - steerMotorInitialPosition[1])-(int)(wheelAngle[1]*18.7277777778))*0.0005);

        steerMotorCurrentPosition[2] = steerMotor2.GetSelectedSensorPosition(0);
        steerMotor2.Set(ControlMode::PercentOutput, steerMotorDirection[2]*((steerMotorCurrentPosition[2] - steerMotorInitialPosition[2])-(int)(wheelAngle[2]*18.7277777778))*0.0005);

        steerMotorCurrentPosition[3] = steerMotor3.GetSelectedSensorPosition(0);
        steerMotor3.Set(ControlMode::PercentOutput, steerMotorDirection[3]*((steerMotorCurrentPosition[3] - steerMotorInitialPosition[3])-(int)(wheelAngle[3]*18.7277777778))*0.0005);

        t = 0;
        while (t < 4)
        {
            if (abs(wheelAngle[t]) > 180)
            {
                if (wheelAngle[t] < 0)
                {
                wheelAngle[t] = 360 + wheelAngle[t];
                steerMotorInitialPosition[t] = steerMotorInitialPosition[t] - 360*18.7277777778;
                }
                else if (wheelAngle[t] > 0)
                {
                wheelAngle[t] = wheelAngle[t] - 360;
                steerMotorInitialPosition[t] = steerMotorInitialPosition[t] + 360*18.7277777778;
                }
            } 
            t++;
        }

        y++;
        frc::Wait(waittime);
    } 
	
	// Stop robot once loopcount is over
	throttleMotor0PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor1PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor2PIDController.SetReference(0, rev::ControlType::kVelocity); 
	throttleMotor3PIDController.SetReference(0, rev::ControlType::kVelocity);     
}

void Auton1(void)
{
	// Auton 1 - Just Move Forward
	AutonMove(0, 0.6, 0, false, 0, 600);
	AutonMove(0, 0, 0, false, 0, 10);
	
	frc::SmartDashboard::PutString("Auton executed", "1");
}

void Auton2(void)   // Auton 2 : Shoot 3 power cells, then move forward a bit
{
    // Spin Up Shooter Motor
	shootMotorPIDController.SetReference(AutonShootRPM, rev::ControlType::kVelocity);

    frc::Wait(2);
    while (abs(shootMotorEncoder.GetVelocity() - AutonShootRPM) > 10)	// wait until shooter motor RPM is +/-10
    {
        // Display current shoot motor RPM
        frc::SmartDashboard::PutNumber("Shoot Motor Current RPM", shootMotorEncoder.GetVelocity());
    }
	
    accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
    accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor

    frc::Wait(3);	
	
	//Turn Off All Motors
	shootMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	accumulatorMotor1->Set(0);			// vertical power cell motor
	accumulatorMotor2->Set(0);			// horizontal power cell motor	
	
	// Move Forward off the Initiation Line
	AutonMove(0, 0.6, 0, false, 0, 600);
	AutonMove(0, 0, 0, false, 0, 100);

	frc::SmartDashboard::PutString("Auton executed", "2");
}

void Auton3(void) 
{		
	// Spin Up Shooter Motor
	shootMotorPIDController.SetReference(AutonShootRPM, rev::ControlType::kVelocity);

    frc::Wait(2);
    while (abs(shootMotorEncoder.GetVelocity() - AutonShootRPM) > 10)	// wait until shooter motor RPM is +/-10
    {
        // Display current shoot motor RPM
        frc::SmartDashboard::PutNumber("Shoot Motor Current RPM", shootMotorEncoder.GetVelocity());
    }
	
    accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
    accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor

    frc::Wait(3);	
	
	//Turn Off All Motors
	shootMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	accumulatorMotor1->Set(0);			// vertical power cell motor
	accumulatorMotor2->Set(0);			// horizontal power cell motor	
	
	// Move Backward off the Initiation Line
	AutonMove(0, -0.6, 0, false, 0, 600);
	AutonMove(0, 0, 0, false, 0, 100);

	frc::SmartDashboard::PutString("Auton executed", "3");
}

void Auton4(void)
{
	int shootRPMAuton = AutonShootRPM;
    int distanceRightError = 0;
    int distanceRightOffset;
    float sideAdjust = 0.1;
    float autonSpeedX, autonSpeedY, autonRotate, autonTime;
	
	// Spin Up Shooter Motor
	shootMotorPIDController.SetReference(shootRPMAuton, rev::ControlType::kVelocity);
    frc::Wait(1);       // this is to allow shooter motor RPM to settle

    while (abs(shootMotorEncoder.GetVelocity() - shootRPMAuton) > 10)	// wait until shooter motor RPM is set
    {
        // Display current shoot motor RPM
        frc::SmartDashboard::PutNumber("Shoot Motor Current RPM", shootMotorEncoder.GetVelocity());
    }
	
    // Feed preloaded power cells to shooter
    accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
    accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor

    frc::Wait(3);	    // wait until all power cells are shot out	   	
    
    // Lower pickup 
    pickupDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kForward);

    // Start Pickup Motors
    pickupMotor->Set(pickupMotorDirection);
    accumulatorMotor2->Set(0.5*accumulatorMotor2Direction); // horizontal power cell motor 

    // Stop Shooting Motors
    shootMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	accumulatorMotor1->Set(0);			// vertical power cell motor	
        
    // Move1 : Move back
    autonSpeedX = frc::SmartDashboard::GetNumber("Move1-X", -0.6);
    autonSpeedY = frc::SmartDashboard::GetNumber("Move1-Y", 0);
    autonTime = frc::SmartDashboard::GetNumber("Time1", 600);
    AutonMove(autonSpeedY, autonSpeedX, 0, false, 0, autonTime);
    //AutonMove(autonSpeedY, autonSpeedX, 0, true, 0, autonTime);
 	AutonMove(0, 0, 0, false, 0, 2);  

    // Move2 : move to the right until slightly bumps wall
    autonSpeedX = frc::SmartDashboard::GetNumber("Move2-X", 0);
    autonSpeedY = frc::SmartDashboard::GetNumber("Move2-Y", 0.5);
    autonTime = frc::SmartDashboard::GetNumber("Time2", 870);
    AutonMove(autonSpeedY, autonSpeedX, 0, false, 0, autonTime);
    //AutonMove(autonSpeedY, autonSpeedX, 0, true, 0, autonTime);
    AutonMove(0, 0, 0, false, 0, 2);    

    // Move 3 : Sensor Check/Adjust
    distanceRightOffset = frc::SmartDashboard::GetNumber("Move3-Sensor Offset", 4000);
    sideAdjust = frc::SmartDashboard::GetNumber("Move3-sideAdjust", 0.1);
    
    distanceRightError = distanceRightSensor.GetAverageValue() - distanceRightOffset;

    while (distanceRightError > 0)    
    {
        
        AutonMove(sideAdjust, 0, 0, false, 0, 1);
        //AutonMove(autonSpeedY, autonSpeedX, 0, true, 0, autonTime);
        distanceRightError = distanceRightSensor.GetAverageValue() - distanceRightOffset;
        //frc::SmartDashboard::PutNumber("Front US Sensor", distanceFrontSensor.GetAverageValue());
	    //frc::SmartDashboard::PutNumber("Right US Sensor", distanceRightSensor.GetAverageValue());
    }

    frc::SmartDashboard::PutNumber("Move3-IterStop", distanceRightError);
   
    // Move4 : move to back to pick up power cells
    autonSpeedX = frc::SmartDashboard::GetNumber("Move4-X", -0.6);
    autonSpeedY = frc::SmartDashboard::GetNumber("Move4-Y", 0);
    autonTime = frc::SmartDashboard::GetNumber("Time4", 1350);
    AutonMove(autonSpeedY, autonSpeedX, 0, false, 0, autonTime);   
    //AutonMove(autonSpeedY, autonSpeedX, 0, true, 0, autonTime); 
   	AutonMove(0, 0, 0, false, 0, 2);    
    
    // Spin Up Shooter Motor
	shootMotorPIDController.SetReference(4000, rev::ControlType::kVelocity);

    // Stop horizontal Accumulator Motor
    accumulatorMotor2->Set(0); // horizontal power cell motor 

    // Move5 : move to the front to shoot from trench
    autonSpeedX = frc::SmartDashboard::GetNumber("Move5-X", 0.6);
    autonSpeedY = frc::SmartDashboard::GetNumber("Move5-Y", 0);
    autonTime = frc::SmartDashboard::GetNumber("Time5", 1200);
    //AutonMove(autonSpeedY, autonSpeedX, 0, true, 0, autonTime);
    AutonMove(autonSpeedY, autonSpeedX, 0, false, 0, autonTime);
    AutonMove(0, 0, 0, false, 0, 2);

    // Move6 : Aim
    autonRotate = frc::SmartDashboard::GetNumber("Move6-Rotate", 0.6);
    autonTime = frc::SmartDashboard::GetNumber("Time6", 250);
    AutonMove(0, 0, autonRotate, false, 0, autonTime);
 	AutonMove(0, 0, 0, false, 0, 2);

    while (shootMotorEncoder.GetVelocity() - 4000 < 0)	// wait until shooter motor RPM is +/-5
    {
        // Display current shoot motor RPM
        frc::SmartDashboard::PutNumber("Shoot Motor Current RPM", shootMotorEncoder.GetVelocity());
    }
	
    accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
    accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor

    frc::Wait(3);	
	
	//Turn Off All Motors
	shootMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	accumulatorMotor1->Set(0);			// vertical power cell motor
	accumulatorMotor2->Set(0);			// horizontal power cell motor 

	frc::SmartDashboard::PutString("Auton executed", "4");
}

void Auton5(void) // Shoot from Middle and move back
{		
	float autonRotate, autonTime;
    
    // Spin Up Shooter Motor
	shootMotorPIDController.SetReference(AutonShootRPM, rev::ControlType::kVelocity);

     // Move7 : Aim from middle
    autonRotate = frc::SmartDashboard::GetNumber("Move7-Rotate", -0.6);
    autonTime = frc::SmartDashboard::GetNumber("Time7", 250);
    AutonMove(0, 0, autonRotate, false, 0, autonTime);
 	AutonMove(0, 0, 0, false, 0, 2);

    while (abs(shootMotorEncoder.GetVelocity() - AutonShootRPM) > 10)	// wait until shooter motor RPM is +/-10
    {
        // Display current shoot motor RPM
        frc::SmartDashboard::PutNumber("Shoot Motor Current RPM", shootMotorEncoder.GetVelocity());
    }
	
    accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
    accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor

    frc::Wait(3);	
	
	//Turn Off All Motors
	shootMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	accumulatorMotor1->Set(0);			// vertical power cell motor
	accumulatorMotor2->Set(0);			// horizontal power cell motor	
	
	// Move Backward off the Initiation Line
	AutonMove(0, -0.6, 0, false, 0, 600);
	AutonMove(0, 0, 0, false, 0, 100);

	frc::SmartDashboard::PutString("Auton executed", "5");
}

