/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// 3839 GitHub,Celtics,Pelicans, and Queen Bronny

// ROBOT INCLUDES ----------------------------------------
#include "Robot.h"
#include "frc/Preferences.h"
#include "frc/WPILib.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>


// COMPONENT INCLUDES ------------------------------------
#include "Components/NavX/AHRS.h"
#include "Components/Motors.h"
#include "Components/Controller_2Axis.h"		// Joystick + Rotary Knob for Driving
#include "Components/Pneumatics.h"
#include "Components/IRSensor.h"
AHRS *navxGyro;
frc::AnalogInput distanceFrontSensor{0};
frc::AnalogInput distanceRightSensor{1};

// SYSTEM INCLUDES ---------------------------------------
float X0, Y0, W0;
int ShootRPM = 4000;	
int AutonShootRPM = 3800;

#include "Systems/SwerveDrive.h"
#include "Systems/Pickup.h"
#include "Systems/ControlPanel.h"
#include "Systems/Accumulator.h"
#include "Systems/Shooter.h"
#include "Systems/Climber.h"
#include "Systems/AutonRoutines.h"

void Robot::RobotInit() {

	m_chooser.SetDefaultOption("Auton 0 : Do Nothing", kAutoNameDefault);
	m_chooser.AddOption("Auton 1 : Move Fwd", kAuto1NameCustom);
	m_chooser.AddOption("Auton 2 : Shoot + Move Fwd", kAuto2NameCustom);
	m_chooser.AddOption("Auton 3 : Shoot + Move Back", kAuto3NameCustom);
	m_chooser.AddOption("Auton 4 : Shoot + Extras", kAuto4NameCustom);
	m_chooser.AddOption("Auton 5 : Shoot From Middle + Move Back", kAuto5NameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
	// navX Gyroscope
	navxGyro = new AHRS(SerialPort::Port::kUSB);
	navxGyro->ZeroYaw();
	//Alternative Ports:  I2C::Port::kMXP, SerialPort::Port::kMXP or SPI::Port::kMXP 
	
	// SET UP COMPONENTS
	SetUpControllers();
	SetUpPneumatics();
	SetupIRSensors();  
	
	// Analog MaxBotix Ultrasonic Sensors
	distanceFrontSensor.SetOversampleBits(4);
	distanceRightSensor.SetOversampleBits(4);
	
	distanceFrontSensor.SetAverageBits(4);
	distanceRightSensor.SetAverageBits(4);
	
	// frc::SmartDashboard::PutNumber("Front Distance", distanceFrontSensor.GetAverageValue());
	// frc::SmartDashboard::PutNumber("Right Distance", distanceRightSensor.GetAverageValue());

	SetupMotors();
	
	// SET UP INITIAL POSITION OF SYSTEMS
	pickupArmPosition = pickupDownSolenoidPosition;
	SetPickupArmPosition(pickupUpSolenoidPosition);

	SetClimberPosition(climberDownSolenoidPosition);
	//isClimberMotorEnabled = false;
	//SetPickupMotors(pickupMotorStop);

	// GET JOYSTICK ZEROES
	X0 = translateJoystick->GetX();  // Side-to-side speed
	Y0 = translateJoystick->GetY();  // Fore-aft speed
	W0 = rotateJoystick->GetX(); // Rotation speed 	
	
	// Set Modifiable Robot Options & Values
	frc::SmartDashboard::PutNumber("Auton Shoot 1 RPM", AutonShootRPM);
	frc::SmartDashboard::PutNumber("Desired ShootRPM", ShootRPM); 
	frc::SmartDashboard::PutNumber("Front US Sensor", distanceFrontSensor.GetAverageValue());
	frc::SmartDashboard::PutNumber("Right US Sensor", distanceRightSensor.GetAverageValue());
	
	// ============================================================
	// SmartDashboard Values for Auton4
	// ============================================================
	// Move1 : Move back
	frc::SmartDashboard::PutNumber("Move1-X", -0.6);
	frc::SmartDashboard::PutNumber("Move1-Y", 0);
	frc::SmartDashboard::PutNumber("Time1", 600);

	// Move2 : move to the right until slightly bumps wall
	frc::SmartDashboard::PutNumber("Move2-X", 0);
	frc::SmartDashboard::PutNumber("Move2-Y", 0.5);
	frc::SmartDashboard::PutNumber("Time2", 870);

	//Move3 : Fine side adjustment with Ultrasonic Sensor
	frc::SmartDashboard::PutNumber("Move3-Sensor Offset", 4000);
    frc::SmartDashboard::PutNumber("Move3-sideAdjust", 0.1);
	frc::SmartDashboard::PutNumber("Move3-IterStop", 0);

	// Move4 : move to back to pick up power cells
	frc::SmartDashboard::PutNumber("Move4-X", -0.6);
	frc::SmartDashboard::PutNumber("Move4-Y", 0);
	frc::SmartDashboard::PutNumber("Time4", 1350);

	// Move5 : move to the front to shoot from trench
	frc::SmartDashboard::PutNumber("Move5-X", 0.6);
	frc::SmartDashboard::PutNumber("Move5-Y", 0);
	frc::SmartDashboard::PutNumber("Time5", 1200);

	// Move6 : Aim
	frc::SmartDashboard::PutNumber("Move6-Rotate", 0);
	frc::SmartDashboard::PutNumber("Time6", 0);

	frc::SmartDashboard::PutNumber("Auton Loop Wait Time", 0.002);
	// ============================================================
    // SmartDashboard Values for Auton5
	// ============================================================
	frc::SmartDashboard::PutNumber("Move7-Rotate", -0.6);
	frc::SmartDashboard::PutNumber("Time7", 250);
	// ============================================================
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
	
  // Check chosen Autonomous Mode
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  
//   // Check chosen Auton start position
//   m_autonStartPositionSelected = m_autonStartPositionChooser.GetSelected();
  
//   // Check chosen Wheel Zero-ing Mode
//   m_zeroWheelsSelected = m_zeroWheelsChooser.GetSelected();

// Check RPM for first shot (with preloaded power cells)
AutonShootRPM = frc::SmartDashboard::GetNumber("Auton Shoot 1 RPM", AutonShootRPM);

}

void Robot::AutonomousPeriodic() {

	// We are starting auton using ma'homies technology, so no automatic zeroing
	navxGyro->ZeroYaw();    
	FaceAngle = navxGyro->GetYaw()*deg2rad;
	SetSteerMotorInitialPosition();
		
	// -------------------------------------------------------------------------------------
	// START "AUTON" AUTON
	// -------------------------------------------------------------------------------------
	if (m_autoSelected == kAuto1NameCustom) 
	{ 
		Auton1();
	} 
	else if (m_autoSelected == kAuto2NameCustom) 
	{ 
		Auton2();	
	}
	else if (m_autoSelected == kAuto3NameCustom) 
	{ 
		Auton3();
	}
	else if (m_autoSelected == kAuto4NameCustom) 
	{ 
		Auton4();
	}
	else //-----------------------------------------------------------------------------
	{ 
		// Default Auto goes here : don't move
		frc::SmartDashboard::PutString("Auton executed", "0");
	}

	while (IsAutonomous());
}

void Robot::TeleopInit()
{
	shootMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	isShooterMotorSpinning = false;
	accumulatorMotor1->Set(0);		// vertical power cell motor
	accumulatorMotor2->Set(0);		// horizontal power cell motor  
	pickupMotor->Set(0);
	SetPickupArmPosition(pickupUpSolenoidPosition);
	ShootRPM = frc::SmartDashboard::GetNumber("Desired ShootRPM", 4000);
}

void Robot::TeleopPeriodic() 
{

  float X, Y, W, A;  
  int t, shootcounter, swervedrivezerocounter; 

  while (IsOperatorControl() && IsEnabled())
  {

	frc::SmartDashboard::PutNumber("Front US Sensor", distanceFrontSensor.GetAverageValue());
	frc::SmartDashboard::PutNumber("Right US Sensor", distanceRightSensor.GetAverageValue());

    X = translateJoystick->GetX() - X0;  // Side-to-side speed
    Y = translateJoystick->GetY() - Y0;  // Fore-aft speed
    //W = translateJoystick->GetZ() - W0;  // Rotation speed 
    W = rotateJoystick->GetX() - W0;
	
    A = navxGyro->GetYaw();  	
	SwerveDriveMe(Y*0.8, -X*0.8, -W*0.4, A, false, 0);	
    
    //---------------------------------------------------------------------------------------------------------------------
	// Set Throttle Motors (Spark Maxes & Neos)
	
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
    //frc::SmartDashboard::PutNumber("Front Right Steer Motor Initial", steerMotorInitialPosition[0]);
    //frc::SmartDashboard::PutNumber("Front Right Steer Motor Current", steerMotorCurrentPosition[0]);
    //frc::SmartDashboard::PutNumber("Front Right Steer Motor Diff", steerMotorCurrentPosition[0] - steerMotorInitialPosition[0]);
    steerMotor0.Set(ControlMode::PercentOutput, steerMotorDirection[0]*((steerMotorCurrentPosition[0] - steerMotorInitialPosition[0])-(int)(wheelAngle[0]*18.7277777778))*0.0005);
  
    steerMotorCurrentPosition[1] = steerMotor1.GetSelectedSensorPosition(0);
    //frc::SmartDashboard::PutNumber("Front Left Steer Motor Initial", steerMotorInitialPosition[1]);
    //frc::SmartDashboard::PutNumber("Front Left Steer Motor Current", steerMotorCurrentPosition[1]);
    //frc::SmartDashboard::PutNumber("Front Left Steer Motor Diff", steerMotorCurrentPosition[1] - steerMotorInitialPosition[1]);
    steerMotor1.Set(ControlMode::PercentOutput, steerMotorDirection[1]*((steerMotorCurrentPosition[1] - steerMotorInitialPosition[1])-(int)(wheelAngle[1]*18.7277777778))*0.0005);

    steerMotorCurrentPosition[2] = steerMotor2.GetSelectedSensorPosition(0);
    //frc::SmartDashboard::PutNumber("Rear Left Steer Motor Initial", steerMotorInitialPosition[2]);
    //frc::SmartDashboard::PutNumber("Rear Left Steer Motor Current", steerMotorCurrentPosition[2]);
    //frc::SmartDashboard::PutNumber("Rear Left Steer Motor Diff", steerMotorCurrentPosition[2] - steerMotorInitialPosition[2]);
    steerMotor2.Set(ControlMode::PercentOutput, steerMotorDirection[2]*((steerMotorCurrentPosition[2] - steerMotorInitialPosition[2])-(int)(wheelAngle[2]*18.7277777778))*0.0005);

    steerMotorCurrentPosition[3] = steerMotor3.GetSelectedSensorPosition(0);
    //frc::SmartDashboard::PutNumber("Rear Right Steer Motor Initial", steerMotorInitialPosition[3]);
    //frc::SmartDashboard::PutNumber("Rear Right Steer Motor Current", steerMotorCurrentPosition[3]);
    //frc::SmartDashboard::PutNumber("Rear Right Steer Motor Diff", steerMotorCurrentPosition[3] - steerMotorInitialPosition[3]);
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
	
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// // PICKUP ARM TOGGLE
	if(coDriverJoystick->GetRawButton(pickupArmUpButton)) 
	{
		while(coDriverJoystick->GetRawButton(pickupArmUpButton));
		SetPickupArmPosition(pickupUpSolenoidPosition);
	}

	if(coDriverJoystick->GetRawButton(pickupArmDownButton)) 
	{
		while(coDriverJoystick->GetRawButton(pickupArmDownButton));
		SetPickupArmPosition(pickupDownSolenoidPosition);
	}
	// //---------------------------------------------------------------------------------------------------------------------------------------------
	// PICKUP MOTORS
	if(coDriverJoystick->GetRawButton(pickupMotorIntakeButton))   // if pickup intake motors button is held
	{
		SetPickupMotors(pickupMotorIntake);
	}
	else if(coDriverJoystick->GetRawButton(pickupMotorReverseButton))   // if pickup intake motors button is held
	{
		SetPickupMotors(pickupMotorReverse);
	}
	else
	{
		SetPickupMotors(pickupMotorStop);
	}	
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// SHOOTER
	
	// Display current shoot motor RPM
	frc::SmartDashboard::PutNumber("Shoot Motor Current RPM", shootMotorEncoder.GetVelocity());

	if (shootMotorEncoder.GetVelocity() > 10)
	{
		frc::SmartDashboard::PutString("Shoot Motor Status", "ON");
	}
	else
	{
		frc::SmartDashboard::PutString("Shoot Motor Status", "OFF");
	}
	
	//-------------------------------------------------------------------------
	if (coDriverJoystick->GetRawButton(shootMotorONButton))	// spin up shooter motor
	{			
		while(coDriverJoystick->GetRawButton(shootMotorONButton));	// motor will spin as soon as button is released

		//-----------------------------------------------------
		// this is to experiment with shooter Motor PID loop
		// comment out once we found optimum control gain parameters
		// ShootMotorGainP = frc::SmartDashboard::GetNumber("Shoot Motor Gain P", 6e-5); 
		// ShootMotorGainI = frc::SmartDashboard::GetNumber("Shoot Motor Gain I", 1e-6);
		// ShootMotorGainD = frc::SmartDashboard::GetNumber("Shoot Motor Gain D", 0);

		// shootMotorPIDController.SetP(ShootMotorGainP);
		// shootMotorPIDController.SetI(ShootMotorGainI);
		// shootMotorPIDController.SetD(ShootMotorGainD);
		//-----------------------------------------------------

		if (coDriverJoystick->GetZ() > 0.8)
		{
			ShootRPM = frc::SmartDashboard::GetNumber("Desired ShootRPM", 0);
		}
		else
		{
			ShootRPM = (5000/1.8)*(-coDriverJoystick->GetRawAxis(3)+0.8)+2000 + 558;
		}	

		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVelocity);
		isShooterMotorSpinning = true;
	}

	if (coDriverJoystick->GetRawButton(shootMotorOFFButton))	// stop motor
	{
		while(coDriverJoystick->GetRawButton(shootMotorOFFButton)); // motor will stop as soon as button is released

		ShootRPM = 0;
		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVoltage);
		isShooterMotorSpinning = false;		
		frc::SmartDashboard::PutString("Shoot Motor Status", "OFF");
	}
	//-------------------------------------------------------------------------
	if (isShooterMotorSpinning)
	{
		if (coDriverJoystick->GetZ() > 0.8)
		{
			ShootRPM = frc::SmartDashboard::GetNumber("Desired ShootRPM", 0);
		}
		else
		{
			ShootRPM = (5000/1.8)*(-coDriverJoystick->GetRawAxis(3)+0.8)+2000 + 558;
		}		
		
		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVelocity);
		isShooterMotorSpinning = true;		
	}
	else
	{
		ShootRPM = 0;
		shootMotorPIDController.SetReference(ShootRPM, rev::ControlType::kVoltage);
		isShooterMotorSpinning = false;			
		frc::SmartDashboard::PutString("Shoot Motor Status", "OFF");
	}
	//-------------------------------------------------------------------------	
	if (coDriverJoystick->GetRawButton(shootButton))   // if shoot button is held
	{
		if (isShooterMotorSpinning == true)
		{
			accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);		// vertical power cell motor
			accumulatorMotor2->Set(accumulatorMotor2Direction);			// horizontal power cell motor				
		}
		else
		{
			accumulatorMotor1->Set(0);
			accumulatorMotor2->Set(0);	
		}				
	}
	else
	{
		if (!coDriverJoystick->GetRawButton(pickupMotorReverseButton))
		{
			accumulatorMotor1->Set(0);
			accumulatorMotor2->Set(0);
		}
	}
	
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// ZERO-ING SWERVE DRIVE
	if(translateJoystick->GetRawButton(7))
	{
		while(translateJoystick->GetRawButton(7));		
		ZeroWheelsOnly();
	}
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// ROTATING SMALL ANGLES
	if(translateJoystick->GetRawButton(4)) // rotate a small angle cw ---------------------
	{
		while(translateJoystick->GetRawButton(4));
		AutonMove(0, 0, 0.6, false, 0, 50);		
	}
	
	if(translateJoystick->GetRawButton(5)) // rotate a small angle ccw --------------------
	{
		while(translateJoystick->GetRawButton(5));
		AutonMove(0, 0, -0.6, false, 0, 50);		
	}
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// CLIMBER
	ClimberFunctions();

	if (translateJoystick->GetRawButton(11))	
	{
		climberMotorPIDController.SetReference(2000, rev::ControlType::kVelocity);
	} 	
	else
	{
		climberMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	}	
	//---------------------------------------------------------------------------------------------------------------------------------------------
	// TEST AUTON MOVES TO GET THE CORRECT PID GAINS FOR DRIVING STRAIGHT
	// COMMENT OUT AFTER DONE
	// if(translateJoystick->GetRawButton(3))
	// {
	// 	while(translateJoystick->GetRawButton(3));

	// 	//AutonMove(Cross-Car, Fore-Aft, Rotation, etc....)
	// 	//	Cross-Car -> left is negative, right is positive
	// 	//	Fore-Aft -> backward is negative, forward is positive
	// 	AutonMove(0, 0, 0.6, false, 0, 200);
	// 	AutonMove(0, 0, 0, false, 0, 1);

	// 	// AutonMove(frc::SmartDashboard::GetNumber("Auton X", 0), 
	// 	// 		  frc::SmartDashboard::GetNumber("Auton Y", 0),
	// 	// 		  frc::SmartDashboard::GetNumber("Auton W", 0), 
	// 	// 		  frc::SmartDashboard::GetBoolean("Auton isSetFaceAngle", false), 
	// 	// 		  frc::SmartDashboard::GetNumber("Auton faceAngleP", 0), 
	// 	// 		  frc::SmartDashboard::GetNumber("Auton loopCount", 0));

	// 	///AutonMove(0, 0, 0, frc::SmartDashboard::GetBoolean("Auton isSetFaceAngle", false), frc::SmartDashboard::GetNumber("Auton faceAngleP", 0), 1000);
	// }
	//---------------------------------------------------------------------------------------------------------------------------------------------
    
	frc::Wait(0.0001);     // Slight delay, adjust as necessary
	
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
