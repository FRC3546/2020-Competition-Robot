 //Link to SparkMAX libraries :https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json
//Link to Talon SRX :http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json

#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/VictorSP.h>

//For throttle and steer motors, array index 0 = right front corner,
// 1 = left front corner, 2 = left rear corner, 3 = right rear corner
//==================================================================================
// Talon SRX Steer motor parameters
//const static int steerMotorCANID[] = {40,41,42,43}; 
const static int steerMotorCANID[] = {42,43,40,41};

int steerMotorDirection[] = {1,1,1,1};		// original - pickup arm forward
//int steerMotorDirection[] = {-1,-1,-1,-1};

const static int encoderCountsPerRevolution = 4096;
const static int kTimeoutMs = 0;
const static int kPIDLoopIdx = 0;
const static int peakCurrent = 30;
const static int peakCurrentDuration = 200;
const static int limitCurrent = 0;
const static bool limitCurrentEnable = false;
const static double steerMotorP[] = {0.05, 0.05, 0.05, 0.05};
const static double talonF = 0.000015;
const static double talonP = 0.1;
const static double talonI = 1e-6;
const static double talonD = 0.0;
int steerMotorInitialPosition[4];
int steerMotorCurrentPosition[4];

// Define steer motor object - Talon SRX
TalonSRX steerMotor0 = {steerMotorCANID[0]}; //Right Front Corner
TalonSRX steerMotor1 = {steerMotorCANID[1]}; //left Front Corner
TalonSRX steerMotor2 = {steerMotorCANID[2]}; //left Rear Corner
TalonSRX steerMotor3 = {steerMotorCANID[3]}; //Right Rear Corner

//==========================================================================
// SPARK MAX BRUSHLESS NEO CAN MOTOR DEFINITION
//--------------------------------------------------------------------------
//static const int throttleMotorCANID[] = {20,21,22,23,24};   // this is SparkMax CAN ID
static const int throttleMotorCANID[] = {22,23,20,21,24};   // this is SparkMax CAN ID
int throttleMotorDirection[] = {1,-1, 1,-1};	// original - pickup arm forward
//int throttleMotorDirection[] = {-1,1, -1, 1};
static const int throttleMaxRPM = 0;

static const int climberMotorCANID = 28;
static const int climberMotorDirection = 1;

rev::CANSparkMax throttleMotor0{throttleMotorCANID[0], rev::CANSparkMax::MotorType::kBrushless}; // this is right front corner
rev::CANSparkMax throttleMotor1{throttleMotorCANID[1], rev::CANSparkMax::MotorType::kBrushless}; // this is left front corner
rev::CANSparkMax throttleMotor2{throttleMotorCANID[2], rev::CANSparkMax::MotorType::kBrushless}; // this is left rear corner
rev::CANSparkMax throttleMotor3{throttleMotorCANID[3], rev::CANSparkMax::MotorType::kBrushless}; // this is right rear corner
rev::CANSparkMax shootMotor{throttleMotorCANID[4], rev::CANSparkMax::MotorType::kBrushless}; // this is the shooter
rev::CANSparkMax climberMotor{climberMotorCANID, rev::CANSparkMax::MotorType::kBrushless}; // this is the climb motor

rev::CANPIDController throttleMotor0PIDController = throttleMotor0.GetPIDController();// this is right front corner    
rev::CANPIDController throttleMotor1PIDController = throttleMotor1.GetPIDController();// this is left front corner
rev::CANPIDController throttleMotor2PIDController = throttleMotor2.GetPIDController();// this is left rear corner 
rev::CANPIDController throttleMotor3PIDController = throttleMotor3.GetPIDController();// this is right rear corner 
rev::CANPIDController shootMotorPIDController = shootMotor.GetPIDController();// this is the shooter
rev::CANPIDController climberMotorPIDController = climberMotor.GetPIDController();// this is the climber motor    

rev::CANEncoder throttleMotor0Encoder = throttleMotor0.GetEncoder(); // right front corner encoder object
rev::CANEncoder throttleMotor1Encoder = throttleMotor1.GetEncoder(); // left front corner encoder object
rev::CANEncoder throttleMotor2Encoder = throttleMotor2.GetEncoder(); // left rear corner encoder object
rev::CANEncoder throttleMotor3Encoder = throttleMotor3.GetEncoder(); // right rear corner encoder object 
rev::CANEncoder shootMotorEncoder = shootMotor.GetEncoder(); // shooter encoder object
rev::CANEncoder climberMotorEncoder = climberMotor.GetEncoder(); // climber encoder object 

double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0; // PID coefficients
float ShootMotorGainP, ShootMotorGainI, ShootMotorGainD;		// shooter motor PID gains
//==========================================================================

// PWM Channels - Victor SP
const static int pickupMotorPWM = 7;
const static int accumulatorMotor1PWM = 5;
const static int accumulatorMotor2PWM = 6;
const static int controlPanelMotorPWM = 3;

// PWM Motor direction - Victor SP
const static int pickupMotorDirection = -1; 
const static int accumulatorMotor1Direction = -1;
const static int accumulatorMotor2Direction = -1; 
const static int controlPanelMotorDirection = 1;
 
// Define motor objects - Victor SP
frc::VictorSP *pickupMotor;
frc::VictorSP *accumulatorMotor1;		// vertical power cell motor
frc::VictorSP *accumulatorMotor2;		// horizontal power cell motor 
frc::VictorSP *controlPanelMotor;

void SetupMotors(void)
{	
    //======================================================================================
    // steerMotor0 Parameters - Talon SRX
	steerMotor0.SetNeutralMode(NeutralMode::Coast);		// Put motor in coast during neutral
	// set encoder as sensor
	steerMotor0.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	steerMotor0.SetSensorPhase(true);
	// set the peak and nominal outputs 
	steerMotor0.ConfigNominalOutputForward(0, kTimeoutMs);
	steerMotor0.ConfigNominalOutputReverse(0, kTimeoutMs);
	steerMotor0.ConfigPeakOutputForward(1, kTimeoutMs);
	steerMotor0.ConfigPeakOutputReverse(-1, kTimeoutMs);
	// set PID gains at PID slot 0, if needed later
	steerMotor0.Config_kF(kPIDLoopIdx, talonF, kTimeoutMs);
	steerMotor0.Config_kP(kPIDLoopIdx, talonP, kTimeoutMs);
	steerMotor0.Config_kI(kPIDLoopIdx, talonI, kTimeoutMs);
	steerMotor0.Config_kD(kPIDLoopIdx, talonD, kTimeoutMs);   

	// current limiting
	steerMotor0.ConfigPeakCurrentLimit(peakCurrent, 10);             // peak current limit = 25 Amps
	steerMotor0.ConfigPeakCurrentDuration(peakCurrentDuration, 10);  // 200ms duration of peak current, before limiting kicks in
	steerMotor0.ConfigContinuousCurrentLimit(limitCurrent, 10);      // current limit, if the above 2 events happened
	steerMotor0.EnableCurrentLimit(limitCurrentEnable);              // turn it on 
    //----------------------------------------------------------------------------------
    // steer1Motor Parameters - Talon SRX
	steerMotor1.SetNeutralMode(NeutralMode::Coast);		// Put motor in coast during neutral
	// set encoder as sensor
	steerMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	steerMotor1.SetSensorPhase(true);
	// set the peak and nominal outputs 
	steerMotor1.ConfigNominalOutputForward(0, kTimeoutMs);
	steerMotor1.ConfigNominalOutputReverse(0, kTimeoutMs);
	steerMotor1.ConfigPeakOutputForward(1, kTimeoutMs);
	steerMotor1.ConfigPeakOutputReverse(-1, kTimeoutMs);
	// set PID gains at PID slot 0, if needed later
	steerMotor1.Config_kF(kPIDLoopIdx, talonF, kTimeoutMs);
	steerMotor1.Config_kP(kPIDLoopIdx, talonP, kTimeoutMs);
	steerMotor1.Config_kI(kPIDLoopIdx, talonI, kTimeoutMs);
	steerMotor1.Config_kD(kPIDLoopIdx, talonD, kTimeoutMs);

	// current limiting
	steerMotor1.ConfigPeakCurrentLimit(peakCurrent, 10);             // peak current limit = 25 Amps
	steerMotor1.ConfigPeakCurrentDuration(peakCurrentDuration, 10);  // 200ms duration of peak current, before limiting kicks in
	steerMotor1.ConfigContinuousCurrentLimit(limitCurrent, 10);      // current limit, if the above 2 events happened
	steerMotor1.EnableCurrentLimit(limitCurrentEnable);              // turn it on 
    //----------------------------------------------------------------------------------------------------------------
    // steerMotor2 Parameters - Talon SRX
	steerMotor2.SetNeutralMode(NeutralMode::Coast);		// Put motor in coast during neutral
	// set encoder as sensor
	steerMotor2.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	steerMotor2.SetSensorPhase(true);
	// set the peak and nominal outputs 
	steerMotor2.ConfigNominalOutputForward(0, kTimeoutMs);
	steerMotor2.ConfigNominalOutputReverse(0, kTimeoutMs);
	steerMotor2.ConfigPeakOutputForward(1, kTimeoutMs);
	steerMotor2.ConfigPeakOutputReverse(-1, kTimeoutMs);
	// set PID gains at PID slot 0, if needed later
	steerMotor2.Config_kF(kPIDLoopIdx, talonF, kTimeoutMs);
	steerMotor2.Config_kP(kPIDLoopIdx, talonP, kTimeoutMs);
	steerMotor2.Config_kI(kPIDLoopIdx, talonI, kTimeoutMs);
	steerMotor2.Config_kD(kPIDLoopIdx, talonD, kTimeoutMs);

 	// current limiting
	steerMotor2.ConfigPeakCurrentLimit(peakCurrent, 10);             // peak current limit = 25 Amps
	steerMotor2.ConfigPeakCurrentDuration(peakCurrentDuration, 10);  // 200ms duration of peak current, before limiting kicks in
	steerMotor2.ConfigContinuousCurrentLimit(limitCurrent, 10);      // current limit, if the above 2 events happened
	steerMotor2.EnableCurrentLimit(limitCurrentEnable);              // turn it on 
    //-------------------------------------------------------------------------------------------------------------------------
    // steerMotor3 Parameters - Talon SRX
	steerMotor3.SetNeutralMode(NeutralMode::Coast);		// Put motor in coast during neutral
	// set encoder as sensor
	steerMotor3.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	steerMotor3.SetSensorPhase(true);
	// set the peak and nominal outputs 
	steerMotor3.ConfigNominalOutputForward(0, kTimeoutMs);
	steerMotor3.ConfigNominalOutputReverse(0, kTimeoutMs);
	steerMotor3.ConfigPeakOutputForward(1, kTimeoutMs);
	steerMotor3.ConfigPeakOutputReverse(-1, kTimeoutMs);
	// set PID gains at PID slot 0, if needed later
	steerMotor3.Config_kF(kPIDLoopIdx, talonF, kTimeoutMs);
	steerMotor3.Config_kP(kPIDLoopIdx, talonP, kTimeoutMs);
	steerMotor3.Config_kI(kPIDLoopIdx, talonI, kTimeoutMs);
	steerMotor3.Config_kD(kPIDLoopIdx, talonD, kTimeoutMs);

   	// current limiting
	steerMotor3.ConfigPeakCurrentLimit(peakCurrent, 10);             // peak current limit = 25 Amps
	steerMotor3.ConfigPeakCurrentDuration(peakCurrentDuration, 10);  // 200ms duration of peak current, before limiting kicks in
	steerMotor3.ConfigContinuousCurrentLimit(limitCurrent, 10);      // current limit, if the above 2 events happened
	steerMotor3.EnableCurrentLimit(limitCurrentEnable);              // turn it on 
	//======================================================================================================
	//Spark Max NEO Motor parameters - ThrottleMotor0
	throttleMotor0.RestoreFactoryDefaults();
	//throttleMotor0.SetIdleMode(rev::CANSparkMax::IdleMode::Coast);
	
	// set PID coefficients for throttle motor
	throttleMotor0PIDController.SetP(kP);
	throttleMotor0PIDController.SetI(kI);
	throttleMotor0PIDController.SetD(kD);
	throttleMotor0PIDController.SetIZone(kIz);
	throttleMotor0PIDController.SetFF(kFF);
	throttleMotor0PIDController.SetOutputRange(kMinOutput, kMaxOutput);
    //-----------------------------------------------------------------------------------------------------------
	//Spark Max NEO Motor parameters - ThrottleMotor1
	throttleMotor1.RestoreFactoryDefaults();

	// set PID coefficients
	throttleMotor1PIDController.SetP(kP);
	throttleMotor1PIDController.SetI(kI);
	throttleMotor1PIDController.SetD(kD);
	throttleMotor1PIDController.SetIZone(kIz);
	throttleMotor1PIDController.SetFF(kFF);
	throttleMotor1PIDController.SetOutputRange(kMinOutput, kMaxOutput);
    //------------------------------------------------------------------------------------------------------------
	//Spark Max NEO Motor parameters - ThrottleMotor2
	throttleMotor2.RestoreFactoryDefaults();

	// set PID coefficients
	throttleMotor2PIDController.SetP(kP);
	throttleMotor2PIDController.SetI(kI);
	throttleMotor2PIDController.SetD(kD);
	throttleMotor2PIDController.SetIZone(kIz);
	throttleMotor2PIDController.SetFF(kFF);
	throttleMotor2PIDController.SetOutputRange(kMinOutput, kMaxOutput);
     //------------------------------------------------------------------------------------------------------------
	//Spark Max NEO Motor parameters - ThrottleMotor3
	throttleMotor3.RestoreFactoryDefaults();

	// set PID coefficients
	throttleMotor3PIDController.SetP(kP);
	throttleMotor3PIDController.SetI(kI);
	throttleMotor3PIDController.SetD(kD);
	throttleMotor3PIDController.SetIZone(kIz);
	throttleMotor3PIDController.SetFF(kFF);
	throttleMotor3PIDController.SetOutputRange(kMinOutput, kMaxOutput);
    //------------------------------------------------------------------------------------------------------------
	//Spark Max NEO Motor parameters - ShootMotor
	shootMotor.RestoreFactoryDefaults();

	//set PID coefficients
	shootMotorPIDController.SetP(0.00025);
	shootMotorPIDController.SetI(0.000001);
	shootMotorPIDController.SetD(0);
	shootMotorPIDController.SetIZone(kIz);
	shootMotorPIDController.SetFF(kFF);
	shootMotorPIDController.SetOutputRange(kMinOutput, kMaxOutput);
    //=============================================================================================================
	//Spark Max NEO Motor parameters - climberMotor
	climberMotor.RestoreFactoryDefaults();
	//climbMotor.SetIdleMode(rev::CANSparkMax::IdleMode::Coast);
	
	// set PID coefficients for climber motor
	climberMotorPIDController.SetP(kP);
	climberMotorPIDController.SetI(kI);
	climberMotorPIDController.SetD(kD);
	climberMotorPIDController.SetIZone(kIz);
	climberMotorPIDController.SetFF(kFF);
	climberMotorPIDController.SetOutputRange(kMinOutput, kMaxOutput);
	//=============================================================================================================
    //Set up Victor SP motors
    pickupMotor = new frc::VictorSP(pickupMotorPWM);
    accumulatorMotor1 = new frc::VictorSP(accumulatorMotor1PWM);
    accumulatorMotor2 = new frc::VictorSP(accumulatorMotor2PWM);     
}

void SetSteerMotorInitialPosition(void)
{
    steerMotorInitialPosition[0]= steerMotor0.GetSelectedSensorPosition(0);
    steerMotorInitialPosition[1]= steerMotor1.GetSelectedSensorPosition(0);
	steerMotorInitialPosition[2]= steerMotor2.GetSelectedSensorPosition(0);
    steerMotorInitialPosition[3]= steerMotor3.GetSelectedSensorPosition(0);

	steerMotorCurrentPosition[0] = steerMotorInitialPosition[0];
    steerMotorCurrentPosition[1] = steerMotorInitialPosition[1];
    steerMotorCurrentPosition[2] = steerMotorInitialPosition[2];
    steerMotorCurrentPosition[3] = steerMotorInitialPosition[3];

	// frc::SmartDashboard::PutNumber("Front Right Steer Motor Initial", steerMotorInitialPosition[0]);
	// frc::SmartDashboard::PutNumber("Front Left Steer Motor Initial", steerMotorInitialPosition[1]);
	// frc::SmartDashboard::PutNumber("Rear Left Steer Motor Initial", steerMotorInitialPosition[2]);
	// frc::SmartDashboard::PutNumber("Rear Right Steer Motor Initial", steerMotorInitialPosition[3]);

	// frc::SmartDashboard::PutNumber("Front Right Steer Motor Current", steerMotorCurrentPosition[0]);
	// frc::SmartDashboard::PutNumber("Front Left Steer Motor Current", steerMotorCurrentPosition[1]);
	// frc::SmartDashboard::PutNumber("Rear Left Steer Motor Current", steerMotorCurrentPosition[2]);
	// frc::SmartDashboard::PutNumber("Rear Right Steer Motor Current", steerMotorCurrentPosition[3]);

	// frc::SmartDashboard::PutNumber("Front Right Steer Motor Diff", steerMotorCurrentPosition[0] - steerMotorInitialPosition[0]);
	// frc::SmartDashboard::PutNumber("Front Left Steer Motor Diff", steerMotorCurrentPosition[1] - steerMotorInitialPosition[1]);
	// frc::SmartDashboard::PutNumber("Rear Left Steer Motor Diff", steerMotorCurrentPosition[2] - steerMotorInitialPosition[2]);
	// frc::SmartDashboard::PutNumber("Rear Right Steer Motor Diff", steerMotorCurrentPosition[3] - steerMotorInitialPosition[3]);
}

// 3839 GitHub,Celtics,Pelicans, and Queen Bronny
