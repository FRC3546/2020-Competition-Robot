 #include "frc/Compressor.h"
 #include "frc/DoubleSolenoid.h"

//-------------------------------------------
// Pneumatics Control Module 
const static int pcm0 = 0;
const static int pickupUpChannel = 2;
const static int pickupDownChannel = 3;
const static int spinnerUpChannel = 4;
const static int spinnerDownChannel = 5;
const static int climberUpChannel = 6;
const static int climberDownChannel = 7;
// const static int climbUpChannel = 6;
// const static int climbDownChannel = 7;
//-------------------------------------------
//-------------------------------------------
// Spinner Air Cylinder Positions
const static int spinnerUPSolenoidPosition = 1;
const static int spinnerDownSolenoidPosition = 0;
// const static int climbUpSolenoidPosition = 1;
// const static int climbDownSolenoidPosition = 0;
//-------------------------------------------

frc::DoubleSolenoid *pickupDoubleSolenoid;
frc::DoubleSolenoid *spinnerDoubleSolenoid;
frc::DoubleSolenoid *climberDoubleSolenoid;
//frc::DoubleSolenoid *climbUpDoubleSolenoid;
//frc::DoubleSolenoid *climbDownDoubleSolenoid;

//  Create compressor object    
frc::Compressor *c;    

void SetUpPneumatics(void)
{
    // Define Compressor
    c = new frc::Compressor(pcm0);
    c->SetClosedLoopControl(true);        // turn on compressor, set to false to turn off

    // Defining DoubleSolenoids 
	pickupDoubleSolenoid = new frc::DoubleSolenoid(pcm0, pickupUpChannel, pickupDownChannel);
	spinnerDoubleSolenoid = new frc::DoubleSolenoid(pcm0, spinnerUpChannel, spinnerDownChannel); 
    climberDoubleSolenoid = new frc::DoubleSolenoid(pcm0, climberUpChannel, climberDownChannel);
    //climbUpDoubleSolenoid = new frc::DoubleSolenoid(pcm0, climbUpChannel);
    //climbDownDoubleSolenoid = new frc::DoubleSolenoid(pcm0, climbDownChannel);   
}

void SpinnerSolenoidPosition(int position)
{
    if (position == spinnerUPSolenoidPosition)
    {
        spinnerDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    }

    if (position == spinnerDownSolenoidPosition)
    {
        spinnerDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    }
}

/* void ClimberSolenoidPosition(int position)
{
    if (position == climberUpSolenoidPosition)
    {
        climberDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    }

    if (position == climberDownSolenoidPosition)
    {
        climberDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    }
} */