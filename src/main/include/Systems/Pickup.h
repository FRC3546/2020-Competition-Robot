// Pickup Air Cylinder Positions
const static int pickupUpSolenoidPosition = 1;
const static int pickupDownSolenoidPosition = 0;

// Pickup Motor Actions
const static int pickupMotorIntake = 1;
const static int pickupMotorStop = 0;
const static int pickupMotorReverse = -1;

int pickupArmPosition = pickupUpSolenoidPosition;
bool pickupArmPositionChange = false;

int pickupMotorAction = pickupMotorStop;
int pickupButtonHeldCount = 0;

//===========================================================================
void SetPickupMotors(int motorAction)
{
	if (motorAction == pickupMotorStop)
    {
        pickupMotor->Set(0);
        accumulatorMotor2->Set(0);								// power cell horizontal motion motor
    }
    
    if (motorAction == pickupMotorIntake)
    {
        pickupMotor->Set(pickupMotorDirection);
        accumulatorMotor2->Set(0.5*accumulatorMotor2Direction);		// power cell horizontal motion motor
        
        if(AccumulateSensor->Get() == AccumulateSensorOpen)
        {
            accumulatorMotor1->Set(0.5*accumulatorMotor1Direction);
        }
        else
        {
            accumulatorMotor1->Set(0);
        }        
    }
	
	if (motorAction == pickupMotorReverse)
    {
        pickupMotor->Set(-0.5*pickupMotorDirection);
        accumulatorMotor2->Set(-0.5*accumulatorMotor2Direction);	// power cell horizontal motion motor
        accumulatorMotor1->Set(-0.5*accumulatorMotor1Direction);	// power cell vertical motion motor
    }
}
//===========================================================================
void SetPickupArmPosition(int position)
{
    if (position == pickupUpSolenoidPosition)
    {
        pickupDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
        pickupArmPosition = pickupUpSolenoidPosition;
    }
    
    if (position == pickupDownSolenoidPosition)
    {
        pickupDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
        pickupArmPosition = pickupDownSolenoidPosition;
    }
}
//===========================================================================








