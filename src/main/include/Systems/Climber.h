const static int climberUpSolenoidPosition = 1;
const static int climberDownSolenoidPosition = 0;
int climberPosition;
int climbercounter = 0;

//=========================================================================================
void SetClimberPosition(int position)
{
    if (position == climberUpSolenoidPosition)
    {
        climberDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
		climberPosition = climberUpSolenoidPosition;
    }

    if (position == climberDownSolenoidPosition)
    {
        climberDoubleSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
		climberPosition = climberDownSolenoidPosition;
    }
}
//=========================================================================================
void ClimberFunctions(void)
{
	//------------------------------------
	// Check Solenoid Positioning Commands
	//------------------------------------
	if(coDriverJoystick->GetRawButton(climberUpButton))
	{
		while(coDriverJoystick->GetRawButton(climberUpButton));
		{
			SetClimberPosition(climberUpSolenoidPosition); 
		}		
	} 
	
	if(coDriverJoystick->GetRawButton(climberDownButton))
	{
		while(coDriverJoystick->GetRawButton(climberDownButton));
		{
			SetClimberPosition(climberUpSolenoidPosition); 
		}
	} 
	
	//------------------------------------
	// Check Motor Commands
	//------------------------------------
	if (coDriverJoystick->GetRawButton(climberMotorEnableButton) && coDriverJoystick->GetY() > 0.8)
	{
		climberMotorPIDController.SetReference(2000, rev::ControlType::kVelocity);	// Climb up
	}
	else
	{
		climberMotorPIDController.SetReference(0, rev::ControlType::kVoltage);
	}	

}
//=========================================================================================