#include <frc/DigitalInput.h>

//==========================================================================
// IR SENSOR DEFINITION ON DIO PINS
//--------------------------------------------------------------------------
const static bool IRSensorOpen = false;
const static bool IRSensorClosed = true;

const static bool AccumulateSensorOpen = true;
const static bool AccumulateSensorClosed = false;

const static int IRSensorDIOPin[] = {0,1,2,3,4};
bool IRSensorValue[5];

frc::DigitalInput *IRSensor0;
frc::DigitalInput *IRSensor1;
frc::DigitalInput *IRSensor2;
frc::DigitalInput *IRSensor3;
frc::DigitalInput *AccumulateSensor;
//==========================================================================

void SetupIRSensors(void)
{
    IRSensor0 = new frc::DigitalInput(IRSensorDIOPin[0]);   // Front Right corner
    //frc::SmartDashboard::PutBoolean("Front Right IRSensor", IRSensor0->Get());

    IRSensor1 = new frc::DigitalInput(IRSensorDIOPin[1]);   // Front Left corner
    //frc::SmartDashboard::PutBoolean("Front Left IRSensor", IRSensor1->Get());

    IRSensor2 = new frc::DigitalInput(IRSensorDIOPin[2]);   // Rear Left corner
    //frc::SmartDashboard::PutBoolean("Rear Left IRSensor", IRSensor2->Get());

    IRSensor3 = new frc::DigitalInput(IRSensorDIOPin[3]);   // Rear Right corner
    //frc::SmartDashboard::PutBoolean("Rear Right IRSensor", IRSensor3->Get());

    AccumulateSensor = new frc::DigitalInput(IRSensorDIOPin[4]);   // Accumulator Sensor 
    //frc::SmartDashboard::PutBoolean("Accumulator Sensor", AccumulateSensor->Get());
}

