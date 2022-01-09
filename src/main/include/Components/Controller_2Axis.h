#include <frc/Joystick.h>

// Controller USB ports
const static int translateJoystickUSBPort = 0;
const static int rotateJoystickUSBPort = 1;
const static int coDriverJoystickUSBPort = 2;

// Controller Buttons for Driver Joystick
const static int zeroSwerveDriveButton = 11;

// Controller Buttons for Co-Driver Joystick
const static int shootButton = 1;
const static int shootMotorONButton = 3;
const static int shootMotorOFFButton = 4;

const static int pickupArmUpButton = 7;
const static int pickupArmDownButton = 8;
const static int pickupMotorIntakeButton = 9;
const static int pickupMotorReverseButton = 10;

const static int climberUpButton = 11;
const static int climberDownButton = 12;
const static int climberMotorEnableButton = 19;

const static int spinnerUpButton = 20;
const static int spinnerDownButton = 21;


// Define controller objects
frc::Joystick *translateJoystick;
frc::Joystick *rotateJoystick;
frc::Joystick *coDriverJoystick;

void SetUpControllers(void)
{
    translateJoystick = new frc::Joystick(translateJoystickUSBPort);
    rotateJoystick = new frc::Joystick(rotateJoystickUSBPort);
    coDriverJoystick = new frc::Joystick(coDriverJoystickUSBPort);
}
