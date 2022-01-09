#include <frc/Joystick.h>

// Controller USB ports
const static int translateJoystickUSBPort = 0;
const static int rotateJoystickUSBPort = 1;
const static int coDriverJoystickUSBPort = 3;

// Controller Buttons for Logitech Extreme 3D Pro Joystick
const static int shootButton = 1;


const static int pickupArmToggleButton = 6;
const static int pickupMotorReverseButton = 7;
const static int pickupMotorIntakeButton = 9;

const static int zeroSwerveDriveButton = 12;

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
