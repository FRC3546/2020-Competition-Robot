#include <frc/Joystick.h>

// Controller USB ports
const static int translateJoystickUSBPort = 0;
const static int rotateJoystickUSBPort = 1;
const static int coDriverJoystickUSBPort = 3;

// Controller Buttons for Logitech Attack 3 Joystick
const static int shootButton = 1;
const static int faceRearButton = 2;
const static int faceFrontButton = 3;
const static int faceLeftButton = 4;
const static int faceRightButton = 5;
const static int pickupToggleButton = 6;
const static int pickupReverseMotorButton = 7;

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
