/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
 
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Auton0";	// Do Nothing
  const std::string kAuto1NameCustom = "Auton1";	// Just Move Forward
  const std::string kAuto2NameCustom = "Auton2";	// Shoot 3 and Move Forward
  const std::string kAuto3NameCustom = "Auton3";	// Shoot 3 and Move Back
  const std::string kAuto4NameCustom = "Auton4";	// Shoot 3 + Extras
  const std::string kAuto5NameCustom = "Auton5";	// Shoot From Middle and move back
  std::string m_autoSelected;
  
  // frc::SendableChooser<std::string> m_autonStartPositionChooser;
  // const std::string kAutonStartPositionDefault = "Right";	// Start at Right in front of Goal
  // const std::string kAutonStartPosition1NameCustom = "Middle";	// Start at Middle Line
  // const std::string kAutonStartPosition2NameCustom = "Left";	// Start at Left
  // std::string m_autonStartPositionSelected;
  
  // frc::SendableChooser<std::string> m_zeroWheelsChooser;
  // const std::string kZeroWheelsDefault = "Automatic";
  // const std::string kZeroWheelsCustom = "Manual";
  // std::string m_zeroWheelsSelected;
  
  // frc::SendableChooser<std::string> m_shootControlChooser;
  // const std::string kshootControlDefault = "Let It Fly";
  // const std::string kshootControlNameCustom = "RPM-controlled";
  // std::string m_shootControlSelected;
  
};
