// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.util.ConsoleLogger;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    // Begin logging console
    ConsoleLogger.init();

    // Begin robot
    System.out.printf("Built with WPILib %s\n", WPILibVersion.Version);
    RobotBase.startRobot(Robot::new);
  }
}
