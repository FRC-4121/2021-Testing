
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MecanumDrivetrain extends GenericDrivetrain {
  /**
   * Creates a new MecanumDrivetrain.
   */

  private WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(LEFT_MOTOR_1);
  private WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(LEFT_MOTOR_2);
  private WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(LEFT_MOTOR_3);
  private WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RIGHT_MOTOR_1);
  private WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RIGHT_MOTOR_2);
  private WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RIGHT_MOTOR_3);

  public MecanumDrivetrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
