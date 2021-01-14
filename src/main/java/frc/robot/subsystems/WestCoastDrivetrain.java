/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class WestCoastDrivetrain extends GenericDrivetrain {
  /**
   * Creates a new WestCoastDrivetrain.
   */

  private WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(LEFT_MOTOR_1);
  private WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(LEFT_MOTOR_2);
  private WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(LEFT_MOTOR_3);

  private WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RIGHT_MOTOR_1);
  private WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RIGHT_MOTOR_2);
  private WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RIGHT_MOTOR_3);
  
  private SpeedControllerGroup leftMotorGroup;
  private SpeedControllerGroup rightMotorGroup;

  private DifferentialDrive drivetrain;

  public WestCoastDrivetrain() {

    if (MOTOR_COUNT == 2)
    {
      leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2);
      rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2);
    }
    else if (MOTOR_COUNT == 3)
    {
      leftMotorGroup = new SpeedControllerGroup(leftMotor1, leftMotor2, leftMotor3);
      rightMotorGroup = new SpeedControllerGroup(rightMotor1, rightMotor2, rightMotor3);
    }

    drivetrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void drive(double leftJoyX, double leftJoyY, double rightJoyX, double rightJoyY){

    double speedcap = .8;

    if(DIRECTION_MULTIPLIER == 1){

      drivetrain.tankDrive(speedcap * DIRECTION_MULTIPLIER * leftJoyY, speedcap * DIRECTION_MULTIPLIER * rightJoyY);   
    
    }
    else{

      drivetrain.tankDrive(speedcap * DIRECTION_MULTIPLIER * rightJoyY, speedcap * DIRECTION_MULTIPLIER * leftJoyY); 
    
    }
  }
  
  //Direction switching for manueverabiltiy
  public void switchDirection(){
    DIRECTION_MULTIPLIER *= -1;
    
  }
}
