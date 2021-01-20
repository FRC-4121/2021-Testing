/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WestCoastDrivetrain extends GenericDrivetrain {
  /**
   * Creates a new WestCoastDrivetrain.
   */

  //Encoders
  private CANEncoder l1_encoder;
  private CANEncoder l2_encoder;
  private CANEncoder R1_encoder;
  private CANEncoder R2_encoder;


  //Motor Controllers
  // private WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(LEFT_MOTOR_1);
  // private WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(LEFT_MOTOR_2);
  private CANSparkMax leftMotor1 = new CANSparkMax(LEFT_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(LEFT_MOTOR_2, MotorType.kBrushless);
  private WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(LEFT_MOTOR_3);

  // private WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RIGHT_MOTOR_1);
  // private WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RIGHT_MOTOR_2);
  private CANSparkMax rightMotor1 = new CANSparkMax(RIGHT_MOTOR_1, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(RIGHT_MOTOR_2, MotorType.kBrushless);
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

    initEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder Position", l1_encoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Position", R1_encoder.getPosition());

    SmartDashboard.putNumber("Left Encoder Position", l1_encoder.getVelocity());
    SmartDashboard.putNumber("Right Encoder Position", R1_encoder.getVelocity());
  }

  private void initEncoders(){

    l1_encoder = leftMotor1.getEncoder();
    l2_encoder = leftMotor2.getEncoder();
    R1_encoder = rightMotor1.getEncoder();
    R2_encoder = rightMotor2.getEncoder();

    l1_encoder.setPosition(0);
    l2_encoder.setPosition(0);
    R1_encoder.setPosition(0);
    R2_encoder.setPosition(0);
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
