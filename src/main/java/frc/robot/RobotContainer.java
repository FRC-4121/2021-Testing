/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------hello------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private WestCoastDrivetrain drivetrain = new WestCoastDrivetrain();

  //Commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveWithJoysticksCommand joysticksCommand = new DriveWithJoysticksCommand(drivetrain);
  private final DriveWithXboxCommand xboxCommand = new DriveWithXboxCommand(drivetrain);
  // private final InstantCommand switchDirection = new InstantCommand(drivetrain::switchDirection);

  //OI Devices
  private final XboxController xbox = new XboxController(XBOX_PORT);
  private final Joystick leftJoy = new Joystick(LEFT_JOY_PORT);
  private final Joystick rightJoy = new Joystick(RIGHT_JOY_PORT);
  private final Joystick gamepad = new Joystick(GAMEPAD_PORT);

  //Buttons
  private JoystickButton invertDirection_joy = new JoystickButton(rightJoy, 2);
  private JoystickButton invertDirection_xbox = new JoystickButton(xbox, 6);

  //Camera
  private UsbCamera driverCam = new UsbCamera("Driver Cam", 0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //Evidently doesn't work under the new system, sadly :(
    //Set up drivetrain
    // switch(DRIVETRAIN_TYPE) {
    //   case 0:
    //     drivetrain = new WestCoastDrivetrain();
    //     break;
    //   case 1:
    //     drivetrain = new MecanumDrivetrain();//note that mecanum is not currently supported
    //     break;
    //   default:
    //     drivetrain = new WestCoastDrivetrain();
    // }

    //Set up camera
    driverCam.setBrightness(50);
    driverCam.setResolution(640, 480);
    driverCam.setFPS(15);

    // Configure the button bindings
    configureButtonBindings();

    //Enable xbox driving
    drivetrain.setDefaultCommand(xboxCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // invertDirection_joy.whenPressed(switchDirection);
    invertDirection_xbox.whenPressed(new InstantCommand(drivetrain::switchDirection, drivetrain)); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
