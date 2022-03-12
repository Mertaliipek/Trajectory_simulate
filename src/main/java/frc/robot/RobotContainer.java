// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.KinematicCmd;
import frc.robot.commands.OdometryCmd;
import frc.robot.commands.PidCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick joystick = new Joystick(0);
  private final DriveTrain m_drive = new DriveTrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  final double xSpeed = -m_speedLimiter.calculate(joystick.getY()) * DriveTrain.kMaxSpeed;

  final double rot = -m_rotLimiter.calculate(joystick.getX()) * DriveTrain.kMaxAngularSpeed;
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(
    new KinematicCmd(m_drive, xSpeed, rot)  
    );
    m_drive.setDefaultCommand(
      new OdometryCmd(m_drive)
    );


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    Trajectory m_trajectory =
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(7.708452,2.252027, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(7.394167,0.575839)),
      new Pose2d(7.708452, 2.252027, Rotation2d.fromDegrees(0)),  // x de gideceği kısım, y de gideceği kısım
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

/*new Pose2d(1.418903,1.474303, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(5.034577, 2.099259)),
      new Pose2d(7.416237, 0.586639, Rotation2d.fromDegrees(0)),  // x de gideceği kısım, y de gideceği kısım
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
*/





// Create and push Field2d to SmartDashboard.
Field2d m_field = new Field2d();
SmartDashboard.putData(m_field);

// Push the trajectory to Field2d.
m_field.getObject("traj").setTrajectory(m_trajectory);

    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup( //
    new PidCommand(m_drive, 10),
    new KinematicCmd(m_drive, xSpeed, rot)); // kino
  }


}
