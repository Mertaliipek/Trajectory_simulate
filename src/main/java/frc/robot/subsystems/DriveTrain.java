// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  
  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;
  
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);     //kP ,kI , kD
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);    //kP , kI , kD

  private final MotorController m_leftLeader = new Victor(1);
  private final MotorController m_leftFollower = new Victor(2);
  private final MotorController m_rightLeader = new Victor(3);
  private final MotorController m_rightFollower = new Victor(4);

  
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);
  private Encoder encoder = new Encoder(5,7, false, EncodingType.k4X);


  private Field2d m_field = new Field2d();

  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightLeader, m_rightFollower);
  
  private final AHRS gyro = new AHRS(Port.kMXP);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);  //double ks, double kv

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  double kP;


  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    gyro.getRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));

  public DriveTrain() {
    gyro.reset();
    
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_rightGroup.setInverted(true);

    SmartDashboard.putData("Field", m_field);

  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond); // Speed of the left side of the robot
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond); // Speed ofo the right side of the robot

    final double leftOutput =
        m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {  // xSpeed Linear velocity in m/s.   rot Angular velocity in rad/s.
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));  // omegaRadiansPerSecond , vxMetersPerSecond , 	vyMetersPerSecond
    setSpeeds(wheelSpeeds);
  }

  public void odometryUpdate() {
    m_odometry.update(
      Rotation2d.fromDegrees(gyro.getYaw())  /*gyro.getRotation2d()*/  , m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  }


  public void pidTest(double setPoint) {

    double sensorPosition = encoder.get() * kDriveTick2Feet;

    // calculations
    double error = setPoint - sensorPosition;

    double outputSpeed = kP * error;

    // output to motors
    m_leftGroup.set(outputSpeed);
    m_rightGroup.set(-outputSpeed);

  }
  @Override
  public void periodic() {
 // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    Trajectory m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("traj").setTrajectory(m_trajectory);
  
}



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
