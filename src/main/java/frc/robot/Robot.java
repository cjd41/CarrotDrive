// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Radians;

import java.util.List;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final LTVUnicycleController m_feedback = new LTVUnicycleController(0.020);
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, Rotation2d.kZero),
            List.of(),
            new Pose2d(6, 4, Rotation2d.kZero),
            new TrajectoryConfig(2, 2));

    SmartDashboard.putNumber("forwardHoldPower", 1);
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
  }

  @Override
  public void autonomousInit() {
    m_timer.restart();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_feedback.calculate(m_drive.getPose(), reference);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }


  enum BackwardsBehavior {ThreePoint, DriveBackwards, OptimizedSpin};

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // double forward = -m_controller.getLeftY();
    // double turn = -m_controller.getRightX();

    BackwardsBehavior selected = BackwardsBehavior.OptimizedSpin;

    double leftX = MathUtil.applyDeadband(m_controller.getLeftX(), 0.1);
    double leftY = MathUtil.applyDeadband(-m_controller.getLeftY(), 0.1);

    
    Translation2d stick = new Translation2d(leftX, leftY);

    // uncomment to plot the joystick on the field as a "pose"
    //m_drive.getFieldSim().getObject("stick").setPose(new Pose2d(stick, new Rotation2d()));

    Translation2d heading = new Translation2d(1, new Rotation2d(m_drive.heading()));

    // uncomment to plot the chassis heading (radians, X axis = 0)
    //m_drive.getFieldSim().getObject("heading").setPose(new Pose2d(heading, new Rotation2d()));


    double dot = heading.dot(stick);
    double cross = heading.cross(stick);

    double forwardHoldPower = SmartDashboard.getNumber("forwardHoldPower", 1);

    // uncomment this for default/gentle angle following
    //double forward = dot;

    // uncomment this for variable/tighter angle following
    double forward = dotProductPowerCosine(stick, heading, forwardHoldPower);

    double turn = cross;

    // stability while driving backwards
    if(selected == BackwardsBehavior.DriveBackwards)
    {
      if(dot < 0)
        turn = -cross;
    }
    // always-forwards
    if(selected == BackwardsBehavior.OptimizedSpin)
    {
      if(dot < 0)
      {
        forward = 0;
        turn = Math.signum(cross);
      }
    }

    SmartDashboard.putNumber("dot", dot);
    SmartDashboard.putNumber("cross", cross); 

    double xSpeed = m_speedLimiter.calculate(forward) * Drivetrain.kMaxSpeed;

    
    double rot = m_rotLimiter.calculate(turn) * Drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, rot);
  }

  private double dotProductPowerCosine(Translation2d stick, Translation2d heading, double forwardHoldPower)
  {
    if(stick.dot(heading) == 0.0)
      return 0;

    double cos = Math.cos(heading.getAngle().minus(stick.getAngle()).getMeasure().in(Radians));

    return stick.getNorm() * heading.getNorm() *
      Math.pow(
        Math.abs(cos),
        forwardHoldPower - 1)
      * cos;
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
}
