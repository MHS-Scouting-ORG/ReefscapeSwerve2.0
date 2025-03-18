// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignModeCommand extends Command {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentricFacingAngle driveFieldFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private CommandSwerveDrivetrain drivetrain; 
    private DoubleSupplier xSupp, ySupp, zSupp; 
    private double desiredHeading, tagID; 

    // KEANI'S FUNKY PID
    private PIDController pid;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  /** Creates a new AlignModeCommands. */
  public AlignModeCommand(CommandSwerveDrivetrain drivetrain, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp) {
    this.drivetrain = drivetrain; 
    this.xSupp = xSupp; 
    this.ySupp = ySupp;
    this.zSupp = zSupp;

    pid = new PIDController(0.1, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // HEADING STUFF 
    tagID = LimelightHelpers.getFiducialID("limelight");

    double xSpeed = xSupp.getAsDouble(); 
    double ySpeed = ySupp.getAsDouble(); 
    double zSpeed = zSupp.getAsDouble(); 

    if (tagID == 20 || tagID == 11) { // FAR A 
      zSpeed = pid.calculate(120, Units.radiansToDegrees(drivetrain.getRotation3d().getZ()));
    } else if (tagID == 21 || tagID == 10) { // FAR B 
      zSpeed = pid.calculate(180, Units.radiansToDegrees(drivetrain.getRotation3d().getZ()));
    } else if (tagID == 22 || tagID == 9) { // FAR C 
      zSpeed = pid.calculate(240, Units.radiansToDegrees(drivetrain.getRotation3d().getZ()));
    } else if (tagID == 19 || tagID == 6) { // CLOSE A 
      zSpeed = pid.calculate(60, Units.radiansToDegrees(drivetrain.getRotation3d().getZ()));
    } else if (tagID == 18 || tagID == 7) { // CLOSE B 
      zSpeed = pid.calculate(0, Units.radiansToDegrees(drivetrain.getRotation3d().getZ()));
    } else if (tagID == 17 || tagID == 8) { // CLOSE C 
      zSpeed = pid.calculate(300, Units.radiansToDegrees(drivetrain.getRotation3d().getZ()));
    }

    // drivetrain.setControl(driveFieldFacingAngle
    //         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    //         .withVelocityX(-xSpeed * MaxSpeed)
    //         .withVelocityY(-ySpeed * MaxSpeed)
    //         .withTargetDirection(new Rotation2d(Units.degreesToRadians(desiredHeading)))
    //         );

    // drivetrain.applyRequest((()->
    //   driveRobotCentric.withVelocityX(-xSpeed * MaxSpeed).withVelocityY(-ySpeed * MaxSpeed).withRotationalRate(zSpeed * MaxAngularRate))
    // );
    // drivetrain.setControl(driveFieldFacingAngle
    // .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    // );

    // if (tagID == -1) {
      drivetrain.setControl(drive
      .withVelocityX(-xSpeed * MaxSpeed)
      .withVelocityY(-ySpeed * MaxSpeed)
      .withRotationalRate(-zSpeed * MaxSpeed));
    // }

    SmartDashboard.putNumber("heading", drivetrain.getPigeon2().getYaw().getValueAsDouble()); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // private double getTagHeading() {
  //       double tagID = LimelightHelpers.getFiducialID("limelight"); 

  //       if (tagID == 20 || tagID == 11) { // FAR A 
  //           return 240; 
  //       } else if (tagID == 21 || tagID == 10) { // FAR B 
  //           return 180; 
  //       } else if (tagID == 22 || tagID == 9) { // FAR C 
  //           return 120; 
  //       } else if (tagID == 19 || tagID == 6) { // CLOSE A 
  //           return 300; 
  //       } else if (tagID == 18 || tagID == 7) { // CLOSE B 
  //           return 0; 
  //       } else if (tagID == 17 || tagID == 8) { // CLOSE C 
  //           return 60; 
  //       } else {
  //         return 0;
  //       }
  //   }
}
