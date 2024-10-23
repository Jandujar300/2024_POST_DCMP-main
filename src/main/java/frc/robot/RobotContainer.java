// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.lang.model.element.Parameterizable;

//import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
import frc.robot.util.LimelightHelpers;
//import frc.robot.util.controllerUtils.MultiButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.*;



public class RobotContainer {
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();

    

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps *.9 ; // 6 meters per second desired top speed change the decimal for speeding up
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveStick = new CommandXboxController(0); //drivestick
  private final CommandXboxController opStick = new CommandXboxController(1); // My joystick

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  SendableChooser<Command> autoChooserAuto = new SendableChooser<>();
  SendableChooser<Command> choiceWait = new SendableChooser<>();
  
  private final Command Wait0 = new WaitCommand(0);
  private final Command Wait1 = new WaitCommand(1);
  private final Command Wait2 = new WaitCommand(2);
  private final Command Wait3 = new WaitCommand(3);
  private final Command Wait4 = new WaitCommand(4);
  private final Command Wait5 = new WaitCommand(5);
  private final Command Wait6 = new WaitCommand(6);
  private final Command Wait7 = new WaitCommand(7);
  private final Command Wait8 = new WaitCommand(8);
  private final Command Wait9 = new WaitCommand(9);
  private final Command Wait10 = new WaitCommand(10);

  private final Command BottomTwoPiece = new SequentialCommandGroup(
         
           new InstantCommand(()-> shooterSubsystem.shootFlywheel(.5)),
           new WaitCommand(1.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(1),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(.5)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new PathPlannerAuto("DCMP 1"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.15),
     
     /*       // new InstantCommand(()-> intakeSubsystem.rollStop()),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Top Close to Shoot")),
          //new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(.5),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),  */
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop()),
          new PathPlannerAuto("DCMP 2")
          );
  

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0175;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight-shooter") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .0175;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-shooter") * kP;
    targetingForwardSpeed *= MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

// simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional_intake()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .0175;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocityIntake = LimelightHelpers.getTX("limelight-intake") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocityIntake *= MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocityIntake *= -1.0;

    return targetingAngularVelocityIntake;
  }

   double limelight_range_proportional_intake()
  {    
    double kP = .0125;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight-intake") * kP;
    targetingForwardSpeed *= MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void configureBindings() {

    //Driver Xbox Controller
  // default command
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
//slow command
    driveStick.leftTrigger().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed*(.5)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.5)) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate*(.35) // Drive counterclockwise with negative X (left)
        )));    
    
  //slow command while intaking  
    driveStick.rightTrigger().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed*(.5)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.35)) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate*(.6) // Drive counterclockwise with negative X (left)
        )));       
   
   driveStick.rightTrigger().whileTrue(
      new StartEndCommand(()-> intakeSubsystem.roll(.3), intakeSubsystem::rollStop));

   driveStick.rightTrigger().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.spinBump(.45), shooterSubsystem::stopBump));     

   driveStick.rightTrigger().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.shootFlywheel(-.175), shooterSubsystem::stopFlywheel));  

   driveStick.rightBumper().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.shootFlywheel(-.175), shooterSubsystem::stopFlywheel));  
// Auto pickup
    driveStick.rightBumper().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX((-driveStick.getLeftY()* MaxSpeed*(.6))) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.50)) // Drive left with negative X (left)
            .withRotationalRate(limelight_aim_proportional_intake() // Drive counterclockwise with negative X (left)
        )));  
   driveStick.rightBumper().whileTrue(
      new StartEndCommand(()-> intakeSubsystem.roll(.3), intakeSubsystem::rollStop));

   driveStick.rightBumper().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.spinBump(.45), shooterSubsystem::stopBump));       


    driveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveStick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveStick.getLeftY(), -driveStick.getLeftX()))));

    driveStick.leftBumper().whileTrue(

    //lime light aim speaker
    drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY()*MaxSpeed*(.4))
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.4))
            .withRotationalRate(limelight_aim_proportional())));
            
    //reset the field-centric heading on y button press
    driveStick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    //Op Xbox Controller
    
    opStick.x().whileTrue(
    new StartEndCommand(() -> shooterSubsystem.shootFlywheel(.335), shooterSubsystem::stopFlywheel));
       
    opStick.rightTrigger().whileTrue(
    new StartEndCommand(() -> shooterSubsystem.shootFlywheel(.25), shooterSubsystem::stopFlywheel));

    opStick.leftTrigger().whileTrue(
    new StartEndCommand(() -> shooterSubsystem.shootAmp(.35), shooterSubsystem::stopFlywheel));

    opStick.b().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(-.2), shooterSubsystem::stopBump));
      
    opStick.a().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(.45), shooterSubsystem::stopBump));

    opStick.y().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(-.5), shooterSubsystem::stopBump));

    opStick.y().whileTrue(
       new StartEndCommand(() -> intakeSubsystem.roll(-.5), intakeSubsystem::rollStop));

    
     
  //  opStick.rightTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbUp(.2), climberSubsystem::stopClimb));
  //   opStick.leftTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbDown(.2), climberSubsystem::stopClimb));

  }


  public RobotContainer() {
    configureBindings();
   
   autoChooserAuto.setDefaultOption("Source Two Piece", BottomTwoPiece);


   choiceWait.setDefaultOption("0 Seconds", Wait0);
   choiceWait.addOption("1 Seconds", Wait1);
   choiceWait.addOption("2 Seconds", Wait2);
   choiceWait.addOption("3 Seconds", Wait3);

   choiceWait.addOption("4 Seconds", Wait4);
   choiceWait.addOption("5 Seconds", Wait5);
   choiceWait.addOption("6 Seconds", Wait6);
   choiceWait.addOption("7 Seconds", Wait7);
   choiceWait.addOption("8 Seconds", Wait8);
   choiceWait.addOption("9 Seconds", Wait9);
   choiceWait.addOption("10 Seconds", Wait10);


   
   SmartDashboard.putData(choiceWait);
   SmartDashboard.putData(autoChooserAuto);


  // SmartDashboard.putData(autoChooserAuto);

  }

    public Command getAutonomousCommand() {
    
      


       return new SequentialCommandGroup(
      choiceWait.getSelected(),
      autoChooserAuto.getSelected()
      );
 
      

/* 

      return new SequentialCommandGroup(
           new ParallelCommandGroup(
              new PathPlannerAuto("Blue Shoot First"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(1.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Speaker to Top Close Blue"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.05),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Shoot 2 Blue")),
          new WaitCommand(1.2),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(1),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
          );

      */

     // return new PathPlannerAuto("Top Blue");
}
}