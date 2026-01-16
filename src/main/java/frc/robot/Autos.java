package frc.robot;

import frc.robot.commands.AutonAlignmentCommand;
import frc.robot.commands.AutonShootCommand;
import frc.robot.commands.FeedAssemblyCommand;
import frc.robot.commands.IntakeAssemblyCommand;
import frc.robot.commands.LauncherAimCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TrackingLauncherAimCommand;
import frc.robot.commands.ZeroLauncherCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;
import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Autos extends SequentialCommandGroup {    

    public Autos(String side, int cases, Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem, SwerveTrackingSubsystem st_SwerveTrackingSubsystem, LauncherTrackingSubsystem lt_LauncherTrackingSubsystem){
        System.out.printf("autos selection: %d\n", cases);
        System.out.println("Selecte Side: " + side);
        switch(side) {
            case "red":
                switch (cases) {
                    case 0:
                        MidAuto(s_Swerve, i_Intake, l_LauncherSubsystem);
                        break;
                    case 1:
                        RedLeftAuto(s_Swerve, i_Intake, l_LauncherSubsystem, st_SwerveTrackingSubsystem, lt_LauncherTrackingSubsystem);
                        break;
                    case 2:
                        RedRightPickupAuto(s_Swerve, i_Intake, l_LauncherSubsystem, st_SwerveTrackingSubsystem, lt_LauncherTrackingSubsystem);
                        break;
                    case 3:
                        RedRightMovementAuto(s_Swerve, i_Intake, l_LauncherSubsystem);
                        break;
                    default:
                        doNothingAuto(s_Swerve);
                        break;
                }
                break;
            case "blue":
                switch (cases) {
                    case 0:
                        MidAuto(s_Swerve, i_Intake, l_LauncherSubsystem);
                        break;
                    case 1:
                        BlueRightAuto(s_Swerve, i_Intake, l_LauncherSubsystem, st_SwerveTrackingSubsystem, lt_LauncherTrackingSubsystem);
                        break;
                    case 2:
                        BlueLeftPickupAuto(s_Swerve, i_Intake, l_LauncherSubsystem, st_SwerveTrackingSubsystem, lt_LauncherTrackingSubsystem);
                        break;
                    case 3:
                        BlueLeftMovementAuto(s_Swerve, i_Intake, l_LauncherSubsystem);
                        break;
                    default:
                        doNothingAuto(s_Swerve);
                        break;
                }
                break;
            default:
                doNothingAuto(s_Swerve);
                break;
        }
    }

    public void doNothingAuto(Swerve s_Swerve) {
        String trajectoryJSON = "paths/MoveBackPickup.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
    
    public void MidAuto(Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem) {
        System.out.println("move auto");  

        String trajectoryJSON = "paths/MoveBackPickup.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
            
        addCommands(
            new ZeroLauncherCommand(l_LauncherSubsystem),
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 50),
            new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 33.5),
            new InstantCommand(() -> i_Intake.setFeedAndIntakeSpeed(0.5, 0.5)),
            swerveControllerCommand,
            new TeleopSwerve(s_Swerve, () -> 0, () -> 0, () -> 0, () -> false).raceWith(new IntakeAssemblyCommand(i_Intake, 0.5, 0.5)),
            new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4)
        );
    }


//----- Red Allience Autons -----

    public void RedLeftAuto(Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem, SwerveTrackingSubsystem t_Tracking, LauncherTrackingSubsystem lt_TrackingSubsystem) {
        System.out.println("Red Allience Left Speaker Auto");  

        String trajectoryJSON = "paths/RedAlliLeft.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
            // An example trajectory to follow.  All units in meters.
            Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
            );



            // String trajectoryJSON2 = "paths/RedAmpMidNote.wpilib.json";
            // Trajectory trajectory2 = new Trajectory(); 
            // try {
            //     Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
            //     trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
            //     System.out.println("Path " + trajectoryPath2);
            // } catch (IOException ex) {
            //     DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON2, ex.getStackTrace());
            //     };
                // An example trajectory to follow.  All units in meters.
                // Trajectory Trajectory2 = trajectory2;
            
            var thetaController2 =
            new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController2.enableContinuousInput(-Math.PI, Math.PI);
            
            // SwerveControllerCommand swerveControllerCommand2 =
            // new SwerveControllerCommand(
            //     Trajectory2,
            //     s_Swerve::getPose,
            //     Constants.Swerve.swerveKinematics,
            //     new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            //     new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            //     thetaController2,
            //     s_Swerve::setModuleStates,
            //     s_Swerve
            //     );
            
            addCommands(
                new ZeroLauncherCommand(l_LauncherSubsystem),
                new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
                new LauncherAimCommand(l_LauncherSubsystem, () -> 50),
                new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4),
                new LauncherAimCommand(l_LauncherSubsystem, () -> 33.5),
                new InstantCommand(() -> i_Intake.setFeedAndIntakeSpeed(0.5, 0.5)),
                swerveControllerCommand,
                new IntakeAssemblyCommand(i_Intake, 0.5, 0.5).raceWith(new TeleopSwerve(s_Swerve, () -> 0, () -> 0, () -> 0, () -> false)),
                new AutonAlignmentCommand(t_Tracking, s_Swerve, -1),
                new TrackingLauncherAimCommand(l_LauncherSubsystem, lt_TrackingSubsystem).raceWith(new FeedAssemblyCommand(i_Intake, 0.5, 2))
                // new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
                // swerveControllerCommand2
        );
    }

    public void RedRightPickupAuto(Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem, SwerveTrackingSubsystem st_TrackingSubsystem, LauncherTrackingSubsystem lt_TrackingSubsystem) {
        System.out.println("Red Allience Right Speaker Note Pickup Auto");  

        String trajectoryJSON = "paths/RedAlliRightNotePickup.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
            
        addCommands(
            new InstantCommand(() -> st_TrackingSubsystem.setPipelineSpeaker()),
            new ZeroLauncherCommand(l_LauncherSubsystem),
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 50),
            new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 33.5),
            new InstantCommand(() -> i_Intake.setFeedAndIntakeSpeed(0.8, 0.65)),
            swerveControllerCommand,
            new IntakeAssemblyCommand(i_Intake, 0.5, 0.5),
            new AutonAlignmentCommand(st_TrackingSubsystem, s_Swerve, 1),
            new TrackingLauncherAimCommand(l_LauncherSubsystem, lt_TrackingSubsystem).raceWith(new FeedAssemblyCommand(i_Intake, 0.5, 2))
        );
    }

    public void RedRightMovementAuto(Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem) {
        System.out.println("Red Allience Right Speaker Move Auto");  

        String trajectoryJSON = "paths/RedAlliRightMove.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
            
        addCommands(
            new ZeroLauncherCommand(l_LauncherSubsystem),
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 50),
            new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 33.5),
            swerveControllerCommand
        );
    }

    //----- Blue Alliance Autons

    public void BlueRightAuto(Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem, SwerveTrackingSubsystem t_Tracking, LauncherTrackingSubsystem lt_TrackingSubsystem) {
        System.out.println("Blue Allience Right Speaker Auto");  

        String trajectoryJSON = "paths/BlueAlliRight.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
            
        addCommands(
            new ZeroLauncherCommand(l_LauncherSubsystem),
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 50),
            new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 33.5),
            new InstantCommand(() -> i_Intake.setFeedAndIntakeSpeed(0.5, 0.5)),
            swerveControllerCommand,
            new IntakeAssemblyCommand(i_Intake, 0.5, 0.5).raceWith(new TeleopSwerve(s_Swerve, () -> 0, () -> 0, () -> 0, () -> false)),
            new AutonAlignmentCommand(t_Tracking, s_Swerve, 1),
            new TrackingLauncherAimCommand(l_LauncherSubsystem, lt_TrackingSubsystem).raceWith(new FeedAssemblyCommand(i_Intake, 0.5, 2))
        );
    }

    public void BlueLeftPickupAuto(Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem, SwerveTrackingSubsystem t_Tracking, LauncherTrackingSubsystem lt_TrackingSubsystem) {
        System.out.println("Blue Allience Left Speaker Note Pickup Auto");  

        String trajectoryJSON = "paths/BlueAlliLeftNotePickup.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
            
        addCommands(
            new InstantCommand(() -> t_Tracking.setPipelineSpeaker()),
            new ZeroLauncherCommand(l_LauncherSubsystem),
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 50),
            new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 33.5),
            new InstantCommand(() -> i_Intake.setFeedAndIntakeSpeed(0.8, 0.65)),
            swerveControllerCommand,
            new IntakeAssemblyCommand(i_Intake, 0.5, 0.5),
            new AutonAlignmentCommand(t_Tracking, s_Swerve, -1),
            new TrackingLauncherAimCommand(l_LauncherSubsystem, lt_TrackingSubsystem).raceWith(new FeedAssemblyCommand(i_Intake, 0.5, 2))
        );
    }

    public void BlueLeftMovementAuto(Swerve s_Swerve, IntakeSubsystem i_Intake, LauncherSubsystem l_LauncherSubsystem) {
        System.out.println("Blue Allience Left Speaker Move Auto");  

        String trajectoryJSON = "paths/BlueAlliLeftMove.wpilib.json";
        Trajectory trajectory = new Trajectory(); 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
        
        var thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            Trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve
        );
            
        addCommands(
            new ZeroLauncherCommand(l_LauncherSubsystem),
            new InstantCommand(() -> s_Swerve.setPose(Trajectory.getInitialPose())),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 50),
            new AutonShootCommand(i_Intake, l_LauncherSubsystem, 0.5, 0.6, 0.4),
            new LauncherAimCommand(l_LauncherSubsystem, () -> 33.5),
            swerveControllerCommand
        );
    }

    
}