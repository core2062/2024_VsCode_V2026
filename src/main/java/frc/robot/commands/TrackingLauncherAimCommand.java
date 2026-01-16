package frc.robot.commands;

import frc.robot.constants.Constants;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherTrackingSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TrackingLauncherAimCommand extends Command{
    private LauncherSubsystem l_Launcher;
    private LauncherTrackingSubsystem lt_LaunchTrack;
    private double DifferenceOfAngle;
    private double lastAngle = 0;

    public TrackingLauncherAimCommand(LauncherSubsystem l_Launcher,LauncherTrackingSubsystem lt_LaunchTrack){
        this.l_Launcher = l_Launcher;
        addRequirements(l_Launcher);

        this.lt_LaunchTrack = lt_LaunchTrack;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Auton State", "Beginging tracking");
    }

    @Override
    public void execute() {
        double currentAngle = l_Launcher.getLeftEncoderValue();
        final double MAX_SPEED_RPM = 3; // Maximum speed of the motor in RPM
        final double ANGLE_TOLERANCE = 0.5;
        // Calculate the angle difference
        if (Constants.VisionConstants.DesiredAngle < 10.3724){
            Constants.VisionConstants.DesiredAngle = lastAngle;
        } else {
            lastAngle = Constants.VisionConstants.DesiredAngle;
        }
        double angleDifference = Constants.VisionConstants.DesiredAngle - currentAngle;
        DifferenceOfAngle = angleDifference;
        // Calculate the speed based on the angle difference
        double speedPercentage = angleDifference / 180.0; // Scaling the angle difference to [-1, 1]
        double speed = (speedPercentage * MAX_SPEED_RPM);
        
        // Ensure the speed is within the motor's range
        speed = Math.min(MAX_SPEED_RPM, Math.max(-MAX_SPEED_RPM, speed));
        
        
        // If the angle difference is within the tolerance, stop the motor
        if (Math.abs(angleDifference) <= ANGLE_TOLERANCE) {
            speed = 0; // Stop the motor
        } else if (Math.abs(angleDifference) > 10 && Math.abs(speed) < 0.5){
            if (speed < 0) {
                speed = -0.5;
            } else if (speed > 0){
                speed = 0.5;
            }
        } else if (Math.abs(angleDifference) > 1 && Math.abs(speed) < 0.3){
            if (speed < 0) {
                speed = -0.17;
            } else if (speed > 0){
                speed = 0.17;
            }
        } else if (Math.abs(angleDifference) > ANGLE_TOLERANCE && Math.abs(speed) < 0.2){
            if (speed < 0) {
                speed = -0.1;
            } else if (speed > 0){
                speed = 0.1;
            }
        }

        double changeINSpeed = 1.21038 - (0.0191341 * Constants.VisionConstants.DesiredAngle);

        double Dist = ((0.985078 * (lt_LaunchTrack.getDistance() * 12)) + 11.3942) + 18.5;
        double floorToPivot = 15.49;
        double apriltagHeight = 57.125 - floorToPivot;
        double xDist = Math.sqrt(Math.pow(Dist, 2) - Math.pow(apriltagHeight, 2));

        if (xDist > 220) {
            changeINSpeed += (changeINSpeed * 0.105);
        }else if (xDist > 190){
            changeINSpeed += (changeINSpeed * 0.048);
        } else if (xDist > 96){
            changeINSpeed += (changeINSpeed * 0.043);
        } else if (xDist < 36) {
            changeINSpeed -= (changeINSpeed * 0.1);
        }
        Constants.LauncherConstants.kSpeakerLaunchSpeed.set(changeINSpeed); 
        SmartDashboard.putNumber("change in speed: ", changeINSpeed);
        l_Launcher.LauncherRotationAngle(speed);
        l_Launcher.setLauncherSpeed(changeINSpeed);
        SmartDashboard.putNumber("Desired Launcher Rotation Speed: ", speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Command Done");
        l_Launcher.LauncherRotationAngle(0);
        l_Launcher.setLauncherSpeed(0.0);
        Constants.LauncherConstants.kSpeakerLaunchSpeed.set(0.6);
        SmartDashboard.putString("Auton State", "Tracking Complete");
    }

    @Override
    public boolean isFinished(){
        if (Constants.AimDone == true){
            return true;
        } else {
            return false;
        }
    }
}

