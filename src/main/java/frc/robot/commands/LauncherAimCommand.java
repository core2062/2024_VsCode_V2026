package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LauncherAimCommand extends Command{
    private LauncherSubsystem l_Launcher;
    private double DifferenceOfAngle;
    private DoubleSupplier desiredAngle;

    public LauncherAimCommand(LauncherSubsystem l_Launcher, DoubleSupplier desiredAngle){
        this.l_Launcher = l_Launcher;
        addRequirements(l_Launcher);

        this.desiredAngle = desiredAngle;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Auton State", "Begining Aimming");
    }

    @Override
    public void execute() {
        // System.out.println(desiredAngle.getAsDouble());
        double currentAngle = l_Launcher.getLeftEncoderValue();
        final double MAX_SPEED_RPM = 3; // Maximum speed of the motor in RPM
        final double ANGLE_TOLERANCE = 1.0;
        // Calculate the angle difference
        double angleDifference = desiredAngle.getAsDouble() - currentAngle;
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
        } else if (Math.abs(angleDifference) > 0.5 && Math.abs(speed) < 0.3){
            if (speed < 0) {
                speed = -0.3;
            } else if (speed > 0){
                speed = 0.3;
            }
        }
        l_Launcher.LauncherRotationAngle(speed);
        SmartDashboard.putNumber("Desired Launcher Rotation Speed: ", speed);
    }

    @Override
    public void end(boolean interrupted) {
        l_Launcher.LauncherRotationAngle(0);
        SmartDashboard.putString("Auton State", "Aimming Complete");
    }

    @Override
    public boolean isFinished(){
        if (Math.abs(DifferenceOfAngle) <= 0.5){
            return true;
        } else{
            return false;
        }
    }
}
