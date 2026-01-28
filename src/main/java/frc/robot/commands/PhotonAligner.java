package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveTrackingSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
public class PhotonAligner extends Command {
    private Swerve s_Swerve;
    private PhotonCamera camera;
    private Joystick controller;
    
    private final double anglekP=0.4;
    private final double driveKP=0.6;
    private final double targetDistance=0.5;
    private double projectedDistance=0;
    private double projectedStraf=0;

    public PhotonAligner(Swerve s_Swerve, PhotonCamera camera, Joystick controller) {
        this.s_Swerve = s_Swerve;
        this.camera = camera;
        this.controller = controller;
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        double forward = -controller.getRawAxis(1) * Constants.Swerve.maxSpeed;
        double strafe = -controller.getRawAxis(0) * Constants.Swerve.maxSpeed;
        double turn = -controller.getRawAxis(4) * Constants.Swerve.maxAngularVelocity;

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 1) {
                        // Found Tag 1, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                        var translation = target.getBestCameraToTarget();
                        projectedDistance=translation.getX();
                        projectedStraf=translation.getY();
                    }
                }
            }
        }
    

        // Auto-align when requested
        if (targetVisible==true) {
            if(Math.abs(targetYaw)>5){
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = targetYaw * anglekP * Constants.Swerve.maxAngularVelocity;
        }
        double distanceError=targetDistance-projectedDistance;
        if(Math.abs(distanceError)>0.15){
        forward=driveKP*Constants.Swerve.maxSpeed*distanceError;
        }
        if(Math.abs(projectedStraf)>0.15){
        strafe=driveKP*Constants.Swerve.maxSpeed*projectedStraf;
        }
    }
        Translation2d translation= new Translation2d(-strafe,-forward);
        // Command drivetrain motors based on target speeds
    
        s_Swerve.drive(translation, turn, false, true);

        // Put debug information to the dashboards
        SmartDashboard.putNumber("Raw Target Yaw", targetYaw);
        SmartDashboard.putNumber("Photon Turn Value", turn);
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        SmartDashboard.putNumber("Photon Forward", forward);
        SmartDashboard.putNumber("Photon strafe", strafe);
        SmartDashboard.putNumber("Photon estimated distance", projectedDistance);
}
}
