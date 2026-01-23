package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Translation2d;
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
    private XboxController controller;
    
    private final double kP=0.05;

    public PhotonAligner(Swerve s_Swerve, PhotonCamera camera, XboxController controller) {
        this.s_Swerve = s_Swerve;
        this.camera = camera;
        this.controller = controller;
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
        double forward = -controller.getLeftY() * Constants.Swerve.maxSpeed;
        double strafe = -controller.getLeftX() * Constants.Swerve.maxSpeed;
        double turn = -controller.getRightX() * Constants.Swerve.maxAngularVelocity;

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
                    }
                }
            }
        }
    

        // Auto-align when requested
        if (controller.getAButton() && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = -1.0 * targetYaw * kP * Constants.Swerve.maxAngularVelocity;
        }
        Translation2d translation= new Translation2d(forward,strafe);
        // Command drivetrain motors based on target speeds
        s_Swerve.drive(translation, turn, true, true);

        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
      
}
}
