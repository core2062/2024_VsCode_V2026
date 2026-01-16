package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TrackingLauncherAimCommand;
import frc.robot.constants.Constants;

public class LauncherTrackingSubsystem extends SubsystemBase {
    double x ;
    double id ;
    double y ;
    double area ;
  
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    public LauncherTrackingSubsystem(){
    }
    @Override
    public void periodic() {
        x = tx.getDouble(0.0);
        id = tid.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        SmartDashboard.putNumber("distance", getDistance());
        SmartDashboard.putNumber("limelightx", x);
        SmartDashboard.putNumber("limelighty", y);
        SmartDashboard.putNumber("limelighta", area);
        SmartDashboard.putNumber("limelightid", id);
        if (getDistance() > 7){
            if (Constants.VisionConstants.SpeakerID == 4){
                Constants.VisionConstants.SpeakerID = 1;
            } else if (Constants.VisionConstants.SpeakerID == 7) {
                Constants.VisionConstants.SpeakerID = 2;
            }
        } else if (getDistance() < 7){
            if (Constants.VisionConstants.SpeakerID == 1){
                Constants.VisionConstants.SpeakerID = 4;
            } else  if (Constants.VisionConstants.SpeakerID == 2) {
                Constants.VisionConstants.SpeakerID = 7;
            }

        }
        Constants.VisionConstants.DesiredAngle = findAngle();
        SmartDashboard.putNumber("Desired Angle of Launcher: ", Constants.VisionConstants.DesiredAngle);
    }

    public Command TargetCommand(LauncherSubsystem l_Launcher){
        Command setPipelineCommand = this.run(
            () -> pipeline.setDouble(Constants.VisionConstants.SpeakerID)
            );
        setPipelineCommand.addRequirements(this);
        Command setAngle = new TrackingLauncherAimCommand(l_Launcher, this);
        return setPipelineCommand.alongWith(setAngle);
    }
    
    public double getDistance(){
        double area = ta.getDouble(0.0);
        double oneSide = Math.sqrt(area);
        double distance = 4.83/oneSide; //TODO: adjust value for apriltag tracking
        return distance;
    }

    public double getTranslation(double targetDistance){
        double distance = getDistance();
        if (Double.isInfinite(distance)){
            return 0;
        }
        else{
            return targetDistance-distance;
        }
    }

    public double getRotation(double targetAngle){
        // System.out.println("id: " + id);
        if (id <= 0){
            System.out.println("id is 0");
            return 0;
        }
        else{
            if (tx.getDouble(0.0) > (targetAngle+10)){
                return (tx.getDouble(0.0)-targetAngle)*-0.03;
            } else {
                return (tx.getDouble(0.0)-targetAngle)*-0.04;
            }
        }
    }

    public double findAngle(){
        //formula calulated from dcode.fr/function-equation-finder and data points collected
        double Dist = ((0.985078 * (getDistance() * 12)) + 11.3942) + 19.5;
        // double Dist = (getDistance() * 12) + 19.5;
        // System.out.println(Dist);

        //height Offset 
        double floorToPivot = 15.49;

        double targetHeight = 78 - floorToPivot;
        double apriltagHeight = 57.125 - floorToPivot;

        double xDist = Math.sqrt(Math.pow(Dist, 2) - Math.pow(apriltagHeight, 2));
        // System.out.println("xDist: " + xDist);
        double targetAngle = Math.toDegrees(Math.atan2(targetHeight, xDist));
        double desiredAngle = ((1.0899 * targetAngle) + 2.3724);
        if (desiredAngle < 19){
            desiredAngle += 4;
        } else if (desiredAngle < 21){
            desiredAngle += 2;
        } else if (desiredAngle < 24){
            desiredAngle += 3;
        } else if (desiredAngle > 25.5){
            desiredAngle += 2;
        }
        return desiredAngle;
    }
}