package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.Constants;

public class SwerveTrackingSubsystem extends SubsystemBase {
    // double angle;
    public double x ;
    public double id ;
    double y ;
    double area ;
    double delta ; 
  
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");
    NetworkTableEntry pipeline = table.getEntry("pipeline");
    public SwerveTrackingSubsystem(){
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
    }
    
    public Command AimAtSpeaker(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
        System.out.println(Constants.VisionConstants.SpeakerID);
        Command setPipelineCommand = this.run(
            () -> pipeline.setDouble(Constants.VisionConstants.SpeakerID)
            );
            setPipelineCommand.addRequirements(this);
            Command rotateSwerveCommand = new TeleopSwerve(
                s_Swerve,
                translationSup,
                strafeSup,
                () -> getRotation(0),
                robotCentricSup 
            );
        return setPipelineCommand.alongWith(rotateSwerveCommand);
    }

    public void setPipelineSpeaker(){
        pipeline.setDouble(Constants.VisionConstants.SpeakerID);
    }
    
    public double getDistance(){
        double area = ta.getDouble(0.0);
        double oneSide = Math.sqrt(area);
        double distance = 4.83/oneSide; 
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
        //adjusting for mounting angle offset
        targetAngle += 2;
        if (id <= 0){
            System.out.println("id is 0");
            return 0;
        }
        else{
            if (tx.getDouble(0.0) > (targetAngle+10)){
                return (tx.getDouble(0.0)-targetAngle)*-0.03;
            } else  if (tx.getDouble(0.0) > (targetAngle+5)){
                return (tx.getDouble(0.0)-targetAngle)*-0.04;
            } else {
                return (tx.getDouble(0.0)-targetAngle)*-0.05;
            }
        }
    }

        public double getRotationAuton(double targetAngle){
        // System.out.println("id: " + id);
        //adjusting for mounting angle offset
        targetAngle += 2;
        if (id <= 0){
            System.out.println("id is 0");
            return 0;
        }
        else{
            delta = tx.getDouble(0.0) - targetAngle;
            if (Math.abs(delta) > 10){
                return delta*-0.02;
            } else  if (Math.abs(delta) > 5){
                return delta*-0.035;
            } else {
                return delta*-0.04;
            }
        }
    }
}