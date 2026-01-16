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

public class ObjectTrackingSubsystem extends SubsystemBase {
    // double angle;
    double x ;
    double id ;
    double y ;
    double area ;
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
        static NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tid = table.getEntry("tid");
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        public ObjectTrackingSubsystem(){
        }
        @Override
        public void periodic() {
            x = tx.getDouble(0.0);
            id = tid.getDouble(0.0);
            y = ty.getDouble(0.0);
            area = ta.getDouble(0.0);
            SmartDashboard.putNumber("intake-distance", getDistance());
            SmartDashboard.putNumber("intake-limelightx", x);
            SmartDashboard.putNumber("intake-limelighty", y);
            SmartDashboard.putNumber("intake-limelighta", area);
            SmartDashboard.putNumber("intake-limelightid", id);
        }
        
        public Command AimAtSpeaker(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, BooleanSupplier robotCentricSup) {
            Command setPipelineCommand = this.run(
                () -> pipeline.setDouble(0)
                );
                setPipelineCommand.addRequirements(this);
                // angle = getRotation(offset);
                Command rotateSwerveCommand = new TeleopSwerve(
                    s_Swerve,
                    translationSup,
                    strafeSup,
                    () -> getRotation(0),
                    robotCentricSup 
                );
            return setPipelineCommand.andThen(rotateSwerveCommand);
        }
        
        public double getDistance(){
            double area = ta.getDouble(0.0);
            double oneSide = Math.sqrt(area);
            double distance = 5.37/oneSide;
            return distance;
        }
    
        public static double getRotation(double targetAngle){
                return (tx.getDouble(0.0)-targetAngle)*-(Constants.kObjectTrackingVal);
    }

    public static boolean hasObject(){
        NetworkTableEntry tv = table.getEntry("tv");
        return tv.getBoolean(false);
    }
}