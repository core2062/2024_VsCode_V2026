package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class AutonShootCommand extends Command{
    private IntakeSubsystem i_Intake;
    private LauncherSubsystem l_Launcher;
    private double launcherSpeed, feedSpeed, delay;
    public static Timer feedDelaytime = new Timer();


    public AutonShootCommand(IntakeSubsystem i_Intake, LauncherSubsystem l_Laucnher, double feedSpeed, double launcherSpeed, double delay){
        this.i_Intake = i_Intake;
        this.l_Launcher = l_Laucnher;
        addRequirements(i_Intake, l_Laucnher);

        this.feedSpeed = feedSpeed;
        this.launcherSpeed = launcherSpeed;
        this.delay = delay;
    }

    @Override
    public void initialize() {
        feedDelaytime.start();
        SmartDashboard.putString("Auton State", "Beginging fireing");
    }

    @Override
    public void execute() {
        l_Launcher.setLauncherSpeed(launcherSpeed);
        if (feedDelaytime.get() > delay){
            i_Intake.setFeedSpeed(feedSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feedDelaytime.stop();
        feedDelaytime.reset();
        i_Intake.setFeedSpeed(0);
        l_Launcher.setLauncherSpeed(0);
        SmartDashboard.putString("Auton State", "Firering Complete");
    }

    @Override
    public boolean isFinished() {
        if (feedDelaytime.get() > 1){
            return true;
        } else {
            return false;
        }
    }
}