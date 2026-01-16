package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class FeedAssemblyCommand extends Command{
    private IntakeSubsystem i_Intake;
    private double speed, delay;
    public static Timer feedDelaytime = new Timer();


    public FeedAssemblyCommand(IntakeSubsystem i_Intake, double speed, double delay){
        this.i_Intake = i_Intake;
        addRequirements(i_Intake);

        this.speed = speed;
        this.delay = delay;
    }

    @Override
    public void initialize() {
        feedDelaytime.start();
    }

    @Override
    public void execute() {
        if (feedDelaytime.get() > delay){
            i_Intake.setFeedSpeed(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feedDelaytime.stop();
        feedDelaytime.reset();
        i_Intake.setFeedSpeed(0);
    }

    @Override
    public boolean isFinished() {
        if (feedDelaytime.get() > 4){
            return true;
        } else {
            return false;
        }
    }
}
