package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoreAssembly;

public class IntakeAssemblyCommand extends Command{
    private IntakeSubsystem i_Intake;
    private double intakeSpeed, feedSpeed;

    public IntakeAssemblyCommand(IntakeSubsystem i_Intake, double intakeSpeed, double feedSpeed){
        this.i_Intake = i_Intake;
        addRequirements(i_Intake);

        this.intakeSpeed = intakeSpeed;
        this.feedSpeed = feedSpeed;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Auton State", "Beginging Intake");
    }

    @Override
    public void execute() {
        i_Intake.setFeedAndIntakeSpeed(intakeSpeed, feedSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        i_Intake.setFeedAndIntakeSpeed(0, 0);
        SmartDashboard.putString("Auton State", "Intake Complete");
    }

    @Override
    public boolean isFinished() {
        boolean finished = ScoreAssembly.getPhotoeye();
        finished = !finished;
        if (Constants.endAssembly1 == true){
            return true;
        } else {
            return finished;
        }
    }
}
