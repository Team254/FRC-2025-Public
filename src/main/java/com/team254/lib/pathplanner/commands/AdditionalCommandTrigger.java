package com.team254.lib.pathplanner.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A command that monitors a running PathfindingCommand and, when its remaining time falls below a
 * specified threshold, schedules an additional command.
 */
public class AdditionalCommandTrigger extends Command {
    private final PathfindingCommand pathfindingCommand;
    private final Command additionalCommand;
    private final double triggerTimeBeforeEnd;
    private boolean triggered = false;

    public AdditionalCommandTrigger(
            PathfindingCommand pathfindingCommand,
            Command additionalCommand,
            double triggerTimeBeforeEnd) {
        this.pathfindingCommand = pathfindingCommand;
        this.additionalCommand = additionalCommand;
        this.triggerTimeBeforeEnd = triggerTimeBeforeEnd;
    }

    @Override
    public void execute() {
        if (!triggered && pathfindingCommand.getRemainingTime() <= triggerTimeBeforeEnd) {
            CommandScheduler.getInstance().schedule(additionalCommand);
            triggered = true;
        }
    }

    @Override
    public boolean isFinished() {
        return triggered;
    }
}
