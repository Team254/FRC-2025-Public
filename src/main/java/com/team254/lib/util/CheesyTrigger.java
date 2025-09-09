package com.team254.lib.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class CheesyTrigger extends Trigger {
    // Do this because Trigger has a bunch of private stuff.
    protected final BooleanSupplier m_condition;
    protected final EventLoop m_loop;
    protected boolean shouldBeRunning = false;

    public CheesyTrigger(EventLoop loop, BooleanSupplier condition) {
        super(loop, condition);
        m_loop = requireNonNullParam(loop, "loop", "Trigger");
        m_condition = requireNonNullParam(condition, "condition", "Trigger");
    }

    public CheesyTrigger(BooleanSupplier condition) {
        this(CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
    }

    /** Functional interface for the body of a trigger binding. */
    @FunctionalInterface
    protected interface BindingBody {
        /**
         * Executes the body of the binding.
         *
         * @param previous The previous state of the condition.
         * @param current The current state of the condition.
         */
        void run(boolean previous, boolean current);
    }

    /**
     * Adds a binding to the EventLoop.
     *
     * @param body The body of the binding to add.
     */
    protected void addBinding(CheesyTrigger.BindingBody body) {
        m_loop.bind(
                new Runnable() {
                    private boolean m_previous = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean current = m_condition.getAsBoolean();

                        body.run(m_previous, current);

                        m_previous = current;
                    }
                });
    }

    // Similar to whileTrue, but always start the command (that is keep trying).
    public Trigger whileTrueAlwaysRunning(Command command) {
        requireNonNullParam(command, "command", "whileTrueAlwaysStart");
        addBinding(
                (previous, current) -> {
                    if (!previous && current) {
                        command.schedule();
                        shouldBeRunning = true;
                    } else if (previous && !current) {
                        command.cancel();
                        shouldBeRunning = false;
                    }
                    if (current && shouldBeRunning && !command.isScheduled()) {
                        // Try again!
                        command.schedule();
                    }
                });
        return this;
    }
}
