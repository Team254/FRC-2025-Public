package com.team254.frc2025.subsystems.superstructure;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class CoralStateTracker {
    public enum CoralPosition {
        NONE,
        AT_FIRST_INDEXER,
        GOING_TO_SECOND_INDEXER,
        AT_SECOND_INDEXER,
        PROCESSING_IN_CLAW,
        STAGED_IN_CLAW
    }

    private static final double TIMEOUT_SECONDS = 0.5;

    private CoralPosition currentPosition = CoralPosition.NONE;
    private double lastTransitionTime = Timer.getFPGATimestamp();

    private boolean firstIndexerTriggered = false;
    private boolean secondIndexerTriggered = false;
    private boolean stageBannerTriggered = false;
    private boolean processingInClaw = false;

    public void updateFirstIndexer(boolean value) {
        firstIndexerTriggered = value;
        recalcState();
    }

    public void updateSecondIndexer(boolean value) {
        secondIndexerTriggered = value;
        recalcState();
    }

    public void updateStageBanner(boolean value) {
        stageBannerTriggered = value;
        recalcState();
    }

    public void updateProcessingInClaw(boolean value) {
        processingInClaw = value;
        recalcState();
    }

    private void recalcState() {
        double now = Timer.getFPGATimestamp();

        Logger.recordOutput("CoralStateTracker/lastTransitionTime", lastTransitionTime);

        switch (currentPosition) {
            case NONE:
                if (firstIndexerTriggered) {
                    currentPosition = CoralPosition.AT_FIRST_INDEXER;
                    lastTransitionTime = now;
                }
                if (secondIndexerTriggered) {
                    currentPosition = CoralPosition.AT_SECOND_INDEXER;
                    lastTransitionTime = now;
                }
                break;

            case AT_FIRST_INDEXER:
                if (firstIndexerTriggered) {
                    lastTransitionTime = now;
                } else {
                    currentPosition = CoralPosition.GOING_TO_SECOND_INDEXER;
                    lastTransitionTime = now;
                }
                if (secondIndexerTriggered) {
                    currentPosition = CoralPosition.AT_SECOND_INDEXER;
                    lastTransitionTime = now;
                }
                break;

            case GOING_TO_SECOND_INDEXER:
                if (secondIndexerTriggered) {
                    currentPosition = CoralPosition.AT_SECOND_INDEXER;
                    lastTransitionTime = now;
                } else if (firstIndexerTriggered) {
                    currentPosition = CoralPosition.AT_FIRST_INDEXER;
                    lastTransitionTime = now;
                } else if (stageBannerTriggered) {
                    currentPosition = CoralPosition.PROCESSING_IN_CLAW;
                    lastTransitionTime = now;
                } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
                    currentPosition = CoralPosition.NONE;
                }
                break;

            case AT_SECOND_INDEXER:
                if (secondIndexerTriggered) {
                    lastTransitionTime = now;
                } else {
                    if (stageBannerTriggered) {
                        currentPosition = CoralPosition.PROCESSING_IN_CLAW;
                        lastTransitionTime = now;
                    } else if (now - lastTransitionTime > TIMEOUT_SECONDS) {
                        currentPosition = CoralPosition.NONE;
                    }
                }
                break;

            case PROCESSING_IN_CLAW:
                if (!processingInClaw && stageBannerTriggered) {
                    currentPosition = CoralPosition.STAGED_IN_CLAW;
                    lastTransitionTime = now;
                } else if (now - lastTransitionTime > 1.5) {
                    currentPosition = CoralPosition.NONE;
                }
                break;

            case STAGED_IN_CLAW:
                if (!stageBannerTriggered && now - lastTransitionTime > TIMEOUT_SECONDS) {
                    currentPosition = CoralPosition.NONE;
                } else if (stageBannerTriggered) {
                    lastTransitionTime = now;
                }
                break;
        }
    }

    public CoralPosition getCurrentPosition() {
        return currentPosition;
    }

    public void forceSet(CoralPosition newState) {
        currentPosition = newState;
        lastTransitionTime = Timer.getFPGATimestamp();
    }
}
