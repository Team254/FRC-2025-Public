package com.team254.lib.reefscape;

public enum ReefBranch {
    FEEDER,
    FEEDER_GROUND,
    FEEDER_GROUND_RETRY,
    BARGE,
    AB('A', 'B', 18, 7),
    CD('C', 'D', 17, 8),
    EF('E', 'F', 22, 9),
    GH('G', 'H', 21, 10),
    IJ('I', 'J', 20, 11),
    KL('K', 'L', 19, 6);

    private final char leftLetter;
    private final char rightLetter;
    private final int tagIdBlue;
    private final int tagIdRed;

    ReefBranch(char leftLetter, char rightLetter, int tagIdBlue, int tagIdRed) {
        this.leftLetter = leftLetter;
        this.rightLetter = rightLetter;
        this.tagIdBlue = tagIdBlue;
        this.tagIdRed = tagIdRed;
    }

    ReefBranch() {
        this.leftLetter = '\0';
        this.rightLetter = '\0';
        this.tagIdBlue = -1;
        this.tagIdRed = -1;
    }

    public char getLeftLetter() {
        return leftLetter;
    }

    public char getRightLetter() {
        return rightLetter;
    }

    public int getBlueTagId() {
        return this.tagIdBlue;
    }

    public int getRedTagId() {
        return this.tagIdRed;
    }
}
