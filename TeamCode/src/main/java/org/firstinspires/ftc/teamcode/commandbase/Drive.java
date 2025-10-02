package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Drive extends SubsystemBase {

    private final Robot robot = Robot.getInstance();

    public void init() {
        setPTO(PTOState.DISENGAGED);
    }

    public enum PTOState {
        ENGAGED,
        DISENGAGED
    }

    public static PTOState ptoState = PTOState.DISENGAGED;

    public void setPTO(PTOState ptoState) {
        switch(ptoState) {
            case ENGAGED:
                robot.rightPTO.setPosition(RIGHT_PTO_ENGAGE);
                robot.leftPTO.setPosition(LEFT_PTO_ENGAGE);
                break;
            case DISENGAGED:
                robot.rightPTO.setPosition(RIGHT_PTO_DISENGAGE);
                robot.leftPTO.setPosition(LEFT_PTO_DISENGAGE);
                break;
        }

        Drive.ptoState = ptoState;
    }

}
