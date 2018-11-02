package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * An opmode to callibrate a REVTrix bot
 *
 * Version history
 * ======= ========
 * v 0.1    11/02/18 @Lorenzo Pedroza. Added this Javadoc
 */

@Autonomous(name="Calibrate", group="REVTrixbot")
public class REVTrix_Callibrate_Iterative extends ModularRobotIterativeOpMode {

    REVTrixbot robot = new REVTrixbot();

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.2;

    @Override
    public void init() {
        robot.dt.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.dt.encoderDrive(DRIVE_SPEED, 48, 48);
    }
}
