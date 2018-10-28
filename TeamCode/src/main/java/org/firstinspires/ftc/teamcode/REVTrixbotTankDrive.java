/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Teleop driving for a REVTrixbot. It is modified and simplified
 * from the FtcRobotController external sample PushbotTeleopTankIterative. The code is structured
 * as an Iterative OpMode. The arm and claw operation in the sample is not implemented here.
 *
 * Tank drive means the gamepad left stick controls the left motors, and the right stick controls
 * the right motors.
 *
 * This OpMode uses the REVTrixbot hardware class to define the devices on the robot.
 * All device access is managed through the REVTrix class.
 *
 * Version history
 * ======= ======
 * v 0.1    10/11/18 jmr primitive version, just enough to test the drive train. Does not extend
 *          GenericRobot class.
 *
 */

@TeleOp(name="REVTrixbot: Teleop Tank", group="REVTrixbot")
//@Disabled
public class REVTrixbotTankDrive extends ModularRobotIterativeTeleOp {

    /* Declare OpMode members. */
    private REVTrixbot robot       = new REVTrixbot();  // Class created to define a REVTrixbot's hardware
    private GoldMineralDetector locater = null;

    Pos pos = Pos.MID;
    String text = "??";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    private boolean visible = false;
    private double x = 0.0;
    private final static int MIDPOINT = 320;  // screen midpoint
    private final static int LEFTPOINT = -106;
    private final static int RIGHTPOINT = 106;
    public enum Pos {
        LEFT, MID, RIGHT, UNKNOWN
    }

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.dt.init(hardwareMap);


        locater = new GoldMineralDetector();
        locater.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        locater.useDefaults();

        // Optional Tuning
        locater.alignSize = 640; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        locater.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        locater.downscale = 0.4; // How much to downscale the input frames

        locater.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        locater.maxAreaScorer.weight = 0.005;

        locater.ratioScorer.weight = 5;
        locater.ratioScorer.perfectRatio = 1.0;



        // Send telemetry message to signify robot waiting;

        telemetry.addData("locator", "Initialized");
        telemetry.update();

        telemetry.addData("Say", "Hello, Driver!");

        locater.enable();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        robot.dt.teleOpTankDrive(gamepad1);

        telemetry.addData("left",  "%.2f", -gamepad1.left_stick_y);
        telemetry.addData("right", "%.2f", -gamepad1.right_stick_y);
        telemetry.addData("Rotations", robot.dt.getAverageDTRotation(true));

        visible = locater.isFound();
        x = locater.getXPosition() - MIDPOINT;

        if(visible) {
            if (x < LEFTPOINT)
                pos = Pos.LEFT;
            else if ((x >= LEFTPOINT) && (x <= RIGHTPOINT))
                pos = Pos. MID;
            else if (x >= RIGHTPOINT)
                pos = Pos.RIGHT;
        }   else {
            pos = Pos.UNKNOWN;
        }

        switch (pos) {
            case LEFT:
                //do left thing
                text = "LEFT";
                targetLeft();

                break;

            case RIGHT:
                //do left thing
                text = "RIGHT";
                targetRight();

                break;

            case MID:
                //do left thing
                text = "Mid";
                targetCenter();
                break;

            default:
                text = "Unknown";
                targetUnknown();
                break;

        }
        telemetry.addData("IsFound" ,visible); // Is the bot aligned with the gold mineral
        telemetry.addData("X Pos" , x); // Gold X pos.
        telemetry.addData("Pos" , text); // Gold X pos.

        telemetry.update();// Gold X pos.

        telemetry.addData("Status" ,"All Done"); // Is the bot aligned with the gold mineral
        telemetry.update();// Gold X pos.
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void targetLeft()  {
        // Lookeebot_4W_TankDrive
        //stop();
    }

    private void targetRight() {
        // Lookeebot_4W_TankDrive
        //stop();
    }

    private void targetCenter() {
        // Lookeebot_4W_TankDrive
        //stop();
    }

    private void targetUnknown() {
        // Lookeebot_4W_TankDrive
        //stop();
    }
}
