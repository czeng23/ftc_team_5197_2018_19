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

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.Round_1_Op.Pos.LEFT;
import static org.firstinspires.ftc.teamcode.Round_1_Op.Pos.MID;
import static org.firstinspires.ftc.teamcode.Round_1_Op.Pos.RIGHT;
import static org.firstinspires.ftc.teamcode.Round_1_Op.Pos.UNKNOWN;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Round 1 Operation", group="Linear Opmode")
//@Disabled
public class Round_1_Op extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private GoldAlignDetector locator = null;
    private boolean visible = false;
    private double x = 0.0;
    final static int MIDPOINT = 312;  // screen midpoint
    final static int LEFTPOINT = -104;
    final static int RIGHTPOINT = 104;
    private boolean done = false;

    public enum Pos {
        LEFT, MID, RIGHT, UNKNOWN
    }

    @Override
    public void runOpMode() {

        Pos pos = Pos.MID;
        String text = "??";


        // Init Detector
        locator = new GoldAlignDetector();
        locator.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        locator.useDefaults();

        // Optional Tuning
        locator.alignSize = 610; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        locator.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        locator.downscale = 0.4; // How much to downscale the input frames

        locator.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        locator.maxAreaScorer.weight = 0.005;

        locator.ratioScorer.weight = 5;
        locator.ratioScorer.perfectRatio = 1.0;
        locator.enable();

        telemetry.addData("locator", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {
        while (!done) {
            visible = locator.isFound();
            x = locator.getXPosition() - MIDPOINT;

            if(visible) {
                if (x < LEFTPOINT)
                    pos = Pos.LEFT;
                else if ((x >= LEFTPOINT) && (x <= RIGHTPOINT))
                    pos = Pos.MID;
                else if (x >= 400)
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
                    break;


            }
            telemetry.addData("IsFound" ,visible); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos" , x); // Gold X pos.
            telemetry.addData("Pos" , text); // Gold X pos.

            telemetry.update();// Gold X pos.
        }

        telemetry.addData("Status" ,"All Done"); // Is the bot aligned with the gold mineral
        telemetry.update();// Gold X pos.
    }

    private void targetLeft()  {

        try {

            //handle right hand side target
            Thread.sleep(3000);
            done = true;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void targetRight() {

        try {

            //handle left hand side target
            Thread.sleep(3000);
            done = true;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private void targetCenter() {
        //handle center target
        try {
            Thread.sleep(3000);
            done = true;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
