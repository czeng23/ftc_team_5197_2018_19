package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="REVTrixbot: V1(only mineral detect)", group ="REVTrixbot")
public class REVTrixbotAutonomousV1 extends ModularRobotIterativeTeleOp {
    private REVTrixbot robot = new REVTrixbot();

    public void init(){
        robot.dt.init(hardwareMap);
        robot.goldLocator.init(hardwareMap);
    }

    @Override
    public void loop() {
        switch (robot.goldLocator.getGoldPos()){
            case LEFT:
                //code to turn left
                break;
            case RIGHT:
                //code
                break;
            case MID:
                //code
                break;
            default:
                //code for out of frame
                break;

        }



    }
}
