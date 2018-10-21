package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="REVTrixbot: V1(only mineral detect)", group ="REVTrixbot")
public class REVTrixbotAutonomousV1 extends ModularRobotIterativeTeleOp {
    private REVTrixbot robot = new REVTrixbot();

    public void init(){
        robot.dt.init(hardwareMap);
        robot.goldAlignmentDetector.init();
    }

    @Override
    public void loop() {

    }
}
