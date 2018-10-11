package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "ModuleBot: Module Test", group= "ModuleBot")
public class ModularRobotInterativeTeleOp extends ModularRobotIterativeOpMode {
    private modularFTCBot Modulebot = new modularFTCBot();

    @Override
    public void init(){
        Modulebot.dt.init(hardwareMap);
    }

    @Override
    public void loop() {

    }

}
