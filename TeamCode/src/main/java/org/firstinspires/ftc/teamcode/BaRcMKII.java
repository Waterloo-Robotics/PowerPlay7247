package org.firstinspires.ftc.teamcode;

import com.ftc.waterloo.h2oloobots.OdometryControl;
import com.ftc.waterloo.h2oloobots.TelemetryControl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BaRcMKII extends LinearOpMode {

    public void runOpMode() {

        TelemetryControl telemetryControl = new TelemetryControl(telemetry);
        OdometryControl odometryControl = new OdometryControl(hardwareMap, telemetryControl);

        waitForStart();

//        odometryControl.back(3, 0.125);
//        odometryControl.strafeLeft(51, 0.125);
        odometryControl.turn(-1, 7);
//        odometryControl.forward(9, 0.125);


    }

}
