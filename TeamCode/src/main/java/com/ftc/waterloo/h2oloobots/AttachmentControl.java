package com.ftc.waterloo.h2oloobots;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AttachmentControl {

    public static ColorSensor color;
    public static DistanceSensor distance;

    public void attachmentInit(HardwareMap hardwareMap) {

        color = hardwareMap.colorSensor.get("color");
        distance = (DistanceSensor) hardwareMap.get(DistanceSensor.class, "distance");

    }

}
