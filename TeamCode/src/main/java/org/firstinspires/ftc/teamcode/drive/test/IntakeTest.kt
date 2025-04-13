package org.firstinspires.ftc.teamcode.drive.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PwmControl

@TeleOp(group = "test")
class IntakeTest : OpMode() {
    private lateinit var left: CRServoImplEx
    private lateinit var right: CRServoImplEx
    override fun init() {
        left = hardwareMap.get(CRServoImplEx::class.java, "left")
        right = hardwareMap.get(CRServoImplEx::class.java, "right")

        left.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        right.direction = DcMotorSimple.Direction.REVERSE

    }

    override fun loop() {
        //setSpeed(-gamepad1.left_stick_y.toDouble())
        setSpeed(1.0)

    }

    fun setSpeed(speed: Double) {
        left.power = speed
        right.power = speed
    }

}