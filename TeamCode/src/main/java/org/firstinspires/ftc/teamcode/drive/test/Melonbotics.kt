package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.CRServoImpl
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import kotlin.math.abs

@Disabled
@TeleOp(group = "test")
class Melonbotics: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var gamepad: GamepadEx
    private lateinit var servo: CRServoImplEx
    private lateinit var servo1: CRServoImplEx

    private var lastPos: Double = Double.NaN
    private var requested: Double = HorizontalConstants.IntakeSpeeds.STOP
    private var count = 0

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        //voltage = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()

        gamepad = GamepadEx(gamepad1)

        servo = hardwareMap.get(CRServoImplEx::class.java, "intakeLeft")
        servo1 = hardwareMap.get(CRServoImplEx::class.java, "intakeRight")

        servo.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        servo1.pwmRange = PwmControl.PwmRange(500.0, 2500.0)

        //servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        servo.power = 0.0
        servo1.power = 0.0

        elapsedtime.reset()
    }

    override fun loop() {

        if(gamepad1.a) {
            requested = 1.0
        }
        if(gamepad1.b) {
            requested = 0.0
        }
        if(gamepad1.x) {
            requested = 0.5
        }

        if ((abs(lastPos-requested) >= 0.005) || (lastPos != 0.0 && requested == 0.0) || (requested != 0.0 && lastPos == 0.0)) {
            //servo.power = requested
            //servo1.power = requested
            servo.power = requested
            servo1.power = requested
            lastPos = requested
            count++
        }
        //servo.power = requested
        //servo1.power = requested

        telemetry.addData("last", lastPos)
        telemetry.addData("current", requested)
        telemetry.addData("count", count)

        telemetry.update()
    }

}

