package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime

@TeleOp(group = "test")
class ServoLimitFinder: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var gamepad: GamepadEx
    private lateinit var servo: ServoImplEx

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //voltage = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()

        gamepad = GamepadEx(gamepad1)

        servo = hardwareMap.get(ServoImplEx::class.java, "test")
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)

        elapsedtime.reset()
    }

    override fun loop() {
        if (gamepad1.a) {
            telemetry.addData("middle", mid)
            servo.position = mid
        }

        if (gamepad1.b) {
            telemetry.addData("max", max)
            servo.position = max
        }
        if (gamepad1.x) {
            telemetry.addData("min", min)
            servo.position = min
        }

        telemetry.update()
    }

    @Config
    companion object ServoPositions {
        @JvmField var min = -1.0
        @JvmField var mid = 0.0
        @JvmField var max = 1.0
    }
}

