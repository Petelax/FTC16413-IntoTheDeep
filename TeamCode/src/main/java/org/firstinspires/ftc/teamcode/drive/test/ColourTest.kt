package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake

@TeleOp(group = "test")
class ColourTest: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var elapsedtime: ElapsedTime
    private lateinit var gamepad: GamepadEx
    //private lateinit var color: RevColorSensorV3
    private lateinit var intake: Intake

    override fun init() {
        elapsedtime = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        intake = Intake(hardwareMap)

        gamepad = GamepadEx(gamepad1)

        elapsedtime.reset()
    }

    override fun loop() {
        // clears the cache on each hub
        for (hub in hubs) {
            hub.clearBulkCache()
        }

        telemetry.addData("red", intake.getRed())
        telemetry.addData("green", intake.getGreen())
        telemetry.addData("blue", intake.getBlue())
        telemetry.addData("distance", intake.getDistance())

        telemetry.addData("state", intake.getGamePiece().name)

        telemetry.addData("ms", elapsedtime.milliseconds())

        intake.periodic()

        elapsedtime.reset()
    }}
