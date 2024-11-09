package org.firstinspires.ftc.teamcode.drive.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds
import com.outoftheboxrobotics.photoncore.Photon
import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.constants.DrivebaseConstants
import org.firstinspires.ftc.teamcode.drive.test.ServoPositions
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import kotlin.math.hypot
import kotlin.math.pow

@TeleOp(group = "test")
class ResetServo: OpMode() {
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
            telemetry.addLine("middle")
            servo.position = ServoPositions.mid
        }

        if (gamepad1.b) {
            telemetry.addLine("1")
            servo.position = ServoPositions.max
        }
        if (gamepad1.x) {
            telemetry.addLine("-1")
            servo.position = ServoPositions.min
        }

        telemetry.update()
    }
}

@Config
object ServoPositions {
    @JvmField var min = -1.0
    @JvmField var mid = 0.0
    @JvmField var max = 1.0
}
