package org.firstinspires.ftc.teamcode.drive.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.pasteurized.SDKGamepad
import dev.frozenmilk.mercurial.Mercurial
import dev.frozenmilk.mercurial.bindings.BoundGamepad
import dev.frozenmilk.mercurial.commands.Lambda
import dev.frozenmilk.mercurial.commands.groups.Parallel
import dev.frozenmilk.mercurial.commands.groups.Race
import dev.frozenmilk.mercurial.commands.groups.Sequential
import dev.frozenmilk.mercurial.commands.util.IfElse
import dev.frozenmilk.mercurial.commands.util.Wait
import org.firstinspires.ftc.teamcode.commands.Timeout
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.DrivetrainPIDCoefficients
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.BulkReads
import org.firstinspires.ftc.teamcode.utils.Globals
import org.firstinspires.ftc.teamcode.utils.LoopTimes
import org.firstinspires.ftc.teamcode.utils.Telemetry

@BulkReads.Attach
@LoopTimes.Attach
@Telemetry.Attach


@TeleOp
class ClimbTest : OpMode() {
    private lateinit var motorRight: DcMotorEx
    private lateinit var motorLeft: DcMotorEx
    override fun init() {
        motorRight = hardwareMap.get(DcMotorEx::class.java, DeviceIDs.ELEVATOR_RIGHT)
        motorLeft = hardwareMap.get(DcMotorEx::class.java, DeviceIDs.ELEVATOR_LEFT)

        motorLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        motorLeft.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorRight.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motorLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motorRight.direction = DcMotorSimple.Direction.REVERSE


    }

    override fun start() {
        //SwerveDrivetrain.resetHeading()
    }

    override fun loop() {
        if (gamepad1.a) {
            motorLeft.power = -1.0
            motorRight.power = -1.0
        } else {
            motorLeft.power = 0.0
            motorRight.power = 0.0
        }
    }

}