package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.utils.Cache

class Intake(hardwareMap: HardwareMap): SubsystemBase() {
    private var left: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_LEFT)
    private var right: CRServoImplEx = hardwareMap.get(CRServoImplEx::class.java, DeviceIDs.INTAKE_RIGHT)

    private var lastSpeed = 0.0

    init {
        left.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setSpeed(speed: Double) {
        val corrected = speed.coerceIn(-1.0..1.0)
        if (Cache.shouldUpdate(lastSpeed, corrected)) {
            left.power = corrected
            right.power = corrected
            lastSpeed = corrected
        }
    }

    fun stop() {
        left.power = 0.0
        right.power = 0.0
    }

}
