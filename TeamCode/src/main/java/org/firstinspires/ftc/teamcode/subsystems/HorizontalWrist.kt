package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.constants.DeviceIDs

class HorizontalWrist(hardwareMap: HardwareMap): SubsystemBase() {
    private var servo: ServoImplEx = hardwareMap.get(ServoImplEx::class.java, DeviceIDs.HORIZONTAL_WRIST)

    init {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
    }

    fun setPosition(position: Double) {
        servo.position = position
    }

}
