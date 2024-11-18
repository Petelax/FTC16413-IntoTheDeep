package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.constants.DeviceIDs

class HorizontalArm(hardwareMap: HardwareMap): SubsystemBase() {
    private var servo: ServoImplEx = hardwareMap.get(ServoImplEx::class.java, DeviceIDs.HORIZONTAL_ARM)

    init {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
    }

    fun setPosition(position: Double) {
        servo.position = position
    }

}

class HorizontalArmCommand(private val horizontalArm: HorizontalArm, private val position: Double): CommandBase() {
    override fun initialize() {
        horizontalArm.setPosition(position)
    }
    override fun execute() {
        super.execute()
    }
    override fun isFinished(): Boolean {
        return true
    }
    override fun end(interrupted: Boolean) {
        super.end(interrupted)
    }
}
