package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.constants.DeviceIDs
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.utils.Cache

class Deposit(hardwareMap: HardwareMap): SubsystemBase() {
    private var servo: ServoImplEx = hardwareMap.get(ServoImplEx::class.java, DeviceIDs.DEPOSIT)
    private var lastPosition = 0.0

    init {
        servo.pwmRange = PwmControl.PwmRange(510.0, 2490.0)
        servo.position = VerticalConstants.DepositPositions.OUT
    }

    fun setPosition(position: Double) {
        if (Cache.shouldUpdate(lastPosition, position, 0.005)) {
            servo.position = position
            lastPosition = position
        }
    }

}

class DepositCommand(private val deposit: Deposit, private val position: Double): CommandBase() {
    override fun initialize() {
        deposit.setPosition(position)
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
