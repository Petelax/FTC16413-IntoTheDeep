package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.utils.AllianceColours
import org.firstinspires.ftc.teamcode.utils.Globals
import java.util.function.BooleanSupplier

class IntakeStop(private var intake: Intake): CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.setSpeed(0.0)
    }

    override fun execute() {
        intake.setSpeed(0.0)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {

    }
}