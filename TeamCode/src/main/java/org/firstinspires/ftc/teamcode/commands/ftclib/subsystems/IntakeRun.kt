package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Intake
import org.firstinspires.ftc.teamcode.utils.AllianceColours
import org.firstinspires.ftc.teamcode.utils.Globals

class IntakeRun(private var intake: Intake): CommandBase() {
    private val allianceColour = Globals.AllianceColour
    init {
        addRequirements(intake)
    }
    override fun initialize() {

    }

    override fun execute() {
        if (allianceColour == AllianceColours.Red) {
            if (intake.getGamePiece() != Intake.Sample.RED && intake.getGamePiece() != Intake.Sample.YELLOW) {
                intake.setSpeed(HorizontalConstants.IntakeSpeeds.MAX)
            } else {
                intake.setSpeed(0.0)
            }
        } else {
            if (intake.getGamePiece() != Intake.Sample.BLUE && intake.getGamePiece() != Intake.Sample.YELLOW) {
                intake.setSpeed(HorizontalConstants.IntakeSpeeds.MAX)
            } else {
                intake.setSpeed(0.0)
            }
        }

    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(interrupted: Boolean) {
        intake.setSpeed(0.0)
    }
}