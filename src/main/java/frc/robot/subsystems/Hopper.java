// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * מערכת משנה האחראית על ההופר / אינדקסר (Hopper / Indexer)
 */
public class Hopper extends SubsystemBase {

    // מצבי אפשריים של ההופר / האינדקסר
    public enum HopperState {
        IDLE,      // במצב המתנה
        PASSIVE,   // מצב פסיבי / החזקה
        HAWCK,     // (שם לא ברור – אולי "הכנס" או פעולה ספציפית?)
        TUHUA,     // (שם לא ברור – אולי "שחרר" או פעולה ספציפית?)
    }

    // משתנה ששומר את המצב הנוכחי של ההופר
    HopperState hopperState = HopperState.IDLE;

    // האם יש חפץ משחק (דלק / קורל) בהופר כרגע
    public boolean hasFuel = false;

    // מנוע TalonFX של ההופר
    TalonFX hopper = new TalonFX(Constants.Ports.HOPPER);

    // חיישן CANrange לאיתור קורל בהופר (כרגע מוסתר)
    // CANrange INDEXERCANRANGE = new CANrange(Constants.Ports.INDEXERCANRANGE);

    /**
     * בנאי של מערכת ההופר
     */
    public Hopper() {
        // יצירת אובייקט הגדרות למנוע
        TalonFXConfiguration config = new TalonFXConfiguration();

        // כיוון סיבוב והתנהגות במצב נייטרלי
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // החלת ההגדרות על המנוע
        hopper.getConfigurator().apply(config);
    }

    /**
     * מחזיר את המצב הנוכחי של ההופר
     * 
     * @return המצב הנוכחי מסוג HopperState
     */
    public HopperState getState() {
        return hopperState;
    }

    /**
     * משנה את המצב של ההופר
     * 
     * @param state המצב החדש
     */
    public void setState(HopperState state) {
        hopperState = state;
    }

    /**
     * מגדיר את מהירות המנוע של ההופר
     * <p>
     * ערכים חיוביים → הכנסת אצות (algae)<br>
     * ערכים שליליים → הכנסת קורל (coral)
     * 
     * @param speed מהירות בין -1.0 ל-1.0
     */
    public void setHopperSpeed(double speed) {
        hopper.set(speed);
    }

    /**
     * בודק האם קיים חפץ משחק בהופר
     * 
     * @return true אם יש חפץ, false אחרת
     */
    public boolean hasGamepiece() {
        return hasFuel;
    }

    // דוגמה לחיישן עתידי (מוסתר כרגע)
    // public boolean isFuelIn() {
    //     return INDEXERCANRANGE.getDistance().getValueAsDouble() < 0.15;
    // }

    @Override
    public void periodic() {
        // עדכון לוח בקרה חכם
        SmartDashboard.putNumber("זרם ההופר (אמפר)", hopper.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putString("מצב ההופר", hopperState.toString());
        // SmartDashboard.putBoolean("יש דלק", isFuelIn());

        // מכונת מצבים של ההופר
        switch (hopperState) {
            case IDLE:
                setHopperSpeed(0);
                break;

            case HAWCK:
                // לוגיקה ישנה (מוסתרת):
                // if (isFuelIn()) {
                //     Timer.delay(0.20);
                //     setkickerSpeed(0);
                //     hasFuel = true;
                // } else if (!hasGamepiece()) {
                //     setkickerSpeed(0.3);
                //     setindexerSpeed(0.3);
                // }
                break;

            case TUHUA:
                // לוגיקה ישנה (מוסתרת):
                // if (hopper.getSupplyCurrent().getValueAsDouble() > 20) {
                //     setHopperSpeed(-1.0);
                // } else {
                //     setHopperSpeed(0.3);
                // }
                break;

            case PASSIVE:
                // מצב פסיבי – החזקה עדינה או תיקון
                if (hopper.getSupplyCurrent().getValueAsDouble() > 39) {
                    setHopperSpeed(0.5);   // דחיפה קלה נגד עומס
                } else {
                    setHopperSpeed(-0.5);  // משיכה עדינה להחזקה
                }
                break;
        }
    }
}