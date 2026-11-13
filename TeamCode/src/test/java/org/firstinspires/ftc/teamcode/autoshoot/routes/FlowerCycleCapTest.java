package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Angles;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;

/**
 * A FLOWER pick-up makes {@code cyclesPerPollen} push-and-release cycles for each POLLEN it is asked for, but never more than
 * {@code Auto.FLOWER_WIGGLE_MAX_CYCLES} ({@link FlowerWiggle#cyclesFor}). When the cap bites, the pick-up delivers FEWER POLLEN than the route asked
 * for. These tests pin that the route plan ({@link RouteAnalysis}), the validator and the robot-side script ({@link FlowerWiggleSequence}) all agree
 * on how many POLLEN that is, and that the validator says so, instead of the plan counting POLLEN the robot never takes (which would make the tip,
 * capacity and shot accounting of everything after the pick-up wrong).
 *
 * <p>How it works: no robot. A route shoots 4 pre-loads and then asks for 3 POLLEN from FLOWER 1. The wiggle numbers are changed through the
 * timing model, never through the constants: 2 cycles per POLLEN with a cap of 4 cycles (a pick-up of 3 needs 6, so it delivers only 2), and the same
 * with a cap of 8 (nothing is cut). The script is run in a small simulated heading loop (critically damped, so it arrives without overshoot).
 */
public class FlowerCycleCapTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double[] FLOWER_1 = {BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][0], BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1]};
    private static final int INTAKE = 3;
    private static final int ASKED = 3;
    private static final double DT = 0.01;
    private static final double WN = 14.0;                 // natural frequency of the simulated heading loop, rad/s

    private static AutoRoute route() {
        // (the 5 s wait lets what the tipped cell drops land before the robot leaves, G409)
        return AutoRoute.builder("cap").start(35, FRONT, -90).shootUp(4).waitSeconds(5).intakeFlower(FLOWER_1[0], FLOWER_1[1], ASKED).build();
    }

    private static TimingModel timing(int cyclesPerPollen, int maxCycles) {
        return WiggleTestRobot.timing().withFlowerSpec(WiggleTestRobot.spec().withCycles(cyclesPerPollen, maxCycles));
    }

    private static RouteAnalysis analyze(int cyclesPerPollen, int maxCycles) {
        return RouteAnalysis.of(route(), new FieldModel(), timing(cyclesPerPollen, maxCycles));
    }

    private static List<Issue> validate(int cyclesPerPollen, int maxCycles) {
        return new RouteValidator(new FieldModel(), timing(cyclesPerPollen, maxCycles)).validate(route());
    }

    /** Runs the robot-side script of a pick-up to its end with a critically damped heading loop and returns the POLLEN it counted. */
    private static int pollenTakenByTheScript(FlowerPress press, int heldAtStart) {
        FlowerWiggleSequence seq = new FlowerWiggleSequence(press, heldAtStart, 0.0);
        double heading = press.start.heading;
        double velocity = 0.0;
        double t = 0.0;
        while (!seq.finished() && t < 60.0) {
            FlowerWiggleSequence.Command c = seq.update(t, heading);
            velocity += (WN * WN * Angles.error(heading, c.hold.heading) - 2.0 * WN * velocity) * DT;
            heading += velocity * DT;
            t += DT;
        }
        assertTrue("the script must finish", seq.finished());
        assertFalse("the heading loop is good, the script must not give up", seq.failed());
        return seq.pollenTaken();
    }

    private static int plannedDelta(RouteAnalysis a) {
        return a.step(INTAKE).pollenAfter - a.step(INTAKE).pollenBefore;
    }

    /**
     * Situation: 2 cycles per POLLEN, a cap of 4 cycles, a pick-up of 3 POLLEN (it needs 6 cycles).
     * Expected: the pick-up makes 4 cycles and delivers 2 POLLEN; the plan adds 2 POLLEN, not 3; and the script, run to its end, counts the same 2.
     * Why: the plan used to add the 3 asked for while the script could only deliver 2, so the tip, capacity and shot counts after the pick-up
     *     assumed a POLLEN the robot never took, and nothing said so.
     * Negative twin: with a cap of 8 nothing is cut: 6 cycles, 3 POLLEN, in the plan and in the script.
     */
    @Test
    public void thePlanCountsOnlyThePollenTheCyclesDeliverAndTheScriptAgrees() {
        RouteAnalysis capped = analyze(2, 4);
        FlowerPress cappedPress = capped.step(INTAKE).press;
        assertNotNull(cappedPress);
        assertEquals(4, cappedPress.cycles);
        assertEquals(2, cappedPress.pollenDelivered());
        assertEquals("the plan adds what is delivered", 2, plannedDelta(capped));
        assertEquals("and the sources lose that many", 2, FlowerWiggle.pollenFor(cappedPress.spec, cappedPress.cycles));
        assertEquals("the script counts the same", plannedDelta(capped), pollenTakenByTheScript(cappedPress, capped.step(INTAKE).pollenBefore));

        RouteAnalysis roomy = analyze(2, 8);
        FlowerPress roomyPress = roomy.step(INTAKE).press;
        assertEquals(6, roomyPress.cycles);
        assertEquals(ASKED, roomyPress.pollenDelivered());
        assertEquals(ASKED, plannedDelta(roomy));
        assertEquals(plannedDelta(roomy), pollenTakenByTheScript(roomyPress, roomy.step(INTAKE).pollenBefore));
    }

    /**
     * Situation: the same two routes, validated.
     * Expected: the capped one has exactly one finding about the pick-up, a WARN on the INTAKE step that says how many POLLEN it asks for, how many
     *     it delivers and what limit would fix it; the uncapped one has none. The gallery sentence says the same.
     * Why: a plan that quietly counts fewer POLLEN than the author wrote would be as confusing as one that counts too many.
     */
    @Test
    public void theValidatorSaysWhenTheCycleLimitCutsAPickUpShort() {
        List<Issue> capped = validate(2, 4);
        int warns = 0;
        for (Issue i : capped) {
            if (i.rule == Rule.FLOWER_PRESS && i.severity == Severity.WARN) {
                warns++;
                assertEquals(INTAKE, i.stepIndex);
                assertTrue(i.message, i.message.contains("asks for 3 POLLEN"));
                assertTrue(i.message, i.message.contains("takes out only 2"));
                assertTrue(i.message, i.message.contains("raise the limit to 6"));
                assertTrue(i.message, i.message.contains("Auto.FLOWER_WIGGLE_MAX_CYCLES"));
            }
        }
        assertEquals(capped.toString(), 1, warns);
        for (Issue i : validate(2, 8)) {
            assertFalse(i.toString(), i.rule == Rule.FLOWER_PRESS && i.severity == Severity.WARN);
        }
        assertTrue(analyze(2, 4).step(INTAKE).pickupText(), analyze(2, 4).step(INTAKE).pickupText().contains("3 asked for, the cycle limit allows only 2"));
        assertTrue(analyze(2, 8).step(INTAKE).pickupText(), analyze(2, 8).step(INTAKE).pickupText().startsWith("FLOWER pick-up: 3 POLLEN in about"));
    }

    /**
     * Situation: the shipped numbers (1 cycle per POLLEN, a cap of 6).
     * Expected: a pick-up of 4 POLLEN makes 4 cycles and delivers all 4, so nothing changes for the robot as it is today.
     */
    @Test
    public void theShippedNumbersNeverCutAPickUpShort() {
        FlowerWiggle.Spec shipped = WiggleTestRobot.spec();
        for (int pollen = 1; pollen <= BioBuzzConstants.Auto.FLOWER_POLLEN; pollen++) {
            assertEquals(pollen, FlowerWiggle.pollenFor(shipped, FlowerWiggle.cyclesFor(shipped, pollen)));
        }
    }
}
