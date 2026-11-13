package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel;
import org.firstinspires.ftc.teamcode.autoshoot.core.Pose2;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.junit.Test;

/**
 * Rule G304 (where a robot may be set up) as ONE piece of code, {@link StartRules}: every part of the rule has a start that breaks only that part and
 * a start that passes it, the route validator shows exactly what the rule finds (there is no second copy of the rule), and the two alliances get the
 * same answers.
 */
public class StartRulesTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;
    private static final double REAR = BioBuzzConstants.Robot.REAR_EXTENT_IN;
    private static final double SIZE = BioBuzzConstants.Field.SIZE_IN;
    private static final FieldModel MODEL = new FieldModel();
    private static final AllianceField RED = new AllianceField(MODEL, Alliance.RED);
    private static final AllianceField BLUE = new AllianceField(MODEL, Alliance.BLUE);

    private static Set<StartRules.Part> parts(AllianceField f, Pose2 start) {
        Set<StartRules.Part> out = EnumSet.noneOf(StartRules.Part.class);
        for (StartRules.Finding finding : StartRules.check(f, start)) {
            out.add(finding.part);
        }
        return out;
    }

    private static Set<StartRules.Part> only(StartRules.Part... p) {
        Set<StartRules.Part> out = EnumSet.noneOf(StartRules.Part.class);
        out.addAll(Arrays.asList(p));
        return out;
    }

    /** The same start for the blue alliance: the half turn of the red one about the field's middle. */
    private static Pose2 blue(Pose2 red) {
        return Alliance.BLUE.fromRed(red, SIZE);
    }

    /**
     * Situation: the starts the team really uses (front on the audience wall, front on the rear wall, back on the wall, a side on the wall, a corner).
     * Expected: no finding at all, for both alliances.
     */
    @Test
    public void legalStartsHaveNoFindings() {
        Pose2[] legal = {
                Pose2.ofDegrees(58.0, FRONT, -90), Pose2.ofDegrees(35.0, FRONT, -90),
                Pose2.ofDegrees(61.5, SIZE - FRONT, 90), Pose2.ofDegrees(35.0, REAR, 90),
                Pose2.ofDegrees(FRONT, 70.0, 180), Pose2.ofDegrees(8.9, 20.0, 90),
                Pose2.ofDegrees(FRONT, 8.9, 180),                       // the audience/red corner, side on the audience wall
        };
        for (Pose2 p : legal) {
            assertTrue("red " + p + ": " + StartRules.check(RED, p), StartRules.check(RED, p).isEmpty());
            assertTrue("blue " + blue(p) + ": " + StartRules.check(BLUE, blue(p)), StartRules.check(BLUE, blue(p)).isEmpty());
            assertTrue(StartRules.isLegal(RED, p));
            assertTrue(StartRules.isLegalAndClear(RED, p));
        }
    }

    /** G304.A, the robot overhangs a wall: the old OpMode start 9.2 in from the audience wall, front first, is 0.57 in through it. */
    @Test
    public void aRobotOverhangingAWallBreaksOnlyPartA() {
        Pose2 over = Pose2.ofDegrees(35.0, 9.2, -90);
        assertEquals(only(StartRules.Part.FIELD), parts(RED, over));
        assertEquals(only(StartRules.Part.FIELD), parts(BLUE, blue(over)));
        StartRules.Finding f = StartRules.check(RED, over).get(0);
        assertTrue(f.error);
        assertEquals("G304.A", f.part.rule);
        assertTrue(f.message, f.message.contains("sticks out of the field by 0.6 in"));
        assertTrue("it says how to place the real robot", f.message.contains("ON the wall"));
        assertFalse(StartRules.isLegal(RED, over));
        // the measurement noise of 0.1 in is forgiven, a little more is not
        assertTrue(StartRules.isLegal(RED, Pose2.ofDegrees(35.0, FRONT - 0.09, -90)));
        assertFalse(StartRules.isLegal(RED, Pose2.ofDegrees(35.0, FRONT - 0.12, -90)));
    }

    /** G304.A, the robot reaches over the center line into the other alliance's half. */
    @Test
    public void aRobotReachingIntoTheOpponentHalfBreaksOnlyTheOwnSidePart() {
        Pose2 across = Pose2.ofDegrees(70.0, FRONT, -90);
        assertEquals(only(StartRules.Part.OWN_SIDE), parts(RED, across));
        assertEquals(only(StartRules.Part.OWN_SIDE), parts(BLUE, blue(across)));
        assertTrue(StartRules.check(RED, across).get(0).message.contains("opponent half"));
        // exactly at the line is fine: the robot's right-hand extent 8.9 in puts its edge on the center line
        assertTrue(StartRules.isLegal(RED, Pose2.ofDegrees(BioBuzzConstants.Field.CENTER_IN - BioBuzzConstants.Robot.LEFT_EXTENT_IN, FRONT, -90)));
        // and the other alliance's start at the same place is the one that is wrong for it
        assertFalse(StartRules.isLegal(BLUE, Pose2.ofDegrees(58.0, FRONT, -90)));
    }

    /** G304.C, the robot touches no wall: 20 in out is not touching, 0.7 in is, 1.0 in is not (the tolerance is 0.75 in). */
    @Test
    public void aRobotAwayFromEveryWallBreaksOnlyPartC() {
        assertEquals(only(StartRules.Part.TOUCH_WALL), parts(RED, Pose2.ofDegrees(35.0, 20.0, -90)));
        assertEquals(only(StartRules.Part.TOUCH_WALL), parts(BLUE, blue(Pose2.ofDegrees(35.0, 20.0, -90))));
        assertTrue(StartRules.isLegal(RED, Pose2.ofDegrees(35.0, FRONT + 0.7, -90)));
        assertFalse(StartRules.isLegal(RED, Pose2.ofDegrees(35.0, FRONT + 1.0, -90)));
        assertTrue(StartRules.check(RED, Pose2.ofDegrees(35.0, 20.0, -90)).get(0).message.contains("must touch a perimeter wall"));
    }

    /** G304.E, the robot is in its own LOADING ZONE; the same place is allowed for the other alliance only if it is not THEIR zone. */
    @Test
    public void aRobotInTheLoadingZoneBreaksOnlyPartE() {
        Pose2 inZone = Pose2.ofDegrees(FRONT, 108.0, 180);
        assertEquals(only(StartRules.Part.LOADING_ZONE), parts(RED, inZone));
        assertEquals(only(StartRules.Part.LOADING_ZONE), parts(BLUE, blue(inZone)));
        assertTrue(StartRules.check(RED, inZone).get(0).message.contains("LOADING ZONE"));
        // just below the zone (its first y is 94.584): standing 8.9 in either side of 85.6 touches it, one more inch is clear
        assertTrue(StartRules.isLegal(RED, Pose2.ofDegrees(FRONT, 94.584 - BioBuzzConstants.Robot.LEFT_EXTENT_IN - 1.0, 180)));
        assertFalse(StartRules.isLegal(RED, Pose2.ofDegrees(FRONT, 94.584 - BioBuzzConstants.Robot.LEFT_EXTENT_IN + 1.0, 180)));
    }

    /** G304.D, the robot touches a FLOWER plate. */
    @Test
    public void aRobotOnAFlowerPlateBreaksOnlyPartD() {
        Pose2 onFlower = Pose2.ofDegrees(FRONT, 48.0, 180);
        assertEquals(only(StartRules.Part.FLOWER), parts(RED, onFlower));
        assertEquals(only(StartRules.Part.FLOWER), parts(BLUE, blue(onFlower)));
        assertTrue(StartRules.check(RED, onFlower).get(0).message.contains("FLOWER 1"));
        // and on the rear wall, beside the rear FLOWER: the plate is x 44.3 to 50.3, a robot facing the wall needs its right edge past 50.3
        double plateMaxX = RED.flowerPlate(1).maxX;
        double left = BioBuzzConstants.Robot.LEFT_EXTENT_IN;
        assertFalse(StartRules.isLegal(RED, Pose2.ofDegrees(plateMaxX + left - 0.5, SIZE - FRONT, 90)));
        assertTrue(StartRules.isLegal(RED, Pose2.ofDegrees(plateMaxX + left + 0.5, SIZE - FRONT, 90)));
    }

    /**
     * G304.D, the wall bracket that holds a FLOWER sticks out 0.36 in past the plate: standing over it is a WARN, so the start is legal but not clear, and
     * the search (which asks for "legal and clear") refuses it.
     */
    @Test
    public void standingOverAFlowerBracketIsAWarnThatTheSearchRefuses() {
        double bracketEnd = BioBuzzConstants.Field.FLOWER_CENTERS_IN[0][1] + BioBuzzConstants.Field.FLOWER_BRACKET_ALONG_WALL_IN / 2.0;
        double side = BioBuzzConstants.Robot.RIGHT_EXTENT_IN;
        Pose2 over = Pose2.ofDegrees(REAR, bracketEnd - 0.18 + side, 0);
        assertEquals(only(StartRules.Part.FLOWER_BRACKET), parts(RED, over));
        assertFalse(StartRules.check(RED, over).get(0).error);
        assertTrue("a WARN does not make the start illegal", StartRules.isLegal(RED, over));
        assertFalse("but the search never proposes it", StartRules.isLegalAndClear(RED, over));
        Pose2 clear = Pose2.ofDegrees(REAR, bracketEnd + 0.02 + side, 0);
        assertTrue(StartRules.isLegalAndClear(RED, clear));
    }

    /** G304.B, the robot is attached to the HIVE frame. A start there also touches no wall, so C is found with it. */
    @Test
    public void aRobotOnTheHiveFrameBreaksPartB() {
        Pose2 onHive = Pose2.ofDegrees(60.0, 70.0, 0);
        Set<StartRules.Part> found = parts(RED, onHive);
        assertTrue(found.toString(), found.contains(StartRules.Part.HIVE));
        assertEquals(only(StartRules.Part.HIVE, StartRules.Part.TOUCH_WALL), found);
        assertEquals(only(StartRules.Part.HIVE, StartRules.Part.TOUCH_WALL), parts(BLUE, blue(onHive)));
    }

    /**
     * Situation: a robot that is outside the field altogether.
     * Expected: part A is found (and C, which looks at the gap to the nearest wall, is not raised for a robot that is past it).
     */
    @Test
    public void aRobotOutsideTheFieldBreaksPartA() {
        assertTrue(parts(RED, Pose2.ofDegrees(-5.0, 70.0, 180)).contains(StartRules.Part.FIELD));
        assertTrue(StartRules.check(RED, Pose2.ofDegrees(-5.0, 70.0, 180)).get(0).message.contains("out of the field"));
    }

    /**
     * Situation: a handful of starts, good and bad, for both alliances, run through the route validator.
     * Expected: its START findings are exactly the findings of {@link StartRules}, one for one, same words, same severity, in the same order: the
     *     validator has no copy of the rule of its own.
     * Why: the search for the best start proposes only what this rule calls legal, so the validator must never call a proposed start illegal.
     */
    @Test
    public void theValidatorShowsExactlyWhatTheRuleFinds() {
        Pose2[] starts = {
                Pose2.ofDegrees(35.0, 9.2, -90), Pose2.ofDegrees(70.0, FRONT, -90), Pose2.ofDegrees(35.0, 20.0, -90),
                Pose2.ofDegrees(FRONT, 108.0, 180), Pose2.ofDegrees(FRONT, 48.0, 180), Pose2.ofDegrees(60.0, 70.0, 0),
                Pose2.ofDegrees(58.0, FRONT, -90), Pose2.ofDegrees(REAR, 112.0 + BioBuzzConstants.Robot.RIGHT_EXTENT_IN, 0),
        };
        for (Alliance alliance : Alliance.values()) {
            AllianceField field = alliance == Alliance.RED ? RED : BLUE;
            for (Pose2 red : starts) {
                AutoRoute route = AutoRoute.builder("x").start(red.x, red.y, red.headingDegrees()).waitSeconds(1).build().forAlliance(alliance);
                List<String> fromRule = new ArrayList<String>();
                for (StartRules.Finding f : StartRules.check(field, route.startPose())) {
                    fromRule.add((f.error ? "ERROR " : "WARN ") + f.message);
                }
                List<String> fromValidator = new ArrayList<String>();
                for (Issue i : RouteValidator.check(route)) {
                    if (i.rule == Rule.START_G304) {
                        assertEquals("the START findings are about step 1", 0, i.stepIndex);
                        fromValidator.add(i.severity + " " + i.message);
                    }
                }
                assertEquals(alliance + " " + red, fromRule, fromValidator);
            }
        }
    }
}
