package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Rule;
import org.firstinspires.ftc.teamcode.autoshoot.routes.Issue.Severity;
import org.junit.Test;

/** The validator rules about HIVE tips (G417 and G409), each with a route that breaks it and a legal one. */
public class TipRulesTest {

    /** Validates for RED and for its BLUE half turn; both must give the same findings. */
    private static List<Issue> validate(AutoRoute red) {
        List<Issue> r = RouteValidator.check(red);
        List<Issue> blue = RouteValidator.check(red.forAlliance(Alliance.BLUE));
        assertEquals("red and blue findings differ:\n" + r + "\n" + blue, signature(r), signature(blue));
        return r;
    }

    private static List<String> signature(List<Issue> issues) {
        List<String> out = new ArrayList<String>();
        for (Issue i : issues) {
            out.add(i.severity + "/" + i.rule + "/" + i.stepIndex);
        }
        return out;
    }

    private static int count(List<Issue> issues, Rule rule, Severity severity) {
        int n = 0;
        for (Issue i : issues) {
            if (i.rule == rule && i.severity == severity) {
                n++;
            }
        }
        return n;
    }

    private static Issue first(List<Issue> issues, Rule rule, Severity severity) {
        for (Issue i : issues) {
            if (i.rule == rule && i.severity == severity) {
                return i;
            }
        }
        throw new AssertionError("no " + severity + " for " + rule + " in " + issues);
    }

    // ---- G417: POLLEN fired after the tip ---------------------------------------------------------------

    @Test
    public void aVolleyThatKeepsFiringAfterTheTipIsAnError() {
        // 2 POLLEN, take 2 back from the floor, then fire 4: the 2nd of those 4 is the tip, the other 2 would go into
        // a cell that is already tipping down
        AutoRoute r = AutoRoute.builder("late").start(58, 9.2, -90)
                .shootUp(2).intakeFloor(60, 30, 59.25, 30, -90, 2).shootUp(4).build();
        List<Issue> l = validate(r);
        assertEquals(l.toString(), 1, count(l, Rule.SHOOT_TARGET, Severity.ERROR));
        Issue issue = first(l, Rule.SHOOT_TARGET, Severity.ERROR);
        assertEquals("the second volley is step 4", 3, issue.stepIndex);
        assertTrue(issue.message, issue.message.contains("tips with POLLEN 2 of the 4"));
        assertTrue(issue.message, issue.message.contains("the last 2"));
        assertTrue(issue.message, issue.message.contains("G417"));
    }

    @Test
    public void aVolleyWhoseLastPollenIsTheTipIsFine() {
        // 4 at once: the 4th POLLEN tips, nothing is fired after it
        assertEquals(0, count(validate(AutoRoute.builder("one").start(58, 9.2, -90).shootUp(4).build()), Rule.SHOOT_TARGET,
                Severity.ERROR));
        // 2 + 2 into the same cell: the last POLLEN of the second volley tips
        assertEquals(0, count(validate(AutoRoute.builder("two").start(58, 9.2, -90).shootUp(2).shootUp(2).build()),
                Rule.SHOOT_TARGET, Severity.ERROR));
        // 3 POLLEN never reach the tip: nothing to complain about either
        assertEquals(0, count(validate(AutoRoute.builder("three").start(58, 9.2, -90).shootUp(3).build()), Rule.SHOOT_TARGET,
                Severity.ERROR));
    }
}
