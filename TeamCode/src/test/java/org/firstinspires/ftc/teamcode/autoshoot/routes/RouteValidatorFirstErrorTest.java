package org.firstinspires.ftc.teamcode.autoshoot.routes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.firstinspires.ftc.teamcode.BioBuzzConstants;
import org.firstinspires.ftc.teamcode.autoshoot.core.Alliance;
import org.junit.Test;

/**
 * {@link RouteValidator#firstError}: the quick yes or no that the search for a start asks hundreds of thousands of times. It must say "no ERROR" exactly when
 * {@link RouteValidator#validate} finds none, and when it finds one that one must be a real finding of the full check.
 */
public class RouteValidatorFirstErrorTest {
    private static final double FRONT = BioBuzzConstants.Robot.FRONT_EXTENT_IN;

    private static void assertAgrees(AutoRoute route) {
        RouteValidator validator = new RouteValidator();
        List<Issue> full = validator.validate(route);
        Issue first = validator.firstError(route);
        assertEquals(route.id() + " " + route.alliance() + ": " + full, RouteValidator.isRunnable(full), first == null);
        if (first != null) {
            boolean listed = false;
            for (Issue i : full) {
                listed |= i.isError() && i.rule == first.rule && i.stepIndex == first.stepIndex && i.message.equals(first.message);
            }
            assertTrue(route.id() + ": the first error is not among the full findings: " + first + "\n" + full, listed);
        }
    }

    /** Situation: every route of the catalog, for both alliances, and the same routes started from a start moved by the placement margin. Expected: it agrees with the full check every time. */
    @Test
    public void itSaysTheSameAsTheFullCheckForTheCatalogAndForMovedStarts() {
        for (AutoRoute route : RouteCatalog.all()) {
            for (Alliance a : Alliance.values()) {
                AutoRoute own = route.forAlliance(a);
                assertAgrees(own);
                AllianceField field = new AllianceField(new org.firstinspires.ftc.teamcode.autoshoot.core.FieldModel(), a);
                for (StartRobustness.Move m : StartRobustness.moves(field, own.startPose())) {
                    assertAgrees(own.withStartPose(m.start));
                }
            }
        }
    }

    /**
     * Situation: a route with one ERROR of each kind of check (a start through the wall, a leg through the HIVE, a volley with nothing to fire, a route that is
     *     too long).
     * Expected: for each, {@code firstError} finds an ERROR that the full check lists; and a legal route gives null.
     */
    @Test
    public void itFindsAnErrorOfEveryKindAndNullForALegalRoute() {
        RouteValidator validator = new RouteValidator();
        AutoRoute legal = RouteAnalysisTest.corridorPark().build();
        assertNull(validator.firstError(legal));
        AutoRoute[] bad = {
                AutoRoute.builder("wall").start(35.0, 9.2, -90).waitSeconds(1).build(),
                AutoRoute.builder("hive").start(58.0, FRONT, -90).driveConstant(58.0, 120.0, -90).build(),
                AutoRoute.builder("empty").start(58.0, FRONT, -90).shootUp(4).shootUp(4).build(),
                AutoRoute.builder("long").start(58.0, FRONT, -90).waitSeconds(40.0).build(),
                AutoRoute.builder("side").start(58.0, FRONT, -90).driveConstant(90.0, FRONT, -90).build(),
        };
        for (AutoRoute r : bad) {
            Issue i = validator.firstError(r);
            assertNotNull(r.id() + " should have an ERROR: " + validator.validate(r), i);
            assertTrue(i.isError());
            assertAgrees(r);
        }
    }
}
