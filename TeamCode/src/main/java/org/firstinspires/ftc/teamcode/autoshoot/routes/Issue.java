package org.firstinspires.ftc.teamcode.autoshoot.routes;

/**
 * One finding of the {@link RouteValidator}: a rule the route breaks (ERROR: the route cannot work or
 * breaks a game rule) or is doubtful about (WARN: probably fine, but check it on the field).
 */
public final class Issue {

    public enum Severity {
        ERROR,
        WARN
    }

    /** The rule that produced a finding, with the BIOBUZZ manual (TU09) number it comes from. */
    public enum Rule {
        /** G304: a legal start placement. */
        START_G304("G304", "legal start placement"),
        /** 9.2 and G412: the robot stays on the field. */
        FIELD_BOUNDS("9.2/G412", "stay inside the field"),
        /** G417 and G409: keep clear of the HIVE frame. */
        HIVE_KEEPOUT("G417/G409", "keep out of the HIVE frame"),
        /** G304.D and G415/G416: no contact with a FLOWER except to retrieve POLLEN. */
        FLOWER_KEEPOUT("G304.D/G415", "keep out of FLOWER plates"),
        /** G402: in AUTO a robot stays on its own side. */
        SIDE_G402("G402", "stay on the own side in AUTO"),
        /** G407: at most 4 POLLEN under control. */
        CAPACITY_G407("G407", "at most 4 POLLEN held"),
        /** 10.3.1 and G402: where POLLEN can be taken from in AUTO. */
        SOURCE("10.3.1/G402", "POLLEN sources"),
        /** G401 and 10.1: AUTO lasts 30 s. */
        TIME_G401("G401/10.1", "finish within the 30 s AUTO"),
        /** 10.5.4 and G402: where the robot ends the route. */
        END_POSE("10.5.4/G402", "end pose"),
        /** G417: shoot into an UP cell from the standoff band. */
        SHOOT_POSE("G417", "shooting pose"),
        /** G417: only an upward-facing cell may be launched into. */
        SHOOT_TARGET("G417", "shoot at an UP cell"),
        /** 10.5.4: PARK means partly inside the own LOADING ZONE. */
        PARK_ZONE("10.5.4", "PARK inside the LOADING ZONE"),
        /** G409: do not catch or deflect what a tipped HIVE releases. */
        SPILL_G409("G409", "stay clear of the cell that tips"),
        /** Design rule: a route that says who shoots the first tip must be consistent with it. */
        TIP_ASSUMPTION("-", "first-tip assumption"),
        /** Design rule: an explicit pose should be where the route already is. */
        IMPLICIT_MOVE("-", "pose reached by an unplanned move");

        private final String manualRule;
        private final String title;

        Rule(String manualRule, String title) {
            this.manualRule = manualRule;
            this.title = title;
        }

        /** The manual rule number(s), e.g. "G402"; "-" for a design rule. */
        public String manualRule() {
            return manualRule;
        }

        public String title() {
            return title;
        }
    }

    public final Severity severity;
    /** Index (0-based) of the step the finding is about, or -1 for the whole route. */
    public final int stepIndex;
    public final Rule rule;
    public final String message;

    public Issue(Severity severity, int stepIndex, Rule rule, String message) {
        this.severity = severity;
        this.stepIndex = stepIndex;
        this.rule = rule;
        this.message = message;
    }

    public boolean isError() {
        return severity == Severity.ERROR;
    }

    /** The step number as people count it (and as the diagram numbers it): 1-based, 0 for the whole route. */
    public int stepNumber() {
        return stepIndex + 1;
    }

    @Override
    public String toString() {
        String where = stepIndex < 0 ? "route" : "step " + stepNumber();
        return severity + " " + where + " [" + rule.manualRule() + "] " + message;
    }
}
