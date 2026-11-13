package org.firstinspires.ftc.teamcode.autoshoot.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Finds the shortest straight-line route between two field points that stays out of a set of
 * rectangular obstacles and inside a rectangular boundary (a visibility graph over the obstacle
 * corners). The robot is treated as a point, so callers pass obstacles that are already inflated by
 * the robot's size plus a safety margin and a boundary that is already shrunk.
 *
 * <p>Pure Java; no allocation beyond what a single plan needs, and fast enough to run on a
 * Control Hub when the driver presses the button.
 */
public final class PathPlanner {
    /** Pushes corner waypoints this far outside an obstacle so a path never grazes it exactly. */
    private static final double CORNER_PUSH_IN = 0.05;

    private final Rect bounds;
    private final List<Rect> obstacles;

    public PathPlanner(Rect bounds, List<Rect> obstacles) {
        this.bounds = bounds;
        this.obstacles = new ArrayList<Rect>(obstacles);
    }

    /** True if the straight segment is inside the bounds and crosses no obstacle's interior. */
    public boolean segmentClear(double x0, double y0, double x1, double y1) {
        if (!bounds.contains(x0, y0) || !bounds.contains(x1, y1)) {
            return false;
        }
        for (Rect o : obstacles) {
            if (o.segmentCrossesInterior(x0, y0, x1, y1)) {
                return false;
            }
        }
        return true;
    }

    /** The obstacle that strictly contains the point, or null. */
    private Rect containing(double x, double y) {
        for (Rect o : obstacles) {
            if (o.containsStrictly(x, y)) {
                return o;
            }
        }
        return null;
    }

    /** Moves a point that is inside an obstacle to the nearest point just outside it. */
    private double[] projectOut(double x, double y) {
        double px = x;
        double py = y;
        for (int guard = 0; guard < 4; guard++) {
            Rect o = containing(px, py);
            if (o == null) {
                break;
            }
            double dLeft = px - o.minX;
            double dRight = o.maxX - px;
            double dBottom = py - o.minY;
            double dTop = o.maxY - py;
            double m = Math.min(Math.min(dLeft, dRight), Math.min(dBottom, dTop));
            if (m == dLeft) {
                px = o.minX - CORNER_PUSH_IN;
            } else if (m == dRight) {
                px = o.maxX + CORNER_PUSH_IN;
            } else if (m == dBottom) {
                py = o.minY - CORNER_PUSH_IN;
            } else {
                py = o.maxY + CORNER_PUSH_IN;
            }
        }
        return new double[]{px, py};
    }

    /**
     * Plans a route.
     *
     * @return the waypoints to drive through, in order, always ending at the goal and never including
     *         the start (a straight shot is just {@code [goal]}); {@code null} if there is no route
     *         (the goal is inside an obstacle or outside the bounds, or it is walled off). If the
     *         start is itself inside an obstacle, the first waypoint is the nearest point just outside it.
     */
    public List<double[]> plan(double startX, double startY, double goalX, double goalY) {
        if (!bounds.contains(goalX, goalY) || containing(goalX, goalY) != null) {
            return null;
        }
        double sx = startX;
        double sy = startY;
        List<double[]> prefix = new ArrayList<double[]>();
        if (containing(sx, sy) != null || !bounds.contains(sx, sy)) {
            double[] out = projectOut(sx, sy);
            // Keep the pushed start inside the bounds (it can only have moved a hair outside an obstacle).
            out[0] = Math.max(bounds.minX, Math.min(bounds.maxX, out[0]));
            out[1] = Math.max(bounds.minY, Math.min(bounds.maxY, out[1]));
            if (containing(out[0], out[1]) != null) {
                return null;
            }
            prefix.add(new double[]{out[0], out[1]});
            sx = out[0];
            sy = out[1];
        }
        if (segmentClear(sx, sy, goalX, goalY)) {
            List<double[]> direct = new ArrayList<double[]>(prefix);
            direct.add(new double[]{goalX, goalY});
            return direct;
        }

        // Nodes: start, goal, and the four pushed-out corners of every obstacle that are usable.
        List<double[]> nodes = new ArrayList<double[]>();
        nodes.add(new double[]{sx, sy});
        nodes.add(new double[]{goalX, goalY});
        for (Rect o : obstacles) {
            double[][] corners = {
                    {o.minX - CORNER_PUSH_IN, o.minY - CORNER_PUSH_IN},
                    {o.maxX + CORNER_PUSH_IN, o.minY - CORNER_PUSH_IN},
                    {o.maxX + CORNER_PUSH_IN, o.maxY + CORNER_PUSH_IN},
                    {o.minX - CORNER_PUSH_IN, o.maxY + CORNER_PUSH_IN}};
            for (double[] c : corners) {
                if (bounds.contains(c[0], c[1]) && containing(c[0], c[1]) == null) {
                    nodes.add(c);
                }
            }
        }

        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] done = new boolean[n];
        for (int i = 0; i < n; i++) {
            dist[i] = Double.POSITIVE_INFINITY;
            prev[i] = -1;
        }
        dist[0] = 0.0;
        for (int iter = 0; iter < n; iter++) {
            int u = -1;
            for (int i = 0; i < n; i++) {
                if (!done[i] && (u < 0 || dist[i] < dist[u])) {
                    u = i;
                }
            }
            if (u < 0 || Double.isInfinite(dist[u])) {
                break;
            }
            done[u] = true;
            if (u == 1) {
                break;
            }
            for (int v = 0; v < n; v++) {
                if (done[v]) {
                    continue;
                }
                double[] a = nodes.get(u);
                double[] b = nodes.get(v);
                if (!segmentClear(a[0], a[1], b[0], b[1])) {
                    continue;
                }
                double d = dist[u] + Math.hypot(b[0] - a[0], b[1] - a[1]);
                if (d < dist[v]) {
                    dist[v] = d;
                    prev[v] = u;
                }
            }
        }
        if (Double.isInfinite(dist[1])) {
            return null;
        }
        List<double[]> reversed = new ArrayList<double[]>();
        for (int at = 1; at != 0; at = prev[at]) {
            reversed.add(nodes.get(at));
        }
        Collections.reverse(reversed);
        List<double[]> result = new ArrayList<double[]>(prefix);
        result.addAll(reversed);
        return result;
    }

    /** Total length of start -> waypoints, in inches. */
    public static double length(double startX, double startY, List<double[]> waypoints) {
        double x = startX;
        double y = startY;
        double total = 0.0;
        for (double[] w : waypoints) {
            total += Math.hypot(w[0] - x, w[1] - y);
            x = w[0];
            y = w[1];
        }
        return total;
    }
}
