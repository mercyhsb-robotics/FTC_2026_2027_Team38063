package org.firstinspires.ftc.teamcode.autoshoot.render;

import java.util.Locale;

/**
 * A tiny helper that writes well-formed SVG/XML text: every attribute value and text content is
 * escaped, numbers are printed with a dot and at most two decimals regardless of the PC's locale.
 * Pure Java, no libraries.
 */
final class SvgWriter {
    private final StringBuilder sb = new StringBuilder(96 * 1024);

    /** A number with at most 2 decimals and no trailing zeros ("12", "12.5", "0.07"). */
    static String n(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            throw new IllegalArgumentException("cannot write " + v + " into an SVG");
        }
        if (Math.abs(v - Math.rint(v)) < 0.005) {
            return Long.toString(Math.round(v));
        }
        String s = String.format(Locale.ROOT, "%.2f", v);
        int end = s.length();
        while (s.charAt(end - 1) == '0') {
            end--;
        }
        if (s.charAt(end - 1) == '.') {
            end--;
        }
        return s.substring(0, end);
    }

    /** XML-escapes text for use in element content or an attribute value. */
    static String esc(String s) {
        StringBuilder out = new StringBuilder(s.length() + 8);
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            switch (c) {
                case '&':
                    out.append("&amp;");
                    break;
                case '<':
                    out.append("&lt;");
                    break;
                case '>':
                    out.append("&gt;");
                    break;
                case '"':
                    out.append("&quot;");
                    break;
                case '\'':
                    out.append("&apos;");
                    break;
                default:
                    // keep the output plain ASCII: anything else becomes a numeric character reference
                    if (c < 0x20 && c != '\n' && c != '\t') {
                        out.append(' ');
                    } else if (c > 0x7e) {
                        out.append("&#").append((int) c).append(';');
                    } else {
                        out.append(c);
                    }
            }
        }
        return out.toString();
    }

    private void attrs(String... nameValue) {
        if (nameValue.length % 2 != 0) {
            throw new IllegalArgumentException("attributes come in name/value pairs");
        }
        for (int i = 0; i < nameValue.length; i += 2) {
            if (nameValue[i + 1] == null) {
                continue;
            }
            sb.append(' ').append(nameValue[i]).append("=\"").append(esc(nameValue[i + 1])).append('"');
        }
    }

    /** Appends already-valid markup. */
    SvgWriter raw(String markup) {
        sb.append(markup);
        return this;
    }

    SvgWriter open(String tag, String... nameValue) {
        sb.append('<').append(tag);
        attrs(nameValue);
        sb.append(">\n");
        return this;
    }

    SvgWriter close(String tag) {
        sb.append("</").append(tag).append(">\n");
        return this;
    }

    SvgWriter empty(String tag, String... nameValue) {
        sb.append('<').append(tag);
        attrs(nameValue);
        sb.append("/>\n");
        return this;
    }

    /** An element with escaped text content on one line. */
    SvgWriter element(String tag, String content, String... nameValue) {
        sb.append('<').append(tag);
        attrs(nameValue);
        sb.append('>').append(esc(content)).append("</").append(tag).append(">\n");
        return this;
    }

    @Override
    public String toString() {
        return sb.toString();
    }
}
