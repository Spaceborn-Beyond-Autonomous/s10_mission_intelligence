#include "intent_parser.h"
#include <string.h>
#include <ctype.h>

// input form
// SURVEY → "survey <area>"
// DELIVER → "deliver from <pickup> to <dropoff>" ["carrying <payload>"]  (payload optional)
// PATROL → "patrol <route_or_area>"
// RETURN_TO_BASE → "return to base" / "return to home"
// LAND → "land <location>" (location optional)
// TAKEOFF        -> "take off" / "takeoff" / "launch"
// LOITER         -> "loiter" / "hover" / "hold position"
//                   ["at <location>"] ["for <N> [seconds|minutes]"]
//                   (duration defaults to 30 s; "min..." means minutes)

// Returns a pointer to the first standalone (word-boundary) occurrence of
// `keyword` in `haystack`, or NULL if none exists. "Standalone" means the
// character immediately before and immediately after the match (if any)
// isn't a letter — e.g. "land" matches in "we land now" but not inside
// "poland" or "landing". Works for multi-word keywords too (e.g.
// "return to base"), since only the very first and very last character
// of the match are checked against their neighbors.
static const char *find_word(const char *haystack, const char *keyword)
{
    size_t klen = strlen(keyword);
    const char *p = haystack;

    while ((p = strstr(p, keyword)) != NULL) {
        int char_before_ok = (p == haystack) || !isalpha((unsigned char)*(p - 1));
        int char_after_ok  = !isalpha((unsigned char)*(p + klen));

        if (char_before_ok && char_after_ok) return p;

        p += klen;
    }
    return NULL;
}

// True/false wrapper over find_word, for the type-detection checks where
// we only care whether the keyword is present, not where.
static int contains_word(const char *haystack, const char *keyword)
{
    return find_word(haystack, keyword) != NULL;
}

// Finds `keyword` as a standalone word in `lower_haystack`, then copies
// whatever comes right after it from `original_text` (same offset) into
// out — stopping at '.' ',' or end of string.
static void extract_after(const char *lower_haystack, const char *original_text,
    const char *keyword, char *out, size_t out_size)
{
    out[0] = '\0';
    const char *match = find_word(lower_haystack, keyword);
    if (!match) return;

    size_t offset = (size_t)(match - lower_haystack) + strlen(keyword);
    const char *src = original_text + offset;
    while (*src == ' ') src++;

    size_t i = 0;
    while (*src && *src != '.' && *src != ',' && i < out_size - 1) {
        out[i++] = *src++;
    }
    out[i] = '\0';
}

// Like extract_after, but stops at the next standalone occurrence of
// stop_kw instead of running to the end of the string. Both start_kw and
// stop_kw are matched as whole words — e.g. stop_kw "to" won't stop
// early on "Toronto" anymore.
static void extract_between(const char *lower_haystack, const char *original_text,
    const char *start_kw, const char *stop_kw,
    char *out, size_t out_size)
{
    out[0] = '\0';
    const char *match = find_word(lower_haystack, start_kw);
    if (!match) return;

    size_t start = (size_t)(match - lower_haystack) + strlen(start_kw);
    while (lower_haystack[start] == ' ') start++;

    size_t end;
    if (stop_kw) {
        const char *stop_match = find_word(lower_haystack + start, stop_kw);
        end = stop_match ? (size_t)(stop_match - lower_haystack) : strlen(lower_haystack);
    } else {
        end = strlen(lower_haystack);
    }
    while (end > start && lower_haystack[end - 1] == ' ') end--;

    size_t copy_len = end - start;
    if (copy_len > out_size - 1) copy_len = out_size - 1;

    memcpy(out, original_text + start, copy_len);
    out[copy_len] = '\0';
}
// Reads the number after a standalone "for" ("for 20 seconds", "for 2 minutes").
// Returns default_s if there is none. Units: seconds, or minutes if "min..." follows.
static int parse_duration_s(const char *lower, int default_s)
{
    const char *p = find_word(lower, "for");
    if (!p) return default_s;
    p += 3;
    while (*p == ' ') p++;
    if (!isdigit((unsigned char)*p)) return default_s;

    long v = 0;
    while (isdigit((unsigned char)*p) && v < 100000) { v = v * 10 + (*p - '0'); p++; }
    while (*p == ' ') p++;
    if (strncmp(p, "min", 3) == 0) v *= 60;
    return v > 0 ? (int)v : default_s;
}
static int starts_with_word(const char *p, const char *w)
{
    size_t n = strlen(w);
    return strncmp(p, w, n) == 0 && !isalpha((unsigned char)p[n]);
}

// Finds "<N> times|laps|loops|rounds", "twice" or "once". Returns the count (default 1)
// and sets *cut to where the phrase starts (including a preceding "for"), or -1 if none.
static int parse_repetitions(const char *lower, int *cut)
{
    static const char *units[] = {"times", "time", "laps", "lap", "loops", "loop", "rounds", "round"};
    *cut = -1;

    for (const char *p = lower; *p; p++) {
        if (p != lower && isalnum((unsigned char)p[-1])) continue; // word starts only

        long v = 0;
        int found = 0;

        if (isdigit((unsigned char)*p)) {
            const char *q = p;
            while (isdigit((unsigned char)*q) && v < 100000) { v = v * 10 + (*q - '0'); q++; }
            while (isdigit((unsigned char)*q)) q++; 
            while (*q == ' ') q++;
            for (size_t i = 0; i < sizeof(units) / sizeof(units[0]); i++) {
                if (starts_with_word(q, units[i])) { found = 1; break; }
            }
        } else if (starts_with_word(p, "twice")) { v = 2; found = 1; }
        else if (starts_with_word(p, "once"))    { v = 1; found = 1; }

        if (!found) continue;

        int start = (int)(p - lower);
        while (start > 0 && lower[start - 1] == ' ') start--;
        if (start >= 3 && strncmp(lower + start - 3, "for", 3) == 0
            && (start == 3 || !isalpha((unsigned char)lower[start - 4]))) {
            start -= 3;
            while (start > 0 && lower[start - 1] == ' ') start--;
        }
        *cut = start;
        return v > 0 ? (int)v : 1;
    }
    return 1;
}

Intent parse_intent(const char *text)
{
    Intent intent = {0};
    intent.priority = 1;

    char lower[256];
    size_t len = strlen(text);
    if (len >= sizeof(lower)) len = sizeof(lower) - 1;
    for (size_t i = 0; i < len; i++) lower[i] = (char)tolower((unsigned char)text[i]);
    lower[len] = '\0';

    if (contains_word(lower, "survey")) {
        intent.type = SURVEY;
        extract_after(lower, text, "survey", intent.area, sizeof(intent.area));
    }
    else if (contains_word(lower, "deliver")) {
        intent.type = DELIVER;
        extract_between(lower, text, "from", "to", intent.pickup_location, sizeof(intent.pickup_location));
        extract_between(lower, text, "to", "carrying", intent.dropoff_location, sizeof(intent.dropoff_location));
        extract_after(lower, text, "carrying", intent.payload, sizeof(intent.payload));
    }
    else if (contains_word(lower, "patrol")) {
        intent.type = PATROL;
        int cut;
        intent.repetitions = parse_repetitions(lower, &cut);

        char text_cut[256];
        strncpy(text_cut, text, sizeof(text_cut) - 1);
        text_cut[sizeof(text_cut) - 1] = '\0';
        if (cut >= 0) { lower[cut] = '\0'; text_cut[cut] = '\0'; } // route name stops before the count phrase

        extract_after(lower, text_cut, "patrol", intent.route_or_area, sizeof(intent.route_or_area));
        size_t rl = strlen(intent.route_or_area);
        while (rl > 0 && intent.route_or_area[rl - 1] == ' ') intent.route_or_area[--rl] = '\0';
    }
    else if (contains_word(lower, "return_to_base") || contains_word(lower, "return to base")
          || contains_word(lower, "return_to_home") || contains_word(lower, "return to home")) {
        intent.type = RETURN_TO_BASE;
    }
    else if (contains_word(lower, "land")) {
        intent.type = LAND;
        extract_after(lower, text, "land", intent.location, sizeof(intent.location));
    }
    else if (contains_word(lower, "take off") || contains_word(lower, "takeoff")
          || contains_word(lower, "launch")) {
        intent.type = TAKEOFF;
    }
    else if (contains_word(lower, "loiter") || contains_word(lower, "hover")
          || contains_word(lower, "hold position")) {
        intent.type = LOITER;
        extract_between(lower, text, "at", "for", intent.location, sizeof(intent.location));
        intent.duration_s = parse_duration_s(lower, 30);
    }
    return intent;
}