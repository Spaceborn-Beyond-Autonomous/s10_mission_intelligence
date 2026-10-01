#include "intent_parser.h"
#include <string.h>
#include <ctype.h>

// input form
// SURVEY → "survey <area>"
// DELIVER → "deliver from <pickup> to <dropoff>" ["carrying <payload>"]  (payload optional)
// PATROL → "patrol <route_or_area>"
// RETURN_TO_BASE → "return to base" / "return to home"
// LAND → "land <location>" (location optional)

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
        extract_after(lower, text, "patrol", intent.route_or_area, sizeof(intent.route_or_area));
        intent.repetitions = 1; // TODO: parse an actual number if present in text
    }
    else if (contains_word(lower, "return_to_base") || contains_word(lower, "return to base")
          || contains_word(lower, "return_to_home") || contains_word(lower, "return to home")) {
        intent.type = RETURN_TO_BASE;
    }
    else if (contains_word(lower, "land")) {
        intent.type = LAND;
        extract_after(lower, text, "land", intent.location, sizeof(intent.location));
    }

    return intent;
}