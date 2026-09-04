/* Host tests for the bound-camera list -- the shipped implementation.
 *   cc -I../../components/duml/include -o test_bind test_bind.c && ./test_bind
 */
#include <stdio.h>
#include <string.h>
#include "cam_bind.h"

static int fails = 0;
#define CHECK(c, m) do { if (!(c)) { printf("   FAIL: %s\n", (m)); fails++; } } while (0)

static cam_bind_t mk(uint8_t tag, const char *label)
{
    cam_bind_t c = {0};
    for (int i = 0; i < 6; i++) c.addr[i] = tag;
    c.model = tag;
    snprintf(c.label, sizeof(c.label), "%s", label);
    return c;
}
static void addr_of(uint8_t tag, uint8_t out[6])
{
    for (int i = 0; i < 6; i++) out[i] = tag;
}
/* The list's addresses, top first, as a compact string like "AB" for tags. */
static void tags(const cam_bind_list_t *l, char *out)
{
    int i = 0;
    for (; i < (int)l->n; i++) out[i] = (char)l->e[i].addr[0];
    out[i] = '\0';
}
#define WANT_ORDER(l, s) do {                                            \
    char got[CAM_BIND_MAX + 1]; tags(&(l), got);                         \
    if (strcmp(got, (s)) != 0) {                                         \
        printf("   FAIL: order is \"%s\", wanted \"%s\"\n", got, (s));   \
        fails++;                                                         \
    }                                                                    \
} while (0)

int main(void)
{
    printf("=== bound camera list ===\n\n");

    printf("1. cameras are held in the order they were bound\n");
    {
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', "NANO"), b = mk('B', "O360");
        CHECK(cam_bind_add(&l, &a), "first add");
        CHECK(cam_bind_add(&l, &b), "second add");
        CHECK(l.n == 2, "two held");
        WANT_ORDER(l, "AB");
    }

    printf("2. re-binding a camera it already holds updates it in place\n");
    {
        /* Picking the same camera again to correct its label must not demote
         * it to last, and must not add a second entry for one MAC -- the
         * second would be unreachable and the list a camera shorter than it
         * looks. */
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', "NANO"), b = mk('B', "O360");
        cam_bind_add(&l, &a);
        cam_bind_add(&l, &b);

        cam_bind_t a2 = mk('A', "A6");
        CHECK(cam_bind_add(&l, &a2), "re-bind succeeds");
        CHECK(l.n == 2, "no duplicate entry");
        WANT_ORDER(l, "AB");
        CHECK(strcmp(l.e[0].label, "A6") == 0, "and the entry was refreshed");
    }

    printf("3. the list is bounded, and says so rather than dropping one\n");
    {
        cam_bind_list_t l = {0};
        for (int i = 0; i < CAM_BIND_MAX; i++) {
            cam_bind_t c = mk((uint8_t)('A' + i), "CAM");
            CHECK(cam_bind_add(&l, &c), "fits");
        }
        cam_bind_t extra = mk('Z', "CAM");
        CHECK(!cam_bind_add(&l, &extra), "one too many is refused");
        CHECK(l.n == CAM_BIND_MAX, "and nothing was evicted to make room");

        /* Full is not the same as frozen: a camera already held can still be
         * refreshed. */
        cam_bind_t again = mk('A', "NANO");
        CHECK(cam_bind_add(&l, &again), "a full list still accepts a re-bind");
    }

    printf("4. forgetting closes the gap\n");
    {
        /* A list numbered 1, 2, 4 would be asking the user what happened to 3. */
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', ""), b = mk('B', ""), c = mk('C', "");
        cam_bind_add(&l, &a); cam_bind_add(&l, &b); cam_bind_add(&l, &c);

        uint8_t addr[6]; addr_of('B', addr);
        CHECK(cam_bind_remove(&l, addr), "removed");
        CHECK(l.n == 2, "two left");
        WANT_ORDER(l, "AC");

        CHECK(!cam_bind_remove(&l, addr), "removing it twice does nothing");
        CHECK(l.n == 2, "and changes nothing");
    }

    printf("5. reordering puts the named ones on top, in the order given\n");
    {
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', ""), b = mk('B', ""), c = mk('C', "");
        cam_bind_add(&l, &a); cam_bind_add(&l, &b); cam_bind_add(&l, &c);

        uint8_t order[3][6];
        addr_of('C', order[0]); addr_of('A', order[1]); addr_of('B', order[2]);
        CHECK(cam_bind_reorder(&l, order, 3) == 3, "all three placed");
        WANT_ORDER(l, "CAB");
    }

    printf("6. BLOCKER: a stale order must not lose a camera\n");
    {
        /* The page sends back an order built from a list it fetched earlier.
         * Between those moments a camera can be added elsewhere. Anything the
         * order does not mention keeps its place at the end rather than
         * vanishing from a module the user cannot see. */
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', ""), b = mk('B', ""), c = mk('C', "");
        cam_bind_add(&l, &a); cam_bind_add(&l, &b); cam_bind_add(&l, &c);

        uint8_t order[1][6];
        addr_of('C', order[0]);
        CHECK(cam_bind_reorder(&l, order, 1) == 1, "one placed");
        CHECK(l.n == 3, "nothing was dropped");
        WANT_ORDER(l, "CAB");
    }

    printf("7. an order naming a camera it does not hold is ignored\n");
    {
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', ""), b = mk('B', "");
        cam_bind_add(&l, &a); cam_bind_add(&l, &b);

        uint8_t order[3][6];
        addr_of('Z', order[0]);   /* never bound */
        addr_of('B', order[1]);
        addr_of('A', order[2]);
        CHECK(cam_bind_reorder(&l, order, 3) == 2, "only the two real ones");
        CHECK(l.n == 2, "and no phantom entry was created");
        WANT_ORDER(l, "BA");
    }

    printf("8. an order naming the same camera twice does not duplicate it\n");
    {
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', ""), b = mk('B', "");
        cam_bind_add(&l, &a); cam_bind_add(&l, &b);

        uint8_t order[3][6];
        addr_of('A', order[0]); addr_of('A', order[1]); addr_of('B', order[2]);
        cam_bind_reorder(&l, order, 3);
        CHECK(l.n == 2, "still two");
        WANT_ORDER(l, "AB");
    }

    printf("9. an empty order leaves the list exactly as it was\n");
    {
        cam_bind_list_t l = {0};
        cam_bind_t a = mk('A', ""), b = mk('B', "");
        cam_bind_add(&l, &a); cam_bind_add(&l, &b);
        CHECK(cam_bind_reorder(&l, NULL, 0) == 0, "nothing placed");
        WANT_ORDER(l, "AB");
    }

    printf("10. find answers honestly on an empty list\n");
    {
        cam_bind_list_t l = {0};
        uint8_t addr[6]; addr_of('A', addr);
        CHECK(cam_bind_find(&l, addr) < 0, "not found");
        CHECK(!cam_bind_remove(&l, addr), "removing nothing fails cleanly");
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILURES" : "ALL PASS",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
