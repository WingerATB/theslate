/* Host tests for vendor identification from a raw advertisement.
 *   cc -I../../components/camvendor/include -o test_camvendor test_camvendor.c
 */
#include <stdio.h>
#include <string.h>
#include "camvendor.h"

static int fails = 0;
#define CHECK(c, m) do { if (!(c)) { printf("   FAIL: %s\n", (m)); fails++; } } while (0)

/* A GoPro advert: flags, then the 16-bit service list carrying FEA6, then
 * manufacturer data under company F202 with status at 10 and model at 11. */
static uint8_t gp_adv[64];
static uint8_t gp_len;
/* Shaped like a real GoPro advert, per GoPro's own SDK: after the two
 * company-id bytes come schema, camera status, model id, capabilities.
 *
 * The first version of this fixture mirrored the implementation's offsets
 * rather than the specification's, so the tests passed on a packet no camera
 * ever sends -- it validated the misunderstanding instead of catching it. */
static void make_gopro(uint8_t model, bool awake)
{
    uint8_t i = 0;
    gp_adv[i++] = 2; gp_adv[i++] = 0x01; gp_adv[i++] = 0x06;          /* flags */
    gp_adv[i++] = 3; gp_adv[i++] = 0x02; gp_adv[i++] = 0xA6; gp_adv[i++] = 0xFE;

    uint8_t len_at = i;
    gp_adv[i++] = 0;                               /* patched below        */
    gp_adv[i++] = 0xFF;                            /* manufacturer data    */
    uint8_t pay = i;
    gp_adv[i++] = 0x02; gp_adv[i++] = 0xF2;        /* 0  company id        */
    gp_adv[i++] = 3;                               /* 2  schema version    */
    gp_adv[i++] = awake ? 0x01 : 0x00;             /* 3  camera status     */
    gp_adv[i++] = model;                           /* 4  model id          */
    gp_adv[i++] = 0x3F;                            /* 5  capabilities      */
    for (int k = 0; k < 6; k++) gp_adv[i++] = 0xAB;/* 6..11 id hash        */
    gp_adv[i++] = 0;                               /* 12 media offload     */
    gp_adv[len_at] = (uint8_t)(1 + (i - pay));
    gp_len = i;
}

int main(void)
{
    printf("=== vendor identification from an advertisement ===\n\n");

    printf("1. a GoPro advert is recognised by its service UUID\n");
    {
        make_gopro(GOPRO_MODEL_HERO12, true);
        CHECK(cam_adv_is_gopro(gp_adv, gp_len), "recognised");
        CHECK(cam_adv_gopro_model(gp_adv, gp_len) == GOPRO_MODEL_HERO12, "model read");
    }

    printf("2. the complete-list AD type works as well as the incomplete one\n");
    {
        /* Which of 0x02 / 0x03 a camera uses is not something to bet a scan
         * filter on. */
        make_gopro(GOPRO_MODEL_HERO11, true);
        gp_adv[4] = 0x03;
        CHECK(cam_adv_is_gopro(gp_adv, gp_len), "0x03 recognised too");
    }

    printf("3. something that is not a GoPro is not mistaken for one\n");
    {
        /* A DJI advert: manufacturer data with DJI's company id and no FEA6. */
        const uint8_t dji[] = { 2, 0x01, 0x06, 6, 0xFF, 0xAA, 0x08, 0x19, 0x00, 0xFA };
        CHECK(!cam_adv_is_gopro(dji, sizeof(dji)), "DJI is not a GoPro");
        CHECK(cam_adv_gopro_model(dji, sizeof(dji)) == 0, "and yields no model");
    }

    printf("4. BLOCKER: a malformed advert must not read past its end\n");
    {
        /* A length byte claiming more than the buffer holds is the classic way
         * to walk off the end of a scan result. */
        const uint8_t bad[] = { 40, 0x02, 0xA6, 0xFE };
        CHECK(!cam_adv_is_gopro(bad, sizeof(bad)), "over-long field refused");

        const uint8_t zero[] = { 0, 0, 0, 0 };
        CHECK(!cam_adv_is_gopro(zero, sizeof(zero)), "zero length terminates");
        CHECK(cam_adv_gopro_model(NULL, 0) == 0, "null is safe");
        CHECK(!cam_adv_is_gopro(NULL, 30), "null is safe");
    }

    printf("5. awake is distinguished from 'cannot tell'\n");
    {
        /* An advert with no manufacturer data tells us nothing, and "I cannot
         * tell" must never read as "asleep" -- that would skip a camera that
         * is sitting there switched on. */
        bool known = false;
        make_gopro(GOPRO_MODEL_HERO12, true);
        CHECK(cam_adv_gopro_awake(gp_adv, gp_len, &known) && known, "awake, known");

        make_gopro(GOPRO_MODEL_HERO12, false);
        CHECK(!cam_adv_gopro_awake(gp_adv, gp_len, &known) && known, "asleep, known");

        const uint8_t bare[] = { 3, 0x02, 0xA6, 0xFE };
        cam_adv_gopro_awake(bare, sizeof(bare), &known);
        CHECK(!known, "no manufacturer data -> not known");
    }

    printf("6. models below HERO 9 are refused, with a reason\n");
    {
        const char *why = NULL;
        CHECK(!cam_gopro_model_supported(54, &why), "HERO 8 refused");
        CHECK(why && strstr(why, "HERO 9") != NULL, "and says why, in English");

        why = NULL;
        CHECK(!cam_gopro_model_supported(0, &why), "unknown model refused");
        CHECK(why != NULL, "with a reason");

        CHECK(cam_gopro_model_supported(GOPRO_MODEL_HERO9, NULL), "HERO 9 allowed");
        CHECK(cam_gopro_model_supported(GOPRO_MODEL_HERO13, NULL), "HERO 13 allowed");
    }

    printf("7. an unknown model ABOVE the floor still binds\n");
    {
        /* A HERO that does not exist yet should work without a firmware
         * update, so the floor is a floor and not a whitelist. */
        CHECK(cam_gopro_model_supported(99, NULL), "future model allowed");
        char lab[6];
        cam_gopro_label(99, lab);
        CHECK(strcmp(lab, "GPRO") == 0, "and gets a generic label");
    }

    printf("7b. the model table matches GoPro's firmware table\n");
    {
        /* hypoxic/GoPro-Research, cross-checked with the SDK's test fixture
         * (which parses a real HERO13 advert to cameraId 65). */
        CHECK(GOPRO_MODEL_HERO9 == 55 && GOPRO_MODEL_HERO10 == 57 &&
              GOPRO_MODEL_HERO11 == 58 && GOPRO_MODEL_HERO11_MINI == 60 &&
              GOPRO_MODEL_HERO12 == 62 && GOPRO_MODEL_MAX2 == 64 &&
              GOPRO_MODEL_HERO13 == 65 && GOPRO_MODEL_HERO2024 == 66 &&
              GOPRO_MODEL_MISSION1PRO == 69 && GOPRO_MODEL_LITHERO == 70 &&
              GOPRO_MODEL_MISSION1 == 71, "ids as GoPro numbers them");
        const char *why = NULL;
        /* Two models inside the range are refused by name, and the picker
         * still names them so the user knows what was refused. */
        why = NULL;
        CHECK(!cam_gopro_model_supported(GOPRO_MODEL_HERO2024, &why) && why &&
              strstr(why, "not supported"), "HERO (2024) is refused, with a reason");
        why = NULL;
        CHECK(!cam_gopro_model_supported(GOPRO_MODEL_LITHERO, &why) && why &&
              strstr(why, "not supported"), "LIT HERO is refused, with a reason");
        CHECK(strcmp(cam_gopro_model_name(66), "GoPro HERO (2024)") == 0 &&
              strcmp(cam_gopro_model_name(70), "GoPro LIT HERO") == 0, "both still named");
        CHECK(cam_gopro_model_supported(GOPRO_MODEL_MAX2, NULL) &&
              cam_gopro_model_supported(GOPRO_MODEL_MISSION1, NULL) &&
              cam_gopro_model_supported(GOPRO_MODEL_MISSION1PRO, NULL), "MAX 2 and MISSION 1 bind");
        CHECK(!cam_gopro_model_supported(50, &why), "HERO8 (50) is refused");
    }

    printf("7c. which models carry 2-byte ids\n");
    {
        CHECK(cam_gopro_wide_ids(GOPRO_MODEL_MISSION1) && cam_gopro_wide_ids(GOPRO_MODEL_MISSION1PRO),
              "MISSION 1 and MISSION 1 Pro do");
        CHECK(!cam_gopro_wide_ids(GOPRO_MODEL_HERO13) && !cam_gopro_wide_ids(GOPRO_MODEL_LITHERO) &&
              !cam_gopro_wide_ids(GOPRO_MODEL_MAX2) && !cam_gopro_wide_ids(GOPRO_MODEL_HERO2024),
              "HERO13, LIT HERO, MAX 2 and HERO (2024) do not");
        CHECK(cam_gopro_wide_ids(72), "a model newer than any we know is assumed to");
    }

    printf("8. labels fit the OSD's four-character budget\n");
    {
        const uint8_t models[] = { 55, 57, 58, 60, 62, 64, 65, 69, 70, 71, 0, 99 };
        for (unsigned i = 0; i < sizeof(models); i++) {
            char lab[6];
            cam_gopro_label(models[i], lab);
            if (strlen(lab) == 0 || strlen(lab) > 4) {
                printf("   FAIL: model %u -> \"%s\" (%zu chars)\n",
                       models[i], lab, strlen(lab));
                fails++;
            }
        }
    }

    printf("9. the model sits where the specification puts it, not where\n");
    printf("   the implementation happened to look\n");
    {
        /* The offsets are counted from the start of the AD payload, which
         * INCLUDES the company id -- GoPro's own SDK reads them one field
         * later because a phone's BLE API strips it first. Getting that wrong
         * shifts everything and lands on the serial number, which parses fine
         * and means nothing. This pins the byte. */
        make_gopro(GOPRO_MODEL_HERO13, true);
        uint8_t n = 0;
        const uint8_t *d = cam_adv_field(gp_adv, gp_len, 0xFF, &n);
        CHECK(d != NULL, "manufacturer data found");
        CHECK(d[0] == 0x02 && d[1] == 0xF2, "company id at 0..1");
        CHECK(d[GOPRO_ADV_SCHEMA_OFFSET] == 3, "schema at 2");
        CHECK(d[GOPRO_ADV_STATUS_OFFSET] == 0x01, "status at 3");
        CHECK(d[GOPRO_ADV_MODEL_OFFSET] == GOPRO_MODEL_HERO13, "model at 4");
        CHECK(cam_adv_gopro_model(gp_adv, gp_len) == GOPRO_MODEL_HERO13,
              "and the accessor agrees");
    }

    printf("10. a short manufacturer field cannot yield a phantom model\n");
    {
        /* Truncated at the schema byte: there is no model to read, and
         * inventing one would bind a camera on a number from nowhere. */
        const uint8_t short_adv[] = {
            3, 0x02, 0xA6, 0xFE,
            4, 0xFF, 0x02, 0xF2, 3,
        };
        CHECK(cam_adv_is_gopro(short_adv, sizeof(short_adv)), "still a GoPro");
        CHECK(cam_adv_gopro_model(short_adv, sizeof(short_adv)) == 0, "no model");
        bool known = true;
        cam_adv_gopro_awake(short_adv, sizeof(short_adv), &known);
        CHECK(!known, "and awake is unknowable");
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILURES" : "ALL PASS",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
