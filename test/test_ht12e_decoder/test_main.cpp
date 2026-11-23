#include "tags/Ht12eDecoder.h"
#include <unity.h>

void setUp() {}
void tearDown() {}

namespace
{
bool sendWord(Ht12eDecoder &decoder, uint16_t data, uint32_t unit, uint16_t &id, uint32_t stretch = 0)
{
    TEST_ASSERT_FALSE(decoder.pulse(false, unit * 36, id));
    TEST_ASSERT_FALSE(decoder.pulse(true, unit, id));
    bool received = false;
    for (int bit = 11; bit >= 0; --bit) {
        const bool one = data & (1U << bit);
        TEST_ASSERT_FALSE(decoder.pulse(false, (one ? 2 : 1) * unit + stretch, id));
        received = decoder.pulse(true, (one ? 1 : 2) * unit - stretch, id);
        if (bit) {
            TEST_ASSERT_FALSE(received);
        }
    }
    return received;
}

void threeIdenticalWords()
{
    Ht12eDecoder decoder;
    uint16_t id = 0;
    TEST_ASSERT_FALSE(sendWord(decoder, 0xa3e, 333, id, 90));
    TEST_ASSERT_FALSE(sendWord(decoder, 0xa3e, 333, id, 90));
    TEST_ASSERT_TRUE(sendWord(decoder, 0xa3e, 333, id, 90));
    TEST_ASSERT_EQUAL_HEX16(0xa3e, id);
}

void differentWordRestartsConfirmation()
{
    Ht12eDecoder decoder;
    uint16_t id = 0xffff;
    TEST_ASSERT_FALSE(sendWord(decoder, 0xfff, 1750, id));
    TEST_ASSERT_FALSE(sendWord(decoder, 0xfff, 1750, id));
    TEST_ASSERT_FALSE(sendWord(decoder, 0, 1750, id));
    TEST_ASSERT_FALSE(sendWord(decoder, 0, 1750, id));
    TEST_ASSERT_TRUE(sendWord(decoder, 0, 1750, id));
    TEST_ASSERT_EQUAL_HEX16(0, id);
}

void invalidPulseAndQueueResetDiscardPartialConfirmation()
{
    Ht12eDecoder decoder;
    uint16_t id = 0xffff;
    TEST_ASSERT_FALSE(sendWord(decoder, 0x123, 320, id));
    TEST_ASSERT_FALSE(decoder.pulse(false, 11520, id));
    TEST_ASSERT_FALSE(decoder.pulse(true, 320, id));
    TEST_ASSERT_FALSE(decoder.pulse(false, 20, id));
    TEST_ASSERT_FALSE(sendWord(decoder, 0x123, 320, id));
    TEST_ASSERT_FALSE(sendWord(decoder, 0x123, 320, id));
    decoder.reset();
    TEST_ASSERT_FALSE(sendWord(decoder, 0x123, 320, id));
    TEST_ASSERT_EQUAL_HEX16(0xffff, id);
}

void nextPilotResynchronizesAfterIncompleteWord()
{
    Ht12eDecoder decoder;
    uint16_t id = 0;
    TEST_ASSERT_FALSE(decoder.pulse(false, 36 * 330, id));
    TEST_ASSERT_FALSE(decoder.pulse(true, 330, id));
    TEST_ASSERT_FALSE(sendWord(decoder, 0x567, 330, id));
    TEST_ASSERT_FALSE(sendWord(decoder, 0x567, 330, id));
    TEST_ASSERT_TRUE(sendWord(decoder, 0x567, 330, id));
    TEST_ASSERT_EQUAL_HEX16(0x567, id);
}
} // namespace

static int runDecoderTests()
{
    UNITY_BEGIN();
    RUN_TEST(threeIdenticalWords);
    RUN_TEST(differentWordRestartsConfirmation);
    RUN_TEST(invalidPulseAndQueueResetDiscardPartialConfirmation);
    RUN_TEST(nextPilotResynchronizesAfterIncompleteWord);
    return UNITY_END();
}

#ifdef HT12E_STANDALONE_TEST
int main()
{
    return runDecoderTests();
}
#else
#include <cstdlib>
extern "C" void setup()
{
    std::exit(runDecoderTests());
}
extern "C" void loop() {}
#endif
