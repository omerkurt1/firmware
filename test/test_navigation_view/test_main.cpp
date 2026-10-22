// NavigationPresenter (src/navigation/NavigationViewModel.cpp) and NavigationRenderer
// (src/graphics/draw/NavigationRenderer.cpp) must turn real NavigationService snapshots
// into safe display content. Guard against stale numbers/arrows surviving a state
// change, unknown ages appearing as zero, and selection changing after rejected input.
#include "MeshTypes.h"
#include "TestUtil.h"
#include "UptimeClock.h"
#include "mesh/NodeDB.h"
#include "navigation/NavigationService.h"
#include "navigation/NavigationViewModel.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <unity.h>

#if HAS_SCREEN && !MESHTASTIC_EXCLUDE_POSITIONDB
#include "graphics/ScreenFonts.h"
#include "graphics/draw/NavigationRenderer.h"

namespace
{
constexpr NodeNum TARGET_A = 0x73500001;
constexpr NodeNum TARGET_B = 0x73500002;
NodeDB *savedNodeDB;
NavigationService *savedNavigation;
meshtastic_DeviceState savedDeviceState;
meshtastic_NodeDatabase savedDatabase;
meshtastic_LocalConfig savedConfig;
meshtastic_LocalModuleConfig savedModuleConfig;
meshtastic_ChannelFile savedChannels;
meshtastic_Position savedPosition;
std::unique_ptr<NodeDB> database;
std::unique_ptr<NavigationService> navigation;

class MemoryDisplay : public OLEDDisplay
{
  public:
    MemoryDisplay(uint16_t width = 128, uint16_t height = 64) { setGeometry(GEOMETRY_RAWMODE, width, height); }
    void display() override {}
    int getBufferOffset() override { return 0; }
    bool connect() override { return true; }
    void sendInitCommands() override {}
    bool hasPixels() const
    {
        for (uint16_t i = 0; i < displayBufferSize; ++i)
            if (buffer[i])
                return true;
        return false;
    }
};

NavigationSnapshot readySnapshot()
{
    NavigationSnapshot snapshot;
    snapshot.targetNode = TARGET_A;
    snapshot.positionState = NavigationPositionState::READY;
    snapshot.distanceMeters = 237;
    snapshot.distanceValid = snapshot.bearingValid = true;
    snapshot.absoluteBearingDegrees = 42;
    snapshot.localPositionAgeKnown = snapshot.targetPositionAgeKnown = true;
    return snapshot;
}

void seedPositions()
{
    meshtastic_Position p{};
    p.latitude_i = 410000000;
    p.longitude_i = 290000000;
    p.has_latitude_i = p.has_longitude_i = true;
    database->updatePosition(database->getNodeNum(), p, RX_SRC_LOCAL);
    p.latitude_i += 10000;
    database->updatePosition(TARGET_A, p, RX_SRC_RADIO);
}

void preview(MemoryDisplay &display, const char *name)
{
    const char *directory = getenv("NAVIGATION_PREVIEW_DIR");
    if (!directory)
        return;
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.pgm", directory, name);
    FILE *file = fopen(path, "wb");
    TEST_ASSERT_NOT_NULL(file);
    fprintf(file, "P5\n%u %u\n255\n", display.getWidth(), display.getHeight());
    for (uint16_t y = 0; y < display.getHeight(); ++y)
        for (uint16_t x = 0; x < display.getWidth(); ++x)
            fputc((display.buffer[x + (y / 8) * display.getWidth()] & (1u << (y % 8))) ? 255 : 0, file);
    TEST_ASSERT_EQUAL_INT(0, fclose(file));
}
} // namespace

void setUp()
{
    savedNodeDB = nodeDB;
    savedNavigation = navigationService;
    savedDeviceState = devicestate;
    savedDatabase = nodeDatabase;
    savedConfig = config;
    savedModuleConfig = moduleConfig;
    savedChannels = channelFile;
    savedPosition = localPosition;
    nodeDatabase = {};
    database = std::make_unique<NodeDB>();
    nodeDB = database.get();
    for (const auto num : database->snapshotPositionNodeNums(0))
        database->eraseNodeSatellites(num);
    navigation = std::make_unique<NavigationService>(*database);
    navigationService = navigation.get();
    config.display.units = meshtastic_Config_DisplayConfig_DisplayUnits_METRIC;
    Time::resetMonotonicForTests();
    Time::setTestMillis(100000);
}

void tearDown()
{
    navigation.reset();
    database.reset();
    navigationService = savedNavigation;
    nodeDB = savedNodeDB;
    devicestate = savedDeviceState;
    nodeDatabase = savedDatabase;
    config = savedConfig;
    moduleConfig = savedModuleConfig;
    channelFile = savedChannels;
    localPosition = savedPosition;
    Time::useRealClock();
    Time::resetMonotonicForTests();
}

static void test_present_noTargetOffersSelection()
{
    const auto view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_EQUAL_STRING("No target", view.primary);
    TEST_ASSERT_EQUAL_STRING("Hold: select target", view.detail);
    TEST_ASSERT_EQUAL_STRING("", view.ages);
    TEST_ASSERT_FALSE(view.showDistance);
    TEST_ASSERT_FALSE(view.showArrow);
}

static void test_present_readyWithoutHeadingKeepsNorthBearingNotArrow()
{
    const auto view = NavigationPresenter::present(readySnapshot());
    TEST_ASSERT_EQUAL_STRING("237 m", view.primary);
    TEST_ASSERT_EQUAL_STRING("N: 42 deg / no arrow", view.detail);
    TEST_ASSERT_EQUAL_STRING("Target:0s  Me:0s", view.ages);
    TEST_ASSERT_TRUE(view.showDistance);
    TEST_ASSERT_FALSE(view.showArrow);
    TEST_ASSERT_EQUAL_UINT32(TARGET_A, view.targetNode);
}

static void test_present_allUnavailableStatesSuppressOldGeometry()
{
    auto snapshot = readySnapshot();
    snapshot.arrowValid = true;
    snapshot.headingSource = NavigationHeadingSource::MAGNETOMETER;
    const NavigationPositionState states[] = {
        NavigationPositionState::NO_LOCAL_POSITION, NavigationPositionState::NO_TARGET_POSITION,
        NavigationPositionState::LOCAL_POSITION_STALE, NavigationPositionState::TARGET_POSITION_STALE,
        NavigationPositionState::POSITION_AGE_UNKNOWN};
    const char *labels[] = {"No local position", "No target position", "Local position old", "Target position old",
                            "Position age unknown"};
    for (size_t i = 0; i < sizeof(states) / sizeof(states[0]); ++i) {
        snapshot.positionState = states[i];
        const auto view = NavigationPresenter::present(snapshot);
        TEST_ASSERT_EQUAL_STRING(labels[i], view.primary);
        TEST_ASSERT_FALSE(view.showDistance);
        TEST_ASSERT_FALSE(view.showArrow);
    }
}

static void test_present_unknownAgeIsNotZero()
{
    auto snapshot = readySnapshot();
    snapshot.positionState = NavigationPositionState::POSITION_AGE_UNKNOWN;
    snapshot.targetPositionAgeKnown = false;
    snapshot.localPositionAgeSeconds = 90;
    TEST_ASSERT_EQUAL_STRING("Target:?  Me:1m", NavigationPresenter::present(snapshot).ages);
}

static void test_present_ageUnitsStayBounded()
{
    auto snapshot = readySnapshot();
    snapshot.targetPositionAgeSeconds = 3600;
    snapshot.localPositionAgeSeconds = UINT32_MAX;
    TEST_ASSERT_EQUAL_STRING("Target:1h  Me:49710d", NavigationPresenter::present(snapshot).ages);
}

static void test_present_distanceUnitsAndInvalidNumbers()
{
    auto snapshot = readySnapshot();
    snapshot.distanceMeters = 1234;
    TEST_ASSERT_EQUAL_STRING("1.2 km", NavigationPresenter::present(snapshot).primary);
    snapshot.distanceMeters = 1610;
    TEST_ASSERT_EQUAL_STRING("1.0 mi", NavigationPresenter::present(snapshot, true).primary);
    snapshot.distanceMeters = 100;
    TEST_ASSERT_EQUAL_STRING("328 ft", NavigationPresenter::present(snapshot, true).primary);
    for (float invalid : {NAN, INFINITY, -1.0f}) {
        snapshot.distanceMeters = invalid;
        TEST_ASSERT_FALSE(NavigationPresenter::present(snapshot).showDistance);
        TEST_ASSERT_FALSE(NavigationPresenter::present(snapshot).showArrow);
    }
}

static void test_present_warningTextHasPriority()
{
    auto snapshot = readySnapshot();
    snapshot.targetPositionWarning = true;
    TEST_ASSERT_EQUAL_STRING("Target position aging", NavigationPresenter::present(snapshot).detail);
    snapshot.localPositionWarning = true;
    TEST_ASSERT_EQUAL_STRING("Both positions aging", NavigationPresenter::present(snapshot).detail);
    snapshot.targetPositionWarning = false;
    TEST_ASSERT_EQUAL_STRING("Local position aging", NavigationPresenter::present(snapshot).detail);
}

static void test_present_arrowRequiresValidHeadingAndNonzeroDistance()
{
    auto snapshot = readySnapshot();
    snapshot.arrowValid = true;
    snapshot.relativeBearingDegrees = -90;
    TEST_ASSERT_FALSE(NavigationPresenter::present(snapshot).showArrow);
    snapshot.headingSource = NavigationHeadingSource::MOVEMENT_COURSE;
    auto view = NavigationPresenter::present(snapshot);
    TEST_ASSERT_TRUE(view.showArrow);
    TEST_ASSERT_EQUAL_FLOAT(-90, view.arrowDegrees);
    TEST_ASSERT_EQUAL_STRING("Movement direction", view.detail);
    snapshot.relativeBearingDegrees = NAN;
    TEST_ASSERT_FALSE(NavigationPresenter::present(snapshot).showArrow);
    snapshot.relativeBearingDegrees = 0;
    snapshot.distanceMeters = 0;
    view = NavigationPresenter::present(snapshot);
    TEST_ASSERT_FALSE(view.showArrow);
    TEST_ASSERT_EQUAL_STRING("Same coordinates", view.detail);
}

static void test_selection_rejectsMissingIgnoredSelfAndBroadcast()
{
    seedPositions();
    TEST_ASSERT_TRUE(graphics::NavigationRenderer::selectTarget(TARGET_A));
    TEST_ASSERT_FALSE(graphics::NavigationRenderer::selectTarget(TARGET_B));
    TEST_ASSERT_FALSE(graphics::NavigationRenderer::selectTarget(database->getNodeNum()));
    TEST_ASSERT_FALSE(graphics::NavigationRenderer::selectTarget(NODENUM_BROADCAST));
    auto *ignored = database->getOrCreateMeshNode(TARGET_B);
    nodeInfoLiteSetBit(ignored, NODEINFO_BITFIELD_IS_IGNORED_MASK, true);
    TEST_ASSERT_FALSE(graphics::NavigationRenderer::selectTarget(TARGET_B));
    TEST_ASSERT_EQUAL_UINT32(TARGET_A, navigation->getTarget());
}

static void test_flow_positionsSelectionTimeoutAndRecovery()
{
    seedPositions();
    TEST_ASSERT_TRUE(graphics::NavigationRenderer::selectTarget(TARGET_A));
    auto view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_EQUAL_STRING("111 m", view.primary);
    TEST_ASSERT_FALSE(view.showArrow);
    Time::advanceTestMillis(120000);
    TEST_ASSERT_EQUAL_STRING("Both positions aging", graphics::NavigationRenderer::currentView().detail);
    Time::advanceTestMillis(480000);
    view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_EQUAL_STRING("Local position old", view.primary);
    TEST_ASSERT_FALSE(view.showDistance);
    seedPositions();
    TEST_ASSERT_TRUE(graphics::NavigationRenderer::currentView().showDistance);
    navigation->clearTarget();
    TEST_ASSERT_EQUAL_STRING("No target", graphics::NavigationRenderer::currentView().primary);
}

static void test_flow_switchTargetNeverShowsPreviousDistance()
{
    seedPositions();
    TEST_ASSERT_TRUE(graphics::NavigationRenderer::selectTarget(TARGET_A));
    database->getOrCreateMeshNode(TARGET_B);
    TEST_ASSERT_TRUE(graphics::NavigationRenderer::selectTarget(TARGET_B));
    const auto view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_EQUAL_UINT32(TARGET_B, view.targetNode);
    TEST_ASSERT_EQUAL_STRING("No target position", view.primary);
    TEST_ASSERT_FALSE(view.showDistance);
}

static void test_render_smallAndLargeFramesAndLongTitle()
{
    for (auto size : {std::pair<uint16_t, uint16_t>{128, 64}, {240, 135}}) {
        MemoryDisplay display(size.first, size.second);
        TEST_ASSERT_TRUE(display.init());
        auto view = NavigationPresenter::present(readySnapshot());
        display.clear();
        graphics::NavigationRenderer::drawView(&display, view, "Find: Elif - North forest collection team");
        TEST_ASSERT_TRUE(display.hasPixels());
        preview(display, size.first == 128 ? "ready-128" : "ready-240");
        auto stale = readySnapshot();
        stale.positionState = NavigationPositionState::TARGET_POSITION_STALE;
        stale.targetPositionAgeSeconds = 630;
        display.clear();
        graphics::NavigationRenderer::drawView(&display, NavigationPresenter::present(stale), "Find: Elif");
        preview(display, size.first == 128 ? "stale-128" : "stale-240");
    }
}

static void test_render_realServiceFrameChangesAfterTimeout()
{
    MemoryDisplay display;
    TEST_ASSERT_TRUE(display.init());
    OLEDDisplayUiState state{};
    display.clear();
    graphics::NavigationRenderer::drawFrame(&display, &state, 0, 0);
    preview(display, "empty-128");
    seedPositions();
    TEST_ASSERT_TRUE(graphics::NavigationRenderer::selectTarget(TARGET_A));
    display.clear();
    graphics::NavigationRenderer::drawFrame(&display, &state, 0, 0);
    uint8_t previous[1024];
    memcpy(previous, display.buffer, sizeof(previous));
    Time::advanceTestMillis(600000);
    display.clear();
    graphics::NavigationRenderer::drawFrame(&display, &state, 0, 0);
    TEST_ASSERT_TRUE(memcmp(previous, display.buffer, sizeof(previous)) != 0);
}

static void test_render_staleAgeLabelSurvivesRedraw()
{
    MemoryDisplay display, expected;
    TEST_ASSERT_TRUE(display.init());
    TEST_ASSERT_TRUE(expected.init());
    auto snapshot = readySnapshot();
    graphics::NavigationRenderer::drawView(&display, NavigationPresenter::present(snapshot), "Find: Elif");
    snapshot.positionState = NavigationPositionState::TARGET_POSITION_STALE;
    snapshot.targetPositionAgeSeconds = 630;
    const auto view = NavigationPresenter::present(snapshot);
    TEST_ASSERT_EQUAL_STRING("Target:10m  Me:0s", view.ages);
    display.clear();
    graphics::NavigationRenderer::drawView(&display, view, "Find: Elif");
    expected.clear();
    expected.setFont(FONT_SMALL_LOCAL);
    expected.setTextAlignment(TEXT_ALIGN_LEFT);
    expected.drawString(2, 49, "Target:10m  Me:0s");
    preview(display, "stale-redraw-128");
    preview(expected, "expected-ages-128");
    for (unsigned y = 49; y < 64; ++y)
        for (unsigned x = 0; x < 128; ++x) {
            const auto offset = x + (y / 8) * 128;
            const auto mask = 1u << (y % 8);
            TEST_ASSERT_EQUAL_UINT8(expected.buffer[offset] & mask, display.buffer[offset] & mask);
        }
}

static void test_render_unsupportedDisplayIsNotOffered()
{
    MemoryDisplay compact(72, 40);
    TEST_ASSERT_FALSE(graphics::NavigationRenderer::supportsDisplay(&compact));
    TEST_ASSERT_FALSE(graphics::NavigationRenderer::supportsDisplay(nullptr));
}
#else
void setUp() {}
void tearDown() {}
#endif

extern "C" void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
#if HAS_SCREEN && !MESHTASTIC_EXCLUDE_POSITIONDB
    RUN_TEST(test_present_noTargetOffersSelection);
    RUN_TEST(test_present_readyWithoutHeadingKeepsNorthBearingNotArrow);
    RUN_TEST(test_present_allUnavailableStatesSuppressOldGeometry);
    RUN_TEST(test_present_unknownAgeIsNotZero);
    RUN_TEST(test_present_ageUnitsStayBounded);
    RUN_TEST(test_present_distanceUnitsAndInvalidNumbers);
    RUN_TEST(test_present_warningTextHasPriority);
    RUN_TEST(test_present_arrowRequiresValidHeadingAndNonzeroDistance);
    RUN_TEST(test_selection_rejectsMissingIgnoredSelfAndBroadcast);
    RUN_TEST(test_flow_positionsSelectionTimeoutAndRecovery);
    RUN_TEST(test_flow_switchTargetNeverShowsPreviousDistance);
    RUN_TEST(test_render_smallAndLargeFramesAndLongTitle);
    RUN_TEST(test_render_realServiceFrameChangesAfterTimeout);
    RUN_TEST(test_render_staleAgeLabelSurvivesRedraw);
    RUN_TEST(test_render_unsupportedDisplayIsNotOffered);
#endif
    exit(UNITY_END());
}

extern "C" void loop() {}
