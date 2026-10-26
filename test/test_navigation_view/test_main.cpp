// NavigationPresenter (src/navigation/NavigationViewModel.cpp) and NavigationRenderer
// (src/graphics/draw/NavigationRenderer.cpp) must turn real NavigationService snapshots
// into safe display content. Guard against stale numbers/arrows surviving a state
// change, unknown ages appearing as zero, and selection changing after rejected input.
#include "FSCommon.h"
#include "MeshTypes.h"
#include "TestUtil.h"
#include "UptimeClock.h"
#include "mesh/NodeDB.h"
#include "navigation/CampPointFileStorage.h"
#include "navigation/NavigationService.h"
#include "navigation/NavigationViewModel.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <unity.h>
#include <utility>

#if HAS_SCREEN && !MESHTASTIC_EXCLUDE_POSITIONDB
#include "graphics/Screen.h"
#include "graphics/ScreenFonts.h"
#include "graphics/draw/MenuHandler.h"
#include "graphics/draw/NavigationRenderer.h"
#include "graphics/draw/NotificationRenderer.h"

namespace
{
constexpr NodeNum TARGET_A = 0x73500001;
constexpr NodeNum TARGET_B = 0x73500002;
NodeDB *savedNodeDB;
NavigationService *savedNavigation;
graphics::menuHandler::screenMenus savedMenu;
meshtastic_DeviceState savedDeviceState;
meshtastic_NodeDatabase savedDatabase;
meshtastic_LocalConfig savedConfig;
meshtastic_LocalModuleConfig savedModuleConfig;
meshtastic_ChannelFile savedChannels;
meshtastic_Position savedPosition;
std::unique_ptr<NodeDB> database;
std::unique_ptr<NavigationService> navigation;
CampPointFileStorage campFiles;
CampPointStore campStore(campFiles);

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

void installBanner(const graphics::BannerOverlayOptions &options)
{
    using Renderer = graphics::NotificationRenderer;
    Renderer::resetBanner();
    snprintf(Renderer::alertBannerMessage, sizeof(Renderer::alertBannerMessage), "%s", options.message);
    Renderer::parseBannerMessageWithFonts(Renderer::alertBannerMessage);
    Renderer::current_notification_type = options.notificationType;
    Renderer::optionsArrayPtr = options.optionsArrayPtr;
    Renderer::optionsEnumPtr = options.optionsEnumPtr;
    Renderer::alertBannerOptions = options.optionsCount;
    Renderer::alertBannerCallback = options.bannerCallback;
    Renderer::curSelected = options.InitialSelected;
}

void selectBannerOption(const graphics::BannerOverlayOptions &options, int8_t selected)
{
    // Exercise the same renderer input -> callback -> reset path as a physical SELECT event.
    TEST_ASSERT_NULL(screen.get());
    installBanner(options);
    graphics::NotificationRenderer::curSelected = selected;
    graphics::NotificationRenderer::inEvent.inputEvent = INPUT_BROKER_SELECT;
    MemoryDisplay display;
    TEST_ASSERT_TRUE(display.init());
    graphics::NotificationRenderer::drawAlertBannerOverlay(&display, nullptr);
    TEST_ASSERT_EQUAL(graphics::notificationTypeEnum::none, graphics::NotificationRenderer::current_notification_type);
}
} // namespace

void setUp()
{
    savedNodeDB = nodeDB;
    savedNavigation = navigationService;
    savedMenu = graphics::menuHandler::menuQueue;
    graphics::menuHandler::menuQueue = graphics::menuHandler::MenuNone;
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
    for (const char *path : {"/prefs/camp_point.dat", "/prefs/camp_point.dat.tmp"})
        if (FSCom.exists(path))
            TEST_ASSERT_TRUE(FSCom.remove(path));
    config.position.fixed_position = false;
    navigation = std::make_unique<NavigationService>(*database, &campStore);
    navigation->restoreCamp();
    navigationService = navigation.get();
    config.display.units = meshtastic_Config_DisplayConfig_DisplayUnits_METRIC;
    Time::resetMonotonicForTests();
    Time::setTestMillis(100000);
}

void tearDown()
{
    graphics::NotificationRenderer::resetBanner();
    graphics::menuHandler::menuQueue = savedMenu;
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
    TEST_ASSERT_EQUAL_STRING("Hold: navigation", view.detail);
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
        NavigationPositionState::NO_LOCAL_POSITION,    NavigationPositionState::NO_TARGET_POSITION,
        NavigationPositionState::LOCAL_POSITION_STALE, NavigationPositionState::TARGET_POSITION_STALE,
        NavigationPositionState::POSITION_AGE_UNKNOWN, NavigationPositionState::LOCAL_POSITION_NOT_LOCAL};
    const char *labels[] = {"No local position",   "No target position",   "Local position old",
                            "Target position old", "Position age unknown", "Local GPS needed"};
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

static void test_camp_view_is_saved_point_not_a_live_radio_target()
{
    seedPositions();
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->saveCamp());
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->selectCamp());
    Time::advanceTestMillis(86400000);
    seedPositions();
    const auto view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_TRUE(view.targetIsCamp);
    TEST_ASSERT_EQUAL_UINT32(0, view.targetNode);
    TEST_ASSERT_EQUAL_STRING("Camp saved  Me:0s", view.ages);
    TEST_ASSERT_EQUAL_STRING("Same coordinates", view.detail);
    TEST_ASSERT_TRUE(view.showDistance);
    TEST_ASSERT_FALSE(view.showArrow);
}

static void test_camp_view_local_expiry_and_device_switch_clear_old_content()
{
    seedPositions();
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->saveCamp());
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->selectCamp());
    Time::advanceTestMillis(600000);
    auto view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_TRUE(view.targetIsCamp);
    TEST_ASSERT_EQUAL_STRING("Camp saved  Me:10m", view.ages);
    TEST_ASSERT_EQUAL_STRING("Local position old", view.primary);
    TEST_ASSERT_FALSE(view.showDistance);
    seedPositions();
    TEST_ASSERT_TRUE(graphics::NavigationRenderer::selectTarget(TARGET_A));
    view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_FALSE(view.targetIsCamp);
    TEST_ASSERT_EQUAL_STRING("Target:0s  Me:0s", view.ages);
    TEST_ASSERT_TRUE(view.showDistance);
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->getCampStatus());
}

static void test_camp_feedback_names_the_problem_and_recovery()
{
    TEST_ASSERT_EQUAL_STRING("Camp ready", NavigationPresenter::campFeedback(CampActionStatus::OK));
    TEST_ASSERT_EQUAL_STRING("No camp saved\nChoose Save camp", NavigationPresenter::campFeedback(CampActionStatus::NO_CAMP));
    TEST_ASSERT_EQUAL_STRING("Fresh GPS needed\nWait, then retry",
                             NavigationPresenter::campFeedback(CampActionStatus::LOCAL_POSITION_NOT_FRESH));
    TEST_ASSERT_EQUAL_STRING("Use local GPS\nCheck GPS settings",
                             NavigationPresenter::campFeedback(CampActionStatus::LOCAL_POSITION_NOT_LOCAL));
    TEST_ASSERT_EQUAL_STRING("Camp data damaged\nSave or delete camp",
                             NavigationPresenter::campFeedback(CampActionStatus::CORRUPT));
    TEST_ASSERT_EQUAL_STRING("Newer camp data\nUpdate firmware",
                             NavigationPresenter::campFeedback(CampActionStatus::UNSUPPORTED_VERSION));
    TEST_ASSERT_EQUAL_STRING("Camp not supported\nOn this firmware",
                             NavigationPresenter::campFeedback(CampActionStatus::UNAVAILABLE));
    TEST_ASSERT_EQUAL_STRING("Camp update failed\nReload, then retry",
                             NavigationPresenter::campFeedback(CampActionStatus::STORAGE_ERROR));
}

static void test_camp_render_boot_restore_move_return_and_delete_flow()
{
    seedPositions();
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->saveCamp());
    navigation = std::make_unique<NavigationService>(*database, &campStore);
    navigationService = navigation.get();
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->restoreCamp());
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->selectCamp());
    meshtastic_Position p{};
    p.latitude_i = 410017600;
    p.longitude_i = 290006300;
    p.has_latitude_i = p.has_longitude_i = true;
    database->updatePosition(database->getNodeNum(), p, RX_SRC_LOCAL);
    auto view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_TRUE(view.targetIsCamp);
    TEST_ASSERT_TRUE(view.showDistance);
    TEST_ASSERT_FALSE(view.showArrow);
    for (const auto dimensions : {std::pair<uint16_t, uint16_t>{128, 64}, {240, 135}}) {
        MemoryDisplay display(dimensions.first, dimensions.second);
        TEST_ASSERT_TRUE(display.init());
        display.clear();
        graphics::NavigationRenderer::drawFrame(&display, nullptr, 0, 0);
        TEST_ASSERT_TRUE(display.hasPixels());
        preview(display, dimensions.first == 128 ? "camp-ready-128" : "camp-ready-240");
    }
    Time::advanceTestMillis(600000);
    view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_FALSE(view.showDistance);
    MemoryDisplay stale;
    TEST_ASSERT_TRUE(stale.init());
    stale.clear();
    graphics::NavigationRenderer::drawFrame(&stale, nullptr, 0, 0);
    preview(stale, "camp-stale-128");
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->deleteCamp());
    view = graphics::NavigationRenderer::currentView();
    TEST_ASSERT_FALSE(view.targetIsCamp);
    TEST_ASSERT_FALSE(view.showDistance);
    TEST_ASSERT_EQUAL_STRING("No target", view.primary);
}

static void test_camp_menu_empty_save_go_reload_and_back_actions()
{
    using Menu = graphics::menuHandler;
    seedPositions();
    TEST_ASSERT_TRUE(navigation->selectTarget(TARGET_A));
    auto options = Menu::navigationCampOptions();
    TEST_ASSERT_EQUAL_STRING("Save camp", options.optionsArrayPtr[options.InitialSelected]);
    selectBannerOption(options, 1);
    TEST_ASSERT_EQUAL(Menu::NavigationCampResult, Menu::menuQueue);
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->getCampStatus());
    TEST_ASSERT_EQUAL_STRING("Camp saved", Menu::navigationCampFeedback().message);
    TEST_ASSERT_GREATER_THAN_UINT32(0, Menu::navigationCampFeedback().durationMs);
    TEST_ASSERT_EQUAL_UINT32(TARGET_A, navigation->getTarget());
    options = Menu::navigationCampOptions();
    TEST_ASSERT_EQUAL_STRING("Go to camp", options.optionsArrayPtr[options.InitialSelected]);
    selectBannerOption(options, 0);
    TEST_ASSERT_TRUE(navigation->isCampTarget());
    selectBannerOption(options, 3);
    TEST_ASSERT_EQUAL(Menu::NavigationCampResult, Menu::menuQueue);
    TEST_ASSERT_TRUE(navigation->isCampTarget());
    selectBannerOption(options, 4);
    TEST_ASSERT_EQUAL(Menu::NavigationRootMenu, Menu::menuQueue);
}

static void test_camp_menu_replace_requires_explicit_confirmation()
{
    using Menu = graphics::menuHandler;
    seedPositions();
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->saveCamp());
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->selectCamp());
    meshtastic_Position moved{};
    moved.latitude_i = 410010000;
    moved.longitude_i = 290000000;
    database->updatePosition(database->getNodeNum(), moved, RX_SRC_LOCAL);
    auto menu = Menu::navigationCampOptions();
    TEST_ASSERT_EQUAL_STRING("Replace camp", menu.optionsArrayPtr[1]);
    selectBannerOption(menu, 1);
    TEST_ASSERT_EQUAL(Menu::NavigationCampSaveConfirm, Menu::menuQueue);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 111.2f, navigation->getSnapshot().distanceMeters);
    auto confirm = Menu::navigationCampConfirmation(false);
    TEST_ASSERT_EQUAL_STRING("Keep camp", confirm.optionsArrayPtr[confirm.InitialSelected]);
    confirm.bannerCallback(-1); // Dismissal never writes.
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 111.2f, navigation->getSnapshot().distanceMeters);
    selectBannerOption(confirm, 0);
    TEST_ASSERT_EQUAL(Menu::NavigationCampMenu, Menu::menuQueue);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 111.2f, navigation->getSnapshot().distanceMeters);
    selectBannerOption(Menu::navigationCampConfirmation(false), 1);
    TEST_ASSERT_EQUAL(Menu::NavigationCampResult, Menu::menuQueue);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, navigation->getSnapshot().distanceMeters);
}

static void test_camp_menu_confirmation_rechecks_position_age()
{
    using Menu = graphics::menuHandler;
    seedPositions();
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->saveCamp());
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->selectCamp());
    meshtastic_Position moved{};
    moved.latitude_i = 410010000;
    moved.longitude_i = 290000000;
    database->updatePosition(database->getNodeNum(), moved, RX_SRC_LOCAL);
    auto confirm = Menu::navigationCampConfirmation(false);
    Time::advanceTestMillis(30000);
    selectBannerOption(confirm, 1);
    TEST_ASSERT_EQUAL(Menu::NavigationCampResult, Menu::menuQueue);
    const auto feedback = Menu::navigationCampFeedback();
    TEST_ASSERT_EQUAL_STRING("Fresh GPS needed\nWait, then retry", feedback.message);
    TEST_ASSERT_EQUAL_UINT32(0, feedback.durationMs);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 111.2f, navigation->getSnapshot().distanceMeters);
}

static void test_camp_menu_delete_cancel_timeout_and_confirmation()
{
    using Menu = graphics::menuHandler;
    seedPositions();
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->saveCamp());
    TEST_ASSERT_TRUE(navigation->selectTarget(TARGET_A));
    selectBannerOption(Menu::navigationCampOptions(), 2);
    TEST_ASSERT_EQUAL(Menu::NavigationCampDeleteConfirm, Menu::menuQueue);
    auto confirm = Menu::navigationCampConfirmation(true);
    TEST_ASSERT_EQUAL_STRING("Delete camp", confirm.optionsArrayPtr[1]);
    confirm.bannerCallback(-1);
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->getCampStatus());
    selectBannerOption(confirm, 0);
    TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->getCampStatus());
    selectBannerOption(Menu::navigationCampConfirmation(true), 1);
    TEST_ASSERT_EQUAL(Menu::NavigationCampResult, Menu::menuQueue);
    TEST_ASSERT_EQUAL(CampActionStatus::NO_CAMP, navigation->getCampStatus());
    TEST_ASSERT_EQUAL_UINT32(TARGET_A, navigation->getTarget());
    NavigationService rebooted(*database, &campStore);
    TEST_ASSERT_EQUAL(CampActionStatus::NO_CAMP, rebooted.restoreCamp());
}

static void test_camp_menu_and_confirmation_framebuffers()
{
    using Menu = graphics::menuHandler;
    using Renderer = graphics::NotificationRenderer;
    seedPositions();
    for (const bool deleting : {false, true}) {
        if (deleting)
            TEST_ASSERT_EQUAL(CampActionStatus::OK, navigation->saveCamp());
        installBanner(deleting ? Menu::navigationCampConfirmation(true) : Menu::navigationCampOptions());
        MemoryDisplay display;
        TEST_ASSERT_TRUE(display.init());
        display.clear();
        Renderer::drawAlertBannerOverlay(&display, nullptr);
        TEST_ASSERT_TRUE(display.hasPixels());
        preview(display, deleting ? "camp-delete-menu-128" : "camp-menu-128");
    }
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
    RUN_TEST(test_camp_view_is_saved_point_not_a_live_radio_target);
    RUN_TEST(test_camp_view_local_expiry_and_device_switch_clear_old_content);
    RUN_TEST(test_camp_feedback_names_the_problem_and_recovery);
    RUN_TEST(test_camp_render_boot_restore_move_return_and_delete_flow);
    RUN_TEST(test_camp_menu_empty_save_go_reload_and_back_actions);
    RUN_TEST(test_camp_menu_replace_requires_explicit_confirmation);
    RUN_TEST(test_camp_menu_confirmation_rechecks_position_age);
    RUN_TEST(test_camp_menu_delete_cancel_timeout_and_confirmation);
    RUN_TEST(test_camp_menu_and_confirmation_framebuffers);
#endif
    exit(UNITY_END());
}

extern "C" void loop() {}
