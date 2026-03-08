#include "../src/RocketSim.h"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct SkipTest : public std::exception {
	explicit SkipTest(std::string msg) : _msg(std::move(msg)) {}
	const char* what() const noexcept override {
		return _msg.c_str();
	}
	std::string _msg;
};

[[noreturn]] void Fail(const std::string& msg) {
	throw std::runtime_error(msg);
}

void AssertTrue(bool cond, const std::string& msg) {
	if (!cond) {
		Fail(msg);
	}
}

void AssertNear(float a, float b, float eps, const std::string& msg) {
	if (std::fabs(a - b) > eps) {
		Fail(msg + " | expected " + std::to_string(b) + ", got " + std::to_string(a));
	}
}

void AssertVecNear(const RocketSim::Vec& a, const RocketSim::Vec& b, float eps, const std::string& msg) {
	if (a.Dist(b) > eps) {
		Fail(msg + " | vec distance=" + std::to_string(a.Dist(b)));
	}
}

template <typename Fn>
void AssertThrows(Fn&& fn, const std::string& msg, const std::string& contains = "") {
	try {
		fn();
		Fail(msg + " | expected exception");
	} catch (const std::exception& e) {
		if (!contains.empty() && std::string(e.what()).find(contains) == std::string::npos) {
			Fail(msg + " | unexpected exception message: " + std::string(e.what()));
		}
	}
}

bool IsFiniteVec(const RocketSim::Vec& v) {
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

std::filesystem::path FindLocalMeshRoot() {
	std::vector<std::filesystem::path> candidates;

	const std::filesystem::path cwd = std::filesystem::current_path();
	candidates.push_back(cwd / "collision_meshes");
	candidates.push_back(cwd.parent_path() / "collision_meshes");
	candidates.push_back(cwd.parent_path().parent_path() / "collision_meshes");
	candidates.push_back(cwd.parent_path().parent_path().parent_path() / "collision_meshes");
	candidates.push_back(cwd.parent_path().parent_path().parent_path().parent_path() / "collision_meshes");

	const std::filesystem::path srcPath = std::filesystem::path(__FILE__).lexically_normal();
	std::filesystem::path p = srcPath.parent_path();
	for (int i = 0; i < 5 && !p.empty(); i++) {
		candidates.push_back(p / "collision_meshes");
		p = p.parent_path();
	}

	for (const auto& candidate : candidates) {
		std::error_code ec;
		if (!std::filesystem::exists(candidate, ec) || !std::filesystem::is_directory(candidate, ec)) {
			continue;
		}

		const std::filesystem::path soccarDir = candidate / "soccar";
		if (std::filesystem::exists(soccarDir, ec) && std::filesystem::is_directory(soccarDir, ec)) {
			return candidate;
		}
	}

	return {};
}

void EnsureInitialized() {
	using namespace RocketSim;
	if (GetStage() == RocketSimStage::INITIALIZED) {
		return;
	}

	const char* meshDir = std::getenv("ROCKETSIM_MESHES_DIR");
	if (meshDir && meshDir[0] != '\0') {
		std::filesystem::path p(meshDir);
		if (std::filesystem::exists(p)) {
			Init(p, true);
		} else {
			InitFromMem({}, true);
		}
	} else {
		std::filesystem::path fallbackMeshRoot = FindLocalMeshRoot();
		if (!fallbackMeshRoot.empty()) {
			Init(fallbackMeshRoot, true);
		} else {
			InitFromMem({}, true);
		}
	}

	AssertTrue(GetStage() == RocketSimStage::INITIALIZED, "RocketSim-GPU failed to initialize");
}

bool HasArenaMeshes(RocketSim::GameMode mode) {
	EnsureInitialized();
	return !RocketSim::GetArenaCollisionShapes(mode).empty();
}

RocketSim::Arena* MakeVoidArena(float tickRate = 120.0f) {
	EnsureInitialized();
	return RocketSim::Arena::Create(RocketSim::GameMode::THE_VOID, {}, tickRate);
}

RocketSim::Car* FindCarById(const RocketSim::Arena* arena, uint32_t id) {
	for (RocketSim::Car* car : arena->_cars) {
		if (car->id == id) {
			return car;
		}
	}
	return nullptr;
}

struct Snapshot {
	RocketSim::BallState ball;
	std::unordered_map<uint32_t, RocketSim::CarState> cars;
};

Snapshot RunDeterministicScenario() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();

	Car* blue = arena->AddCar(Team::BLUE);
	Car* orange = arena->AddCar(Team::ORANGE);

	BallState b = arena->ball->GetState();
	b.pos = Vec(0, 0, 1200);
	b.vel = Vec(700, -200, 50);
	arena->ball->SetState(b);

	CarState sBlue = blue->GetState();
	sBlue.pos = Vec(-300, -1000, 1000);
	sBlue.vel = Vec(300, 200, 0);
	blue->SetState(sBlue);
	blue->controls.throttle = 1.0f;
	blue->controls.boost = true;

	CarState sOrange = orange->GetState();
	sOrange.pos = Vec(300, 1000, 1000);
	sOrange.vel = Vec(-300, -200, 0);
	orange->SetState(sOrange);
	orange->controls.throttle = -1.0f;
	orange->controls.boost = false;

	arena->Step(120);

	Snapshot out;
	out.ball = arena->ball->GetState();
	for (Car* car : arena->GetCars()) {
		out.cars[car->id] = car->GetState();
	}

	delete arena;
	return out;
}

void TestInitAndStage() {
	using namespace RocketSim;
	EnsureInitialized();
	AssertTrue(GetStage() == RocketSimStage::INITIALIZED, "GetStage should be INITIALIZED");
}

void TestCudaEnabledAfterInit() {
#ifdef RS_CUDA_ENABLED
	EnsureInitialized();
	AssertTrue(RocketSim::IsCudaEnabled(), "CUDA should be enabled after RocketSim::InitFromMem/Init");
#else
	throw SkipTest("RS_CUDA_ENABLED is not defined in this build");
#endif
}

void TestCudaSetupSelfTest() {
#ifdef RS_CUDA_ENABLED
	EnsureInitialized();
	AssertTrue(RocketSim::TestCudaSetup(), "RocketSim::TestCudaSetup returned false");
#else
	throw SkipTest("RS_CUDA_ENABLED is not defined in this build");
#endif
}

void TestVoidArenaCreationBasics() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	AssertTrue(arena != nullptr, "Arena::Create returned null");
	AssertTrue(arena->gameMode == GameMode::THE_VOID, "Game mode should be THE_VOID");
	AssertNear(arena->GetTickRate(), 120.0f, 1e-5f, "Tick rate mismatch");
	AssertTrue(arena->GetBoostPads().empty(), "THE_VOID should not have boost pads");
	AssertTrue(arena->GetCars().empty(), "New arena should not have cars");
	delete arena;
}

void TestReinitIdempotentNoThrow() {
	using namespace RocketSim;
	EnsureInitialized();
	InitFromMem({}, true);
	AssertTrue(GetStage() == RocketSimStage::INITIALIZED, "Stage should remain INITIALIZED after repeated InitFromMem");
#ifdef RS_CUDA_ENABLED
	AssertTrue(RocketSim::IsCudaEnabled(), "CUDA should remain enabled after repeated InitFromMem");
#endif
}

void TestAddGetRemoveCarLifecycle() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* car = arena->AddCar(Team::BLUE);
	AssertTrue(car != nullptr, "AddCar returned null");
	AssertTrue(car->id > 0, "Car id should be > 0");
	AssertTrue(arena->GetCars().size() == 1, "Arena should have one car");
	AssertTrue(arena->GetCar(car->id) == car, "GetCar should return the same car");
	AssertTrue(arena->RemoveCar(car->id), "RemoveCar by id should succeed");
	AssertTrue(arena->GetCars().empty(), "Arena should have no cars after removal");
	delete arena;
}

void TestRemoveCarByPointer() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* car = arena->AddCar(Team::ORANGE);
	AssertTrue(arena->RemoveCar(car), "RemoveCar by pointer should succeed");
	AssertTrue(arena->GetCars().empty(), "Arena should have no cars");
	delete arena;
}

void TestRemoveMissingCarReturnsFalse() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* car = arena->AddCar(Team::BLUE);
	AssertTrue(car != nullptr, "AddCar returned null");
	AssertTrue(!arena->RemoveCar(9999999), "RemoveCar should return false for missing id");
	AssertTrue(arena->GetCars().size() == 1, "Removing missing car should not change arena cars");
	delete arena;
}

void TestAddMultipleCarsUniqueIds() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* a = arena->AddCar(Team::BLUE);
	Car* b = arena->AddCar(Team::ORANGE);
	Car* c = arena->AddCar(Team::BLUE);
	AssertTrue(a->id != b->id && b->id != c->id && a->id != c->id, "Car IDs should be unique");
	AssertTrue(arena->_lastCarID >= c->id, "_lastCarID should track latest id");
	delete arena;
}

void TestBallStateSetGetRoundtrip() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	BallState s = arena->ball->GetState();
	s.pos = Vec(1200.0f, -300.0f, 800.0f);
	s.vel = Vec(500.0f, -200.0f, 50.0f);
	s.angVel = Vec(1.0f, 2.0f, -1.5f);
	s.hsInfo.curTargetSpeed = 1234.0f;
	arena->ball->SetState(s);

	BallState out = arena->ball->GetState();
	AssertVecNear(out.pos, s.pos, 0.05f, "Ball pos roundtrip mismatch");
	AssertVecNear(out.vel, s.vel, 0.05f, "Ball vel roundtrip mismatch");
	AssertVecNear(out.angVel, s.angVel, 0.05f, "Ball angVel roundtrip mismatch");
	AssertNear(out.hsInfo.curTargetSpeed, s.hsInfo.curTargetSpeed, 1e-3f, "Ball hsInfo mismatch");
	delete arena;
}

void TestCarStateSetGetRoundtrip() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* car = arena->AddCar(Team::BLUE);
	CarState s = car->GetState();
	s.pos = Vec(-500.0f, 750.0f, 1200.0f);
	s.vel = Vec(333.0f, -444.0f, 120.0f);
	s.angVel = Vec(0.5f, -0.2f, 1.1f);
	s.boost = 42.0f;
	s.isOnGround = false;
	s.hasJumped = true;
	s.hasDoubleJumped = false;
	car->SetState(s);

	CarState out = car->GetState();
	AssertVecNear(out.pos, s.pos, 0.1f, "Car pos roundtrip mismatch");
	AssertVecNear(out.vel, s.vel, 0.1f, "Car vel roundtrip mismatch");
	AssertVecNear(out.angVel, s.angVel, 0.1f, "Car angVel roundtrip mismatch");
	AssertNear(out.boost, s.boost, 0.05f, "Car boost mismatch");
	AssertTrue(out.hasJumped == s.hasJumped, "Car hasJumped mismatch");
	delete arena;
}

void TestStepIncrementsTickCount() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	const uint64_t start = arena->tickCount;
	arena->Step(5);
	AssertTrue(arena->tickCount == start + 5, "tickCount should increment by Step argument");
	delete arena;
}

void TestBallMovesWithVelocityWhenGravityDisabled() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();

	MutatorConfig cfg = arena->GetMutatorConfig();
	cfg.gravity = Vec(0, 0, 0);
	arena->SetMutatorConfig(cfg);

	BallState s = arena->ball->GetState();
	s.pos = Vec(0, 0, 1000);
	s.vel = Vec(1000, 0, 0);
	arena->ball->SetState(s);

	arena->Step(1);
	BallState out = arena->ball->GetState();
	AssertTrue(out.pos.x > s.pos.x, "Ball x should increase after stepping with positive x velocity");
	delete arena;
}

void TestGravityAffectsBallVelocity() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	BallState s = arena->ball->GetState();
	s.pos = Vec(0, 0, 1000);
	s.vel = Vec(0, 0, 1.0f);
	arena->ball->SetState(s);

	arena->Step(2);
	BallState out = arena->ball->GetState();
	AssertTrue(out.vel.z < s.vel.z, "Gravity should reduce z velocity over time");
	delete arena;
}

void TestMutatorConfigSetGet() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	MutatorConfig cfg = arena->GetMutatorConfig();
	cfg.gravity = Vec(0, 0, -500.0f);
	cfg.ballMass *= 1.2f;
	cfg.bumpForceScale = 1.5f;
	cfg.rechargeBoostEnabled = true;
	cfg.rechargeBoostPerSecond = 20.0f;
	arena->SetMutatorConfig(cfg);

	const MutatorConfig& out = arena->GetMutatorConfig();
	AssertVecNear(out.gravity, cfg.gravity, 1e-4f, "Mutator gravity mismatch");
	AssertNear(out.ballMass, cfg.ballMass, 1e-4f, "Mutator ballMass mismatch");
	AssertNear(out.bumpForceScale, cfg.bumpForceScale, 1e-4f, "Mutator bumpForceScale mismatch");
	AssertTrue(out.rechargeBoostEnabled == cfg.rechargeBoostEnabled, "Mutator rechargeBoostEnabled mismatch");
	delete arena;
}

void TestClonePreservesCoreState() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* car = arena->AddCar(Team::BLUE);
	BallState bs = arena->ball->GetState();
	bs.pos = Vec(10, 20, 30);
	bs.vel = Vec(100, -50, 25);
	arena->ball->SetState(bs);

	CarState cs = car->GetState();
	cs.pos = Vec(300, -200, 1300);
	cs.vel = Vec(250, 10, -5);
	car->SetState(cs);

	arena->Step(3);
	Arena* copy = arena->Clone(false);

	AssertTrue(copy->tickCount == arena->tickCount, "Clone tickCount mismatch");
	AssertTrue(copy->GetCars().size() == arena->GetCars().size(), "Clone car count mismatch");
	AssertVecNear(copy->ball->GetState().pos, arena->ball->GetState().pos, 0.2f, "Clone ball position mismatch");
	AssertTrue(FindCarById(copy, car->id) != nullptr, "Clone should contain car with original ID");

	delete copy;
	delete arena;
}

void TestSerializeDeserializeRoundtrip() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* blue = arena->AddCar(Team::BLUE);
	Car* orange = arena->AddCar(Team::ORANGE);

	BallState bs = arena->ball->GetState();
	bs.pos = Vec(-1000, 200, 1400);
	bs.vel = Vec(600, 0, -50);
	arena->ball->SetState(bs);

	CarState csBlue = blue->GetState();
	csBlue.pos = Vec(-200, -300, 700);
	csBlue.vel = Vec(900, 0, 0);
	blue->SetState(csBlue);

	CarState csOrange = orange->GetState();
	csOrange.pos = Vec(200, 300, 700);
	csOrange.vel = Vec(-900, 0, 0);
	orange->SetState(csOrange);

	arena->Step(4);

	DataStreamOut out;
	arena->Serialize(out);
	AssertTrue(!out.data.empty(), "Serialized arena data should not be empty");

	DataStreamIn in;
	in.data = out.data;
	Arena* restored = Arena::DeserializeNew(in);

	AssertTrue(restored->gameMode == arena->gameMode, "Deserialized gameMode mismatch");
	AssertTrue(restored->tickCount == arena->tickCount, "Deserialized tickCount mismatch");
	AssertTrue(restored->GetCars().size() == arena->GetCars().size(), "Deserialized car count mismatch");
	AssertVecNear(restored->ball->GetState().pos, arena->ball->GetState().pos, 0.3f, "Deserialized ball pos mismatch");
	AssertTrue(FindCarById(restored, blue->id) != nullptr, "Deserialized blue car missing");
	AssertTrue(FindCarById(restored, orange->id) != nullptr, "Deserialized orange car missing");

	delete restored;
	delete arena;
}

void TestVoidScoringQueriesAreFalse() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	AssertTrue(!arena->IsBallScored(), "IsBallScored should be false in THE_VOID");
	// Note: IsBallProbablyGoingIn() is not supported for void mode and will fatal error
	// so we skip that check here
	delete arena;
}

void TestGoalScoreCallbackThrowsInVoid() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	AssertThrows(
		[&]() {
			arena->SetGoalScoreCallback([](Arena*, Team, void*) {}, nullptr);
		},
		"SetGoalScoreCallback should throw in THE_VOID",
		"THE_VOID"
	);
	delete arena;
}

void TestIsBallProbablyGoingInThrowsInVoid() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Team teamOut = Team::BLUE;
	AssertThrows(
		[&]() {
			(void)arena->IsBallProbablyGoingIn(2.0f, 0.0f, &teamOut);
		},
		"IsBallProbablyGoingIn should throw in THE_VOID",
		"not supported"
	);
	delete arena;
}

void TestSoccarScoringQueriesWhenMeshesAvailable() {
	using namespace RocketSim;
	if (!HasArenaMeshes(GameMode::SOCCAR)) {
		throw SkipTest("Soccar meshes not available (set ROCKETSIM_MESHES_DIR to a folder containing soccar/*.cmf)");
	}

	Arena* arena = Arena::Create(GameMode::SOCCAR);
	AssertTrue(!arena->IsBallScored(), "Fresh soccar arena should not be scored");

	Team teamOut = Team::BLUE;
	bool goingIn = arena->IsBallProbablyGoingIn(2.0f, 0.0f, &teamOut);
	AssertTrue(goingIn == false, "Fresh kickoff ball should not be probably going in");
	AssertTrue(teamOut == Team::BLUE || teamOut == Team::ORANGE, "Goal team output should be a valid team enum");

	delete arena;
}

void TestCarStateHelpers() {
	using namespace RocketSim;
	CarState s;
	AssertTrue(s.HasFlipOrJump(), "Default car state should have flip or jump");

	s.isOnGround = false;
	s.hasJumped = false;
	s.hasFlipped = false;
	s.hasDoubleJumped = false;
	s.airTimeSinceJump = 0.1f;
	AssertTrue(s.HasFlipOrJump(), "Airborne fresh state should still have flip or jump");
	AssertTrue(s.HasFlipReset(), "Should report flip reset when airborne and unjumped");
	AssertTrue(s.GotFlipReset(), "Should report got flip reset when airborne and unjumped");

	s.hasFlipped = true;
	AssertTrue(!s.HasFlipOrJump(), "Flipped airborne state should have no flip/jump");
}

void TestBallStateMatchesMargins() {
	using namespace RocketSim;
	BallState a;
	BallState b;
	a.pos = Vec(0, 0, 0);
	a.vel = Vec(100, 0, 0);
	a.angVel = Vec(1, 1, 1);

	b = a;
	b.pos.x += 0.2f;
	b.vel.y += 0.1f;
	b.angVel.z += 0.01f;

	AssertTrue(a.Matches(b), "BallState::Matches should pass within default margins");
	AssertTrue(!a.Matches(b, 0.01f, 0.01f, 0.001f), "BallState::Matches should fail with tight margins");
}

void TestDemolishAndRespawnCallPath() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* car = arena->AddCar(Team::BLUE);
	car->Demolish(0.2f);
	AssertTrue(car->GetState().isDemoed, "Car should be demoed after Demolish");
	car->Respawn(GameMode::THE_VOID, 123, 33.0f);
	CarState s = car->GetState();
	AssertTrue(!s.isDemoed, "Car should not remain demoed after Respawn");
	AssertNear(s.boost, 33.0f, 0.1f, "Respawn boost amount mismatch");
	delete arena;
}

void TestStepZeroAndNegativeNoTickAdvance() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	const uint64_t startTick = arena->tickCount;
	BallState startBall = arena->ball->GetState();

	arena->Step(0);
	AssertTrue(arena->tickCount == startTick, "Step(0) should not advance tick count");

	arena->Step(-4);
	AssertTrue(arena->tickCount == startTick, "Step(negative) should not advance tick count");

	BallState out = arena->ball->GetState();
	AssertVecNear(out.pos, startBall.pos, 0.001f, "Ball position should remain unchanged when no ticks are simulated");
	delete arena;
}

void TestCloneIsolationAfterMutation() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	Car* car = arena->AddCar(Team::BLUE);

	BallState ballState = arena->ball->GetState();
	ballState.pos = Vec(100, 200, 300);
	arena->ball->SetState(ballState);

	CarState carState = car->GetState();
	carState.pos = Vec(-400, 800, 1200);
	car->SetState(carState);

	Arena* clone = arena->Clone(false);
	Car* clonedCar = clone->GetCar(car->id);
	AssertTrue(clonedCar != nullptr, "Cloned arena should contain cloned car");

	BallState cloneBall = clone->ball->GetState();
	cloneBall.pos = Vec(999, 999, 999);
	clone->ball->SetState(cloneBall);

	CarState cloneCarState = clonedCar->GetState();
	cloneCarState.pos = Vec(777, -777, 777);
	clonedCar->SetState(cloneCarState);

	AssertVecNear(arena->ball->GetState().pos, ballState.pos, 0.001f, "Original ball state should not change when clone mutates");
	AssertVecNear(car->GetState().pos, carState.pos, 0.001f, "Original car state should not change when clone mutates");

	delete clone;
	delete arena;
}

void TestDeserializeCorruptedDataThrows() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	arena->AddCar(Team::BLUE);
	arena->Step(2);

	DataStreamOut out;
	arena->Serialize(out);
	AssertTrue(!out.data.empty(), "Serialized data must not be empty");

	std::vector<byte> truncated = out.data;
	truncated.resize(truncated.size() / 2);

	DataStreamIn in;
	in.data = truncated;

	AssertThrows(
		[&]() {
			Arena* restored = Arena::DeserializeNew(in);
			delete restored;
		},
		"Deserialize should throw on truncated/corrupted data"
	);

	delete arena;
}

void TestHighCarCountStressNoNaNs() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	constexpr int kCarCount = 32;
	for (int i = 0; i < kCarCount; i++) {
		auto team = (i % 2 == 0) ? Team::BLUE : Team::ORANGE;
		Car* car = arena->AddCar(team);
		AssertTrue(car != nullptr, "Failed to add stress-test car");

		CarState cs = car->GetState();
		// Spread cars out to avoid pathological deep-overlap impulses while still stress-testing.
		int col = i % 8;
		int row = i / 8;
		cs.pos = Vec(-2800.0f + col * 800.0f, -1800.0f + row * 1200.0f, 1200.0f);
		cs.vel = Vec((i % 3 - 1) * 150.0f, ((i + 1) % 3 - 1) * 150.0f, 0.0f);
		cs.isOnGround = false;
		car->SetState(cs);
	}

	arena->Step(60);
	AssertTrue(arena->GetCars().size() == kCarCount, "Car count changed unexpectedly after stress step");

	BallState b = arena->ball->GetState();
	AssertTrue(IsFiniteVec(b.pos) && IsFiniteVec(b.vel) && IsFiniteVec(b.angVel), "Ball state contains non-finite values after stress step");
	for (Car* car : arena->GetCars()) {
		CarState c = car->GetState();
		AssertTrue(IsFiniteVec(c.pos) && IsFiniteVec(c.vel) && IsFiniteVec(c.angVel), "Car state contains non-finite values after stress step");
	}

	delete arena;
}

void TestStepStabilityNoNaNs() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	arena->AddCar(Team::BLUE);
	arena->AddCar(Team::ORANGE);
	arena->Step(300);

	BallState b = arena->ball->GetState();
	AssertTrue(IsFiniteVec(b.pos) && IsFiniteVec(b.vel) && IsFiniteVec(b.angVel), "Ball state contains non-finite values");
	for (Car* car : arena->GetCars()) {
		CarState c = car->GetState();
		AssertTrue(IsFiniteVec(c.pos) && IsFiniteVec(c.vel) && IsFiniteVec(c.angVel), "Car state contains non-finite values");
	}
	delete arena;
}

void TestSetCarBumpCallbackNoCrash() {
	using namespace RocketSim;
	Arena* arena = MakeVoidArena();
	int callbackCount = 0;
	arena->SetCarBumpCallback(
		[&callbackCount](Arena*, Car*, Car*, bool, void*) {
			callbackCount++;
		},
		nullptr
	);
	arena->Step(5);
	AssertTrue(callbackCount >= 0, "Callback counter should be valid");
	delete arena;
}

void TestSoccarArenaCreationWhenMeshesAvailable() {
	using namespace RocketSim;
	if (!HasArenaMeshes(GameMode::SOCCAR)) {
		throw SkipTest("Soccar meshes not available (set ROCKETSIM_MESHES_DIR to enable)");
	}

	Arena* arena = Arena::Create(GameMode::SOCCAR);
	AssertTrue(!arena->GetBoostPads().empty(), "Soccar arena should have boost pads");
	int goalCallbacks = 0;
	arena->SetGoalScoreCallback(
		[&goalCallbacks](Arena*, Team, void*) {
			goalCallbacks++;
		},
		nullptr
	);
	arena->Step(30);
	AssertTrue(goalCallbacks >= 0, "Goal callback counter should remain valid");
	delete arena;
}

void TestCustomBoostPadsWhenMeshesAvailable() {
	using namespace RocketSim;
	if (!HasArenaMeshes(GameMode::SOCCAR)) {
		throw SkipTest("Soccar meshes not available for custom boost pad test");
	}

	ArenaConfig cfg;
	cfg.useCustomBoostPads = true;
	cfg.customBoostPads = {
		BoostPadConfig{Vec(-500, 0, 0), true},
		BoostPadConfig{Vec(500, 0, 0), false}
	};

	Arena* arena = Arena::Create(GameMode::SOCCAR, cfg);
	AssertTrue(arena->GetBoostPads().size() == 2, "Custom boost pad count should be exactly 2");
	delete arena;
}

void TestDropshotTileStateSetGetWhenMeshesAvailable() {
	using namespace RocketSim;
	if (!HasArenaMeshes(GameMode::DROPSHOT)) {
		throw SkipTest("Dropshot meshes not available");
	}

	Arena* arena = Arena::Create(GameMode::DROPSHOT);
	DropshotTilesState st = arena->GetDropshotTilesState();
	st.states[0][0].damageState = DropshotTileState::STATE_BROKEN;
	st.states[1][0].damageState = DropshotTileState::STATE_DAMAGED;
	arena->SetDropshotTilesState(st);

	DropshotTilesState out = arena->GetDropshotTilesState();
	AssertTrue(out.states[0][0].damageState == DropshotTileState::STATE_BROKEN, "Dropshot tile 0 state mismatch");
	AssertTrue(out.states[1][0].damageState == DropshotTileState::STATE_DAMAGED, "Dropshot tile mirrored state mismatch");
	delete arena;
}

void TestDeterministicScenarioRepeatability() {
	Snapshot a = RunDeterministicScenario();
	Snapshot b = RunDeterministicScenario();

	AssertVecNear(a.ball.pos, b.ball.pos, 0.05f, "Determinism check ball pos mismatch");
	AssertVecNear(a.ball.vel, b.ball.vel, 0.05f, "Determinism check ball vel mismatch");
	AssertTrue(a.cars.size() == b.cars.size(), "Determinism check car counts differ");
	for (const auto& kv : a.cars) {
		auto it = b.cars.find(kv.first);
		AssertTrue(it != b.cars.end(), "Determinism check missing car id");
		AssertVecNear(kv.second.pos, it->second.pos, 0.1f, "Determinism check car pos mismatch");
		AssertVecNear(kv.second.vel, it->second.vel, 0.1f, "Determinism check car vel mismatch");
	}
}

void TestArenaBatchLifecycle() {
#ifdef RS_CUDA_ENABLED
	using namespace RocketSim;
	EnsureInitialized();
	Arena* a1 = MakeVoidArena();
	Arena* a2 = MakeVoidArena();
	ArenaBatch batch;
	batch.AddArena(a1);
	batch.AddArena(a2);
	AssertTrue(batch.GetNumArenas() == 2, "ArenaBatch should have two arenas");
	batch.RemoveArena(a1);
	AssertTrue(batch.GetNumArenas() == 1, "ArenaBatch should have one arena after removal");
	batch.Clear();
	AssertTrue(batch.GetNumArenas() == 0, "ArenaBatch should be empty after clear");
	delete a1;
	delete a2;
#else
	throw SkipTest("RS_CUDA_ENABLED is not defined in this build");
#endif
}

void TestArenaBatchStepAllMixedCarCounts() {
#ifdef RS_CUDA_ENABLED
	using namespace RocketSim;
	EnsureInitialized();

	Arena* a1 = MakeVoidArena();
	Arena* a2 = MakeVoidArena();
	a1->AddCar(Team::BLUE);
	a1->AddCar(Team::ORANGE);

	ArenaBatch batch;
	batch.AddArena(a1);
	batch.AddArena(a2); // Keep zero-car arena last to exercise edge bounds

	uint64_t start1 = a1->tickCount;
	uint64_t start2 = a2->tickCount;

	batch.StepAll(3);

	AssertTrue(a1->tickCount == start1 + 3, "ArenaBatch should advance populated arena tick count");
	AssertTrue(a2->tickCount == start2 + 3, "ArenaBatch should advance empty arena tick count");

	BallState b1 = a1->ball->GetState();
	BallState b2 = a2->ball->GetState();
	AssertTrue(IsFiniteVec(b1.pos) && IsFiniteVec(b1.vel), "ArenaBatch populated arena ball state should remain finite");
	AssertTrue(IsFiniteVec(b2.pos) && IsFiniteVec(b2.vel), "ArenaBatch empty arena ball state should remain finite");

	delete a1;
	delete a2;
#else
	throw SkipTest("RS_CUDA_ENABLED is not defined in this build");
#endif
}

void TestArenaBatchStepAllOptional() {
#ifdef RS_CUDA_ENABLED
	const char* runBatchStep = std::getenv("ROCKETSIM_RUN_BATCH_STEP_TEST");
	if (!(runBatchStep && std::string(runBatchStep) == "1")) {
		throw SkipTest("Set ROCKETSIM_RUN_BATCH_STEP_TEST=1 to enable batch StepAll test");
	}

	using namespace RocketSim;
	Arena* a1 = MakeVoidArena();
	Arena* a2 = MakeVoidArena();
	ArenaBatch batch;
	batch.AddArena(a1);
	batch.AddArena(a2);
	batch.StepAll(10);
	AssertTrue(a1->tickCount == 10 && a2->tickCount == 10, "ArenaBatch StepAll should advance both arenas");
	batch.Clear();
	delete a1;
	delete a2;
#else
	throw SkipTest("RS_CUDA_ENABLED is not defined in this build");
#endif
}

} // namespace

int main() {
	using TestEntry = std::pair<std::string, std::function<void()>>;
	std::vector<TestEntry> tests = {
		{"InitAndStage", TestInitAndStage},
		{"CudaEnabledAfterInit", TestCudaEnabledAfterInit},
		{"CudaSetupSelfTest", TestCudaSetupSelfTest},
		{"ReinitIdempotentNoThrow", TestReinitIdempotentNoThrow},
		{"VoidArenaCreationBasics", TestVoidArenaCreationBasics},
		{"AddGetRemoveCarLifecycle", TestAddGetRemoveCarLifecycle},
		{"RemoveCarByPointer", TestRemoveCarByPointer},
		{"RemoveMissingCarReturnsFalse", TestRemoveMissingCarReturnsFalse},
		{"AddMultipleCarsUniqueIds", TestAddMultipleCarsUniqueIds},
		{"BallStateSetGetRoundtrip", TestBallStateSetGetRoundtrip},
		{"CarStateSetGetRoundtrip", TestCarStateSetGetRoundtrip},
		{"StepIncrementsTickCount", TestStepIncrementsTickCount},
		{"BallMovesWithVelocityWhenGravityDisabled", TestBallMovesWithVelocityWhenGravityDisabled},
		{"GravityAffectsBallVelocity", TestGravityAffectsBallVelocity},
		{"MutatorConfigSetGet", TestMutatorConfigSetGet},
		{"ClonePreservesCoreState", TestClonePreservesCoreState},
		{"CloneIsolationAfterMutation", TestCloneIsolationAfterMutation},
		{"SerializeDeserializeRoundtrip", TestSerializeDeserializeRoundtrip},
		{"DeserializeCorruptedDataThrows", TestDeserializeCorruptedDataThrows},
		{"VoidScoringQueriesAreFalse", TestVoidScoringQueriesAreFalse},
		{"GoalScoreCallbackThrowsInVoid", TestGoalScoreCallbackThrowsInVoid},
		{"IsBallProbablyGoingInThrowsInVoid", TestIsBallProbablyGoingInThrowsInVoid},
		{"CarStateHelpers", TestCarStateHelpers},
		{"BallStateMatchesMargins", TestBallStateMatchesMargins},
		{"DemolishAndRespawnCallPath", TestDemolishAndRespawnCallPath},
		{"StepZeroAndNegativeNoTickAdvance", TestStepZeroAndNegativeNoTickAdvance},
		{"StepStabilityNoNaNs", TestStepStabilityNoNaNs},
		{"HighCarCountStressNoNaNs", TestHighCarCountStressNoNaNs},
		{"SetCarBumpCallbackNoCrash", TestSetCarBumpCallbackNoCrash},
		{"SoccarScoringQueriesWhenMeshesAvailable", TestSoccarScoringQueriesWhenMeshesAvailable},
		{"SoccarArenaCreationWhenMeshesAvailable", TestSoccarArenaCreationWhenMeshesAvailable},
		{"CustomBoostPadsWhenMeshesAvailable", TestCustomBoostPadsWhenMeshesAvailable},
		{"DropshotTileStateSetGetWhenMeshesAvailable", TestDropshotTileStateSetGetWhenMeshesAvailable},
		{"DeterministicScenarioRepeatability", TestDeterministicScenarioRepeatability},
		{"ArenaBatchLifecycle", TestArenaBatchLifecycle},
		{"ArenaBatchStepAllMixedCarCounts", TestArenaBatchStepAllMixedCarCounts},
		{"ArenaBatchStepAllOptional", TestArenaBatchStepAllOptional},
	};

	int passed = 0;
	int failed = 0;
	int skipped = 0;

	for (const auto& [name, fn] : tests) {
		try {
			std::cout << "[RUN] " << name << '\n';
			std::cout.flush();
			fn();
			passed++;
			std::cout << "[PASS] " << name << '\n';
		} catch (const SkipTest& e) {
			skipped++;
			std::cout << "[SKIP] " << name << " - " << e.what() << '\n';
		} catch (const std::exception& e) {
			failed++;
			std::cerr << "[FAIL] " << name << " - " << e.what() << '\n';
		} catch (...) {
			failed++;
			std::cerr << "[FAIL] " << name << " - unknown exception" << '\n';
		}
	}

	std::cout << "\nSummary: passed=" << passed << ", skipped=" << skipped << ", failed=" << failed << '\n';
	return failed == 0 ? 0 : 1;
}
