#include "Arena.h"
#include "../../RocketSim.h"

#include "../Cuda/CudaEngine.h"
#include "../Cuda/GpuTypes.h"
#include "../Cuda/GpuMeshCollision.h"
#include "DropshotTiles/DropshotTiles.h"

#include <map>

RS_NS_START

namespace {
	std::mutex g_gpuArenaCollisionCacheMutex;
	std::map<GameMode, GpuArenaCollisionData*> g_gpuArenaCollisionCache;

	GpuArenaCollisionData* GetOrCreateCachedArenaCollisionData(GameMode gameMode, CudaEngine* cudaEngine) {
		std::lock_guard<std::mutex> lock(g_gpuArenaCollisionCacheMutex);

		auto cacheIt = g_gpuArenaCollisionCache.find(gameMode);
		if (cacheIt != g_gpuArenaCollisionCache.end()) {
			return cacheIt->second;
		}

		const auto& collisionMeshes = RocketSim::GetArenaCollisionMeshes(gameMode);
		if (collisionMeshes.empty()) {
			g_gpuArenaCollisionCache[gameMode] = nullptr;
			return nullptr;
		}

		std::vector<CpuBvhBuilder::Triangle> allTriangles;
		for (const auto& meshFile : collisionMeshes) {
			for (const auto& triIdx : meshFile.tris) {
				const auto& v0 = meshFile.vertices[triIdx.vertexIndexes[0]];
				const auto& v1 = meshFile.vertices[triIdx.vertexIndexes[1]];
				const auto& v2 = meshFile.vertices[triIdx.vertexIndexes[2]];

				CpuBvhBuilder::Triangle tri;
				tri.v0 = { v0.x, v0.y, v0.z };
				tri.v1 = { v1.x, v1.y, v1.z };
				tri.v2 = { v2.x, v2.y, v2.z };

				GpuVec3 e1 = {
					tri.v1.x - tri.v0.x,
					tri.v1.y - tri.v0.y,
					tri.v1.z - tri.v0.z
				};
				GpuVec3 e2 = {
					tri.v2.x - tri.v0.x,
					tri.v2.y - tri.v0.y,
					tri.v2.z - tri.v0.z
				};

				tri.normal = {
					e1.y * e2.z - e1.z * e2.y,
					e1.z * e2.x - e1.x * e2.z,
					e1.x * e2.y - e1.y * e2.x
				};

				float len = sqrtf(tri.normal.x * tri.normal.x +
					tri.normal.y * tri.normal.y +
					tri.normal.z * tri.normal.z);
				if (len > 1e-6f) {
					tri.normal.x /= len;
					tri.normal.y /= len;
					tri.normal.z /= len;
				}
				tri.padding = 0;

				allTriangles.push_back(tri);
			}
		}

		if (allTriangles.empty()) {
			g_gpuArenaCollisionCache[gameMode] = nullptr;
			return nullptr;
		}

		cudaEngine->MakeContextCurrent();

		GpuArenaCollisionData* cacheData = new GpuArenaCollisionData();
		cacheData->triangles = nullptr;
		cacheData->triangleCount = 0;
		cacheData->bvhNodes = nullptr;
		cacheData->bvhNodeCount = 0;
		cacheData->meshes = nullptr;
		cacheData->numMeshes = 0;

		auto stream = cudaEngine->GetStream();
		LoadArenaCollisionMeshesFromTriangles(allTriangles, cacheData, stream);

		if (!cacheData->triangles || !cacheData->bvhNodes) {
			UnloadArenaCollisionMeshes(cacheData);
			delete cacheData;
			return nullptr;
		}

		g_gpuArenaCollisionCache[gameMode] = cacheData;
		return cacheData;
	}
}

void Arena::SetMutatorConfig(const MutatorConfig& mutatorConfig) {

	this->_mutatorConfig = mutatorConfig;
	ball->_SetPhysicsProps(mutatorConfig);
}

Car* Arena::AddCar(Team team, const CarConfig& config) {
	std::lock_guard<std::recursive_mutex> lock(_arenaMutex);

	Car* car = Car::_AllocateCar();
	
	car->config = config;
	car->team = team;
	
	if (!_AddCarFromPtr(car)) {
		delete car;
		RS_ERR_CLOSE("Arena::AddCar(): failed to insert new car (ID collision)");
	}

	car->Respawn(gameMode, -1, _mutatorConfig.carSpawnBoostAmount);

	return car;
}

bool Arena::_AddCarFromPtr(Car* car) {

	car->id = ++_lastCarID;

	if (_carIDMap.find(car->id) == _carIDMap.end()) {
		assert(!_cars.count(car));
		
		_carIDMap[car->id] = car;
		_cars.insert(car);
		return true;

	} else {
		return false;
	}
}

bool Arena::RemoveCar(uint32_t id) {
	std::lock_guard<std::recursive_mutex> lock(_arenaMutex);

	auto itr = _carIDMap.find(id);

	if (itr != _carIDMap.end()) {
		Car* car = itr->second;
		_carIDMap.erase(itr);
		_cars.erase(car);
		if (ownsCars)
			delete car;
		return true;
	} else {
		return false;
	}
}

Car* Arena::GetCar(uint32_t id) {
	auto it = _carIDMap.find(id);
	return (it != _carIDMap.end()) ? it->second : nullptr;
}

void Arena::SetGoalScoreCallback(GoalScoreEventFn callbackFunc, void* userInfo) {
	if (gameMode == GameMode::THE_VOID)
		RS_ERR_CLOSE("Cannot set a goal score callback when on THE_VOID gamemode");

	_goalScoreCallback.func = callbackFunc;
	_goalScoreCallback.userInfo = userInfo;
}

void Arena::SetCarBumpCallback(CarBumpEventFn callbackFunc, void* userInfo) {
	_carBumpCallback.func = callbackFunc;
	_carBumpCallback.userInfo = userInfo;
}

void Arena::ResetToRandomKickoff(int seed) {
	using namespace RLConst;
	// TODO: Make shuffling of kickoff setup more efficient (?)

	static thread_local std::array<int, CAR_SPAWN_LOCATION_AMOUNT> KICKOFF_ORDER_TEMPLATE = { -1 };
	if (KICKOFF_ORDER_TEMPLATE[0] == -1) {
		// Initialize
		for (int i = 0; i < CAR_SPAWN_LOCATION_AMOUNT; i++)
			KICKOFF_ORDER_TEMPLATE[i] = i;
	}

	auto kickoffOrder = KICKOFF_ORDER_TEMPLATE;

	std::default_random_engine* randEngine;
	if (seed == -1) {
		randEngine = &Math::GetRandEngine();
	} else {
		randEngine = new std::default_random_engine(seed);
	}

	int locationAmount = (gameMode == GameMode::HEATSEEKER) ? CAR_SPAWN_LOCATION_AMOUNT_HEATSEEKER : CAR_SPAWN_LOCATION_AMOUNT;

	std::shuffle(kickoffOrder.begin(), kickoffOrder.begin() + locationAmount, *randEngine);

	const CarSpawnPos* CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_SOCCAR;
	const CarSpawnPos* CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_SOCCAR;
	if (gameMode == GameMode::HOOPS) {
		CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_HOOPS;
		CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_HOOPS;
	} else if (gameMode == GameMode::HEATSEEKER) {
		CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_HEATSEEKER;
		CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_SOCCAR;
	} else if (gameMode == GameMode::DROPSHOT) {
		CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_DROPSHOT;
		CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_DROPSHOT;
	}

	std::vector<Car*> blueCars, orangeCars;
	for (Car* car : _cars)
		((car->team == Team::BLUE) ? blueCars : orangeCars).push_back(car);

	int numCarsAtRespawnPos[CAR_RESPAWN_LOCATION_AMOUNT] = {};

	int kickoffPositionAmount = RS_MAX(blueCars.size(), orangeCars.size());
	for (int i = 0; i < kickoffPositionAmount; i++) {

		CarSpawnPos spawnPos;
	
		if (i < locationAmount) {
			spawnPos = CAR_SPAWN_LOCATIONS[RS_MIN(kickoffOrder[i], locationAmount - 1)];
		} else {
			int respawnPosIdx = (i - (locationAmount)) % locationAmount;
			spawnPos = CAR_RESPAWN_LOCATIONS[respawnPosIdx];

			// Extra offset to add to multiple cars spawning at the same respawn point,
			//	helps prevent insane numbers of cars from spawning in eachother.
			// Eventually, they will spawn so far away that they clip out of the arena,
			//	but that's not my problem.
			constexpr float CAR_SPAWN_EXTRA_OFFSET_Y = 250;
			spawnPos.y += CAR_SPAWN_EXTRA_OFFSET_Y * numCarsAtRespawnPos[respawnPosIdx];
			numCarsAtRespawnPos[respawnPosIdx]++;
		}

		for (int teamIndex = 0; teamIndex < 2; teamIndex++) {
			bool isBlue = (teamIndex == 0);
			std::vector<Car*> teamCars = isBlue ? blueCars : orangeCars;

			if (i < teamCars.size()) {
				CarState spawnState;
				spawnState.boost = _mutatorConfig.carSpawnBoostAmount;
				spawnState.pos = { spawnPos.x, spawnPos.y, CAR_SPAWN_REST_Z };
				Angle angle = Angle(spawnPos.yawAng, 0, 0);
				spawnState.isOnGround = true;

				if (!isBlue) {
					spawnState.pos *= { -1, -1, 1 };
					angle.yaw += M_PI;
				}

				spawnState.rotMat = angle.ToRotMat();

				teamCars[i]->SetState(spawnState);
			}
		}
	}

	BallState ballState = BallState();
	if (gameMode == GameMode::HEATSEEKER) {
		int nextRand = (*randEngine)();
		Vec scale = Vec(1, (nextRand % 2) ? 1 : -1, 1);
		ballState.pos = Heatseeker::BALL_START_POS * scale;
		ballState.vel = Heatseeker::BALL_START_VEL * scale;
	} else if (gameMode == GameMode::SNOWDAY) {
		// Don't freeze
		ballState.vel.z = FLT_EPSILON;
	}
	ball->SetState(ballState);

	// Reset boost pads
	for (BoostPad* boostPad : _boostPads)
		boostPad->SetState(BoostPadState());

	// Reset tile states
	if (gameMode == GameMode::DROPSHOT)
		SetDropshotTilesState({});

	if (seed != -1) {
		// Custom random engine was created for this seed, so we need to free it
		delete randEngine;
	}
}

void Arena::SetDropshotTilesState(const DropshotTilesState& state) {
	_dropshotTilesState = state;
}

Arena::Arena(GameMode gameMode, const ArenaConfig& config, float tickRate) : _mutatorConfig(gameMode), _config(config) {

	// Tickrate must be from 15 to 120tps
	assert(tickRate >= 15 && tickRate <= 120);

	RocketSim::AssertInitialized("Cannot create Arena, ");

	this->gameMode = gameMode;
	this->tickTime = 1 / tickRate;

	bool loadArenaStuff = gameMode != GameMode::THE_VOID;

	{ // Initialize ball
		ball = Ball::_AllocBall();
		ball->_SetPhysicsProps(_mutatorConfig);
		ball->SetState(BallState());
	}

	if (loadArenaStuff && gameMode != GameMode::DROPSHOT) { // Initialize boost pads
		using namespace RLConst::BoostPads;

		if (_config.useCustomBoostPads) {
			for (auto& padConfig : _config.customBoostPads) {
				BoostPad* pad = BoostPad::_AllocBoostPad();
				pad->_Setup(padConfig);

				_boostPads.push_back(pad);
			}
		} else {
			bool isHoops = gameMode == GameMode::HOOPS;

			int amountSmall = isHoops ? LOCS_AMOUNT_SMALL_HOOPS : LOCS_AMOUNT_SMALL_SOCCAR;
			_boostPads.reserve(LOCS_AMOUNT_BIG + amountSmall);

			for (int i = 0; i < (LOCS_AMOUNT_BIG + amountSmall); i++) {

				BoostPadConfig padConfig;

				padConfig.isBig = i < LOCS_AMOUNT_BIG;
				if (isHoops) {
					padConfig.pos = padConfig.isBig ? LOCS_BIG_HOOPS[i] : LOCS_SMALL_HOOPS[i - LOCS_AMOUNT_BIG];
				} else {
					padConfig.pos = padConfig.isBig ? LOCS_BIG_SOCCAR[i] : LOCS_SMALL_SOCCAR[i - LOCS_AMOUNT_BIG];
				}

				BoostPad* pad = BoostPad::_AllocBoostPad();
				pad->_Setup(padConfig);

				_boostPads.push_back(pad);
				_boostPadGrid.Add(pad);
			}
		}
	}

	// Initialize CUDA buffers if available
	_InitCudaBuffers();
}

Arena* Arena::Create(GameMode gameMode, const ArenaConfig& arenaConfig, float tickRate) {
	return new Arena(gameMode, arenaConfig, tickRate);
}

void Arena::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(gameMode, tickTime, tickCount, _lastCarID);

	_config.Serialize(out);

	{ // Serialize cars
		out.Write<uint32_t>(_cars.size());
		for (auto car : _cars) {
			out.Write(car->team);
			out.Write(car->id);
			car->Serialize(out);
		}
	}

	if (_boostPads.size() > 0) { // Serialize boost pads
		out.Write<uint32_t>(_boostPads.size());
		for (auto pad : _boostPads)
			pad->GetState().Serialize(out);
	}

	{ // Serialize ball
		ball->GetState().Serialize(out);
	}

	{ // Serialize mutators
		_mutatorConfig.Serialize(out);
	}
}

Arena* Arena::DeserializeNew(DataStreamIn& in) {
	constexpr char ERROR_PREFIX[] = "Arena::Deserialize(): ";

	GameMode gameMode;
	float tickTime;
	uint64_t tickCount;
	uint32_t lastCarID;

	in.ReadMultiple(gameMode, tickTime, tickCount, lastCarID);

	ArenaConfig newConfig = {};
	newConfig.Deserialize(in);

	Arena* newArena = new Arena(gameMode, newConfig, 1.f / tickTime);
	newArena->tickCount = tickCount;
	// Reserve temporary IDs above serialized IDs so per-car insertion order cannot collide.
	newArena->_lastCarID = RS_MAX(newArena->_lastCarID, lastCarID);
	
	{ // Deserialize cars
		uint32_t carAmount = in.Read<uint32_t>();
		for (uint32_t i = 0; i < carAmount; i++) {
			Team team;
			uint32_t id;
			in.Read(team);
			in.Read(id);

#ifndef RS_MAX_SPEED
			if (newArena->_carIDMap.count(id))
				RS_ERR_CLOSE(ERROR_PREFIX << "Failed to load, got repeated car ID of " << id);
#endif

			Car* newCar = newArena->DeserializeNewCar(in, team);

			// Force ID
			newArena->_carIDMap.erase(newCar->id);
			newArena->_carIDMap[id] = newCar;
			newCar->id = id;
		}

		newArena->_lastCarID = lastCarID;
	}

	// Deserialize boost pads
	if (newArena->_boostPads.size() > 0) {
		uint32_t boostPadAmount = in.Read<uint32_t>();

#ifndef RS_MAX_SPEED
		if (boostPadAmount != newArena->_boostPads.size())
			RS_ERR_CLOSE(ERROR_PREFIX << "Failed to load, " <<
				"different boost pad amount written in file (" << boostPadAmount << "/" << newArena->_boostPads.size() << ")");
#endif

		for (auto pad : newArena->_boostPads) {
			BoostPadState padState = BoostPadState();
			padState.Deserialize(in);
			pad->SetState(padState);
		}
	}

	{ // Deserialize ball
		BallState ballState = BallState();
		ballState.Deserialize(in);
		newArena->ball->SetState(ballState);
	}

	{ // Serialize mutators
		newArena->_mutatorConfig.Deserialize(in);
		newArena->SetMutatorConfig(newArena->_mutatorConfig);
	}

	return newArena;
}

Arena* Arena::Clone(bool copyCallbacks) {
	Arena* newArena = new Arena(this->gameMode, this->_config, this->GetTickRate());
	// Reserve temporary IDs above source IDs to avoid collisions while remapping.
	newArena->_lastCarID = this->_lastCarID;
	
	if (copyCallbacks) {
		newArena->_goalScoreCallback = this->_goalScoreCallback;
		newArena->_carBumpCallback = this->_carBumpCallback;
	}

	newArena->ball->SetState(this->ball->GetState());
	newArena->ball->_velocityImpulseCache = this->ball->_velocityImpulseCache;

	for (Car* car : this->_cars) {
		Car* newCar = newArena->AddCar(car->team, car->config);
		
		newCar->SetState(car->GetState());

		// Remap from temporary AddCar ID to source ID while keeping map consistency.
		uint32_t tempID = newCar->id;
		newArena->_carIDMap.erase(tempID);
#ifndef RS_MAX_SPEED
		if (newArena->_carIDMap.count(car->id)) {
			RS_ERR_CLOSE("Arena::Clone(): duplicate car ID during clone remap: " << car->id);
		}
#endif
		newArena->_carIDMap[car->id] = newCar;
		newCar->id = car->id;

		newCar->controls = car->controls;
		newCar->_velocityImpulseCache = car->_velocityImpulseCache;
	}

	assert(this->_boostPads.size() == newArena->_boostPads.size());
	for (int i = 0; i < this->_boostPads.size(); i++)
		newArena->_boostPads[i]->SetState(this->_boostPads[i]->GetState());

	newArena->tickCount = this->tickCount;
	newArena->_lastCarID = this->_lastCarID;

	return newArena;
}

Car* Arena::DeserializeNewCar(DataStreamIn& in, Team team) {
	Car* car = Car::_AllocateCar();
	car->_Deserialize(in);
	car->team = team;

	if (!_AddCarFromPtr(car)) {
		RS_ERR_CLOSE("Arena::DeserializeNewCar(): Failed to insert car while deserializing");
	}

	car->SetState(car->_internalState);

	return car;
}

void Arena::Step(int ticksToSimulate) {
	std::lock_guard<std::recursive_mutex> lock(_arenaMutex);

	// GPU-ONLY MODE - No CPU fallback
	if (!_useCuda) {
		RS_ERR_CLOSE("Arena::Step() - GPU acceleration required but not available! "
					 "CUDA must be initialized before creating arenas.");
	}

	if (ticksToSimulate <= 0)
		return;

	// Capture current controls for every car and queue them for each tick
	// so Step() behaves identically to the old per-tick model.
	std::unordered_map<uint32_t, CarControls> snapshot;
	for (Car* car : _cars)
		snapshot[car->id] = car->controls;
	for (int i = 0; i < ticksToSimulate; i++)
		_actionQueue.push_back(snapshot);

	FlushGPU(ticksToSimulate);
}

// Returns negative: within
// Note that the returned margin is squared
float BallWithinHoopsGoalXYMarginSq(float x, float y) {
	constexpr float
		SCALE_Y = 0.9f,
		OFFSET_Y = 2770.f,
		RADIUS_SQ = 716 * 716;

	float dy = abs(y) * SCALE_Y - OFFSET_Y;
	float distSq = x * x + dy * dy;
	return distSq - RADIUS_SQ;
}

bool Arena::IsBallProbablyGoingIn(float maxTime, float extraMargin, Team* goalTeamOut) const {
	Vec ballPos = ball->_internalState.pos;
	Vec ballVel = ball->_internalState.vel;

	if (gameMode == GameMode::SOCCAR || gameMode == GameMode::SNOWDAY) {
		if (abs(ballVel.y) < FLT_EPSILON)
			return false;

		float scoreDirSgn = RS_SGN(ballVel.y);
		float goalY = _mutatorConfig.goalBaseThresholdY * scoreDirSgn;
		float distToGoal = abs(ballPos.y - goalY);

		float timeToGoal = distToGoal / abs(ballVel.y);
		
		if (timeToGoal > maxTime)
			return false;

		Vec extrapPosWhenScore = ballPos + (ballVel * timeToGoal) + (_mutatorConfig.gravity * timeToGoal * timeToGoal) / 2;

		// From: https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
		constexpr float
			APPROX_GOAL_HALF_WIDTH = 892.755f,
			APPROX_GOAL_HEIGHT = 642.775;

		float scoreMargin = _mutatorConfig.ballRadius * 0.1f + extraMargin;

		if (extrapPosWhenScore.z > APPROX_GOAL_HEIGHT + scoreMargin)
			return false; // Too high

		if (abs(extrapPosWhenScore.x) > APPROX_GOAL_HALF_WIDTH + scoreMargin)
			return false; // Too far to the side

		if (goalTeamOut)
			*goalTeamOut = RS_TEAM_FROM_Y(scoreDirSgn);

		// Ok it's probably gonna score, or at least be very close
		return true;
	} else if (gameMode == GameMode::HOOPS) {

		constexpr float
			APPROX_RIM_HEIGHT = 365;
		
		float minHeight = APPROX_RIM_HEIGHT + _mutatorConfig.ballRadius * 1.2f;

		if (ballVel.z < -FLT_EPSILON && ballPos.z < minHeight) {
			if (BallWithinHoopsGoalXYMarginSq(ballPos.x, ballPos.y) < 0) {
				if (goalTeamOut)
					*goalTeamOut = RS_TEAM_FROM_Y(ballPos.y);
				return true; // Already in the net
			}
		}

		float margin = _mutatorConfig.ballRadius * 1.0f;
		float marginSq = margin * margin;

		float upQuadIntercept;
		float downQuadIntercept;

		// Calculate time to score using quadratic intercept
		{
			float g = _mutatorConfig.gravity.z;
			if (g > -FLT_EPSILON)
				return false; 

			float v = ballVel.z;
			float h = ballPos.z - minHeight;

			float sqrtInput = v * v - 2 * g * h;
			if (sqrtInput > 0) {
				float sqrtOutput = sqrtf(sqrtInput);
				upQuadIntercept = (-v + sqrtOutput) / g;
				downQuadIntercept = (-v - sqrtOutput) / g;
			} else {
				// Never reaches the rim height
				if (BallWithinHoopsGoalXYMarginSq(ballPos.x, ballPos.y) < -marginSq) {
					// If started within the hoop, it will stay within the hoop and is therefore scoring
					return true;
				} else {
					// Otherwise, it can never get into the hoop and is therefore never scoring
					return false;
				}
			}
		}
		
		if (upQuadIntercept >= 0) {
			// Ball has to go up before it can fall into the hoop
			// Make sure it cant hit the rim on the way up

			Vec extrapPosUp = ballPos + (ballVel * upQuadIntercept);
			float upMarginSq = BallWithinHoopsGoalXYMarginSq(extrapPosUp.x, extrapPosUp.y);

			float minClearanceMargin = 60 + _mutatorConfig.ballRadius;

			if (upMarginSq > -marginSq && upMarginSq < (minClearanceMargin * minClearanceMargin))
				return false; // Will probably hit rim
		}

		Vec extrapPosDown = ballPos + (ballVel * downQuadIntercept);
		extrapPosDown.y = abs(extrapPosDown.y);

		{ // Very approximate prediction of backboard bounce
			float wallBounceY = RLConst::ARENA_EXTENT_Y_HOOPS - _mutatorConfig.ballRadius;
			if (extrapPosDown.y > wallBounceY) {
				float margin = extrapPosDown.y - wallBounceY;
				extrapPosDown.y -= margin * (1 + _mutatorConfig.ballWorldRestitution);
			}
		}
		
		if (BallWithinHoopsGoalXYMarginSq(extrapPosDown.x, extrapPosDown.y) < -marginSq) {
			if (goalTeamOut)
				*goalTeamOut = RS_TEAM_FROM_Y(extrapPosDown.y);
			return true;
		} else {
			return false;
		}

	} else {
		RS_ERR_CLOSE("Arena::IsBallProbablyGoingIn() is not supported for gamemode " << GAMEMODE_STRS[(int)gameMode]);
		return false;
	}
}

bool Arena::IsBallScored() const {
	switch (gameMode) {
	case GameMode::SOCCAR:
	case GameMode::HEATSEEKER:
	case GameMode::SNOWDAY:
	{
		float ballPosY = ball->_internalState.pos.y;
		return abs(ballPosY) > (_mutatorConfig.goalBaseThresholdY + _mutatorConfig.ballRadius);
	}
	case GameMode::HOOPS:
	{
		if (ball->_internalState.pos.z < RLConst::HOOPS_GOAL_SCORE_THRESHOLD_Z) {
			constexpr float
				SCALE_Y = 0.9f,
				OFFSET_Y = 2770.f,
				RADIUS_SQ = 716 * 716;

			Vec ballPos = ball->_internalState.pos;
			return BallWithinHoopsGoalXYMarginSq(ballPos.x, ballPos.y) < 0;
		} else {
			return false;
		}
	}
	case GameMode::DROPSHOT:
	{
		if (ball->_internalState.pos.z < -(_mutatorConfig.ballRadius * 1.75f)) {
			return true;
		} else {
			return false;
		}
	}
	default:
		return false;
	}
}

Arena::~Arena() {
	std::lock_guard<std::recursive_mutex> lock(_arenaMutex);

	// Remove all cars
	if (ownsCars) {
		for (Car* car : _cars)
			delete car;
	}

	// Remove the ball
	if (ownsBall) {
		Ball::_DestroyBall(ball);
	}

	if (_boostPads.size() > 0) {
		if (ownsBoostPads) {
			// Remove all boost pads
			for (BoostPad* boostPad : _boostPads)
				delete boostPad;
		}
	}

	_CleanupCudaBuffers();
}

// GPU acceleration implementation

void Arena::_InitCudaBuffers() {
	// Constructor only - no lock needed (unique ownership during init)
	if (_useCuda) return;

	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine) {
		RS_ERR_CLOSE("CUDA engine not initialized! Call RocketSim::InitCuda() before creating arenas.");
	}

	if (!cudaEngine->IsEnabled()) {
		RS_ERR_CLOSE("CUDA engine initialization failed! GPU is required for this build.");
	}

	// Use thread-safe allocations (handles context binding and mutual exclusion)
	_gpuBall = cudaEngine->AllocateDeviceSafe<GpuBallState>(1);
	if (!_gpuBall) {
		RS_ERR_CLOSE("Failed to allocate GPU memory for ball! GPU memory exhausted or driver issue.");
	}

	// Allocate GPU memory for cars (start with capacity for 8 cars)
	_gpuCarsCapacity = 8;
	_gpuCars = cudaEngine->AllocateDeviceSafe<GpuCarState>(_gpuCarsCapacity);

	// Check if allocation succeeded
	if (!_gpuCars) {
		cudaEngine->FreeDeviceSafe<GpuBallState>(_gpuBall);
		RS_ERR_CLOSE("Failed to allocate GPU memory for cars! GPU memory exhausted or driver issue.");
	}

	_useCuda = true;

	// Initial sync to GPU
	_SyncStatesToGPU();

	// Phase 2: Reuse immutable arena collision data per game mode.
	try {
		_gpuArenaCollision = GetOrCreateCachedArenaCollisionData(gameMode, cudaEngine);
	} catch (const std::exception& e) {
		// Non-critical error - GPU collision just won't be available
		RS_WARN("Failed to load arena collision meshes to GPU: " << e.what());
		_gpuArenaCollision = nullptr;
	}
}

void Arena::_CleanupCudaBuffers() {
	// Note: Called from destructor which already holds _arenaMutex
	// Do NOT acquire the lock again (recursive_mutex would waste resources)
	
	if (!_useCuda) return;

	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine || !cudaEngine->IsEnabled()) {
		// Engine no longer available, but buffers were allocated - try to notify
		RS_WARN("CUDA engine not available during cleanup, GPU memory may leak!");
		return;
	}

	// Synchronize GPU to ensure no operations are using this memory
	cudaEngine->Synchronize();

	// Free GPU buffers using thread-safe methods (which also handle context binding)
	if (_gpuBall) {
		cudaEngine->FreeDeviceSafe<GpuBallState>(_gpuBall);
	}

	if (_gpuCars) {
		cudaEngine->FreeDeviceSafe<GpuCarState>(_gpuCars);
		_gpuCarsCapacity = 0;
	}

	// Arena collision data is immutable and shared across arenas for the process lifetime.
	_gpuArenaCollision = nullptr;

	// Action buffer is arena-private — free it.
	if (_gpuActionBuffer) {
		cudaEngine->FreeDeviceSafe<GpuCarControls>(_gpuActionBuffer);
		// FreeDeviceSafe nulls the pointer.
	}
	_gpuActionBufferSlots = 0;
	_actionQueue.clear();

	_useCuda = false;
}

void Arena::_SyncStatesToGPU() {
	if (!_useCuda || !_gpuBall || !_gpuCars) return;

	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine || !cudaEngine->IsEnabled()) {
		RS_ERR_CLOSE("CUDA engine not available during _SyncStatesToGPU()");
	}

	cudaEngine->MakeContextCurrent();
	(void)cudaGetLastError();
	cudaStream_t stream = cudaEngine->GetStreamForArena(this);

	const int numCars = static_cast<int>(_cars.size());
	if (numCars > _gpuCarsCapacity) {
		// Ensure no in-flight kernels are accessing the old car buffer before reallocation.
		cudaEngine->Synchronize();

		// Free old buffer using thread-safe method
		cudaEngine->FreeDeviceSafe<GpuCarState>(_gpuCars);
		_gpuCars = nullptr;

		// Allocate new buffer with expanded capacity using thread-safe method
		_gpuCarsCapacity = RS_MAX(numCars * 2, 8);
		_gpuCars = cudaEngine->AllocateDeviceSafe<GpuCarState>(_gpuCarsCapacity);

		if (!_gpuCars) {
			RS_ERR_CLOSE("Failed to reallocate GPU memory for cars! GPU memory exhausted.");
		}
	}
	
	// Sync ball state
	BallState ballState = ball->_internalState;
	GpuBallState gpuBall = {};
	gpuBall.pos = ToGpuVec3(ballState.pos);
	gpuBall.vel = ToGpuVec3(ballState.vel);
	gpuBall.angVel = ToGpuVec3(ballState.angVel);
	gpuBall.rotMat = ToGpuMat3x3(ballState.rotMat);
	gpuBall.radius = _mutatorConfig.ballRadius;
	gpuBall.mass = _mutatorConfig.ballMass;
	gpuBall.drag = _mutatorConfig.ballDrag;
	gpuBall.friction = _mutatorConfig.ballWorldFriction;
	gpuBall.restitution = _mutatorConfig.ballWorldRestitution;
	gpuBall.maxSpeed = _mutatorConfig.ballMaxSpeed;
	gpuBall.tickCount = tickCount;
	
	// Sync heatseeker info
	gpuBall.hsInfo.yTargetDir = ballState.hsInfo.yTargetDir;
	gpuBall.hsInfo.curTargetSpeed = ballState.hsInfo.curTargetSpeed;
	gpuBall.hsInfo.timeSinceHit = ballState.hsInfo.timeSinceHit;
	
	// Sync dropshot info
	gpuBall.dsInfo.chargeLevel = ballState.dsInfo.chargeLevel;
	gpuBall.dsInfo.accumulatedHitForce = ballState.dsInfo.accumulatedHitForce;
	gpuBall.dsInfo.yTargetDir = ballState.dsInfo.yTargetDir;
	gpuBall.dsInfo.hasDamaged = ballState.dsInfo.hasDamaged;
	gpuBall.dsInfo.lastDamageTick = ballState.dsInfo.lastDamageTick;

	CUDA_CHECK(cudaMemcpyAsync(_gpuBall, &gpuBall, sizeof(GpuBallState), cudaMemcpyHostToDevice, stream));
	
	// Sync car states
	std::vector<GpuCarState> gpuCarsHost;
	gpuCarsHost.resize(numCars);

	int carIdx = 0;
	for (Car* car : _cars) {
		if (carIdx >= numCars)
			break;

		CarState carState = car->_internalState;
		GpuCarState& gpuCar = gpuCarsHost[carIdx];
		gpuCar = {};
		
		gpuCar.pos = ToGpuVec3(carState.pos);
		gpuCar.vel = ToGpuVec3(carState.vel);
		gpuCar.angVel = ToGpuVec3(carState.angVel);
		gpuCar.rotMat = ToGpuMat3x3(carState.rotMat);
		
		// Controls
		gpuCar.throttle = car->controls.throttle;
		gpuCar.steer = car->controls.steer;
		gpuCar.pitch = car->controls.pitch;
		gpuCar.yaw = car->controls.yaw;
		gpuCar.roll = car->controls.roll;
		gpuCar.jump = car->controls.jump;
		gpuCar.boost = car->controls.boost;
		gpuCar.handbrake = car->controls.handbrake;
		
		// State
		gpuCar.isOnGround = carState.isOnGround;
		gpuCar.isDemoed = carState.isDemoed;
		gpuCar.hasJumped = carState.hasJumped;
		gpuCar.hasDoubleJumped = carState.hasDoubleJumped;
		gpuCar.hasFlipped = carState.hasFlipped;
		gpuCar.isJumping = carState.isJumping;
		gpuCar.isFlipping = carState.isFlipping;
		gpuCar.isBoosting = carState.isBoosting;
		gpuCar.jumpTime = carState.jumpTime;
		gpuCar.flipTime = carState.flipTime;
		gpuCar.airTimeSinceJump = carState.airTimeSinceJump;
		gpuCar.boostingTime = carState.boostingTime;
		gpuCar.handbrakeVal = carState.handbrakeVal;
		gpuCar.flipRelTorque = ToGpuVec3(carState.flipRelTorque);
		gpuCar.boost_amount = carState.boost;
		gpuCar.mass = _mutatorConfig.carMass;
		gpuCar.hitboxSize = ToGpuVec3(car->config.hitboxSize);
		gpuCar.hitboxPosOffset = ToGpuVec3(car->config.hitboxPosOffset);
		gpuCar.numWheels = 4;
		gpuCar.tickCount = tickCount;
		
		// Additional state tracking
		gpuCar.isSupersonic = carState.isSupersonic;
		gpuCar.supersonicTime = carState.supersonicTime;
		gpuCar.timeSinceBoosted = carState.timeSinceBoosted;
		gpuCar.airTime = carState.airTime;
		
		// Auto-flip state
		gpuCar.isAutoFlipping = carState.isAutoFlipping;
		gpuCar.autoFlipTimer = carState.autoFlipTimer;
		gpuCar.autoFlipTorqueScale = carState.autoFlipTorqueScale;
		
		// World contact
		gpuCar.worldContact.hasContact = carState.worldContact.hasContact;
		gpuCar.worldContact.contactNormal = ToGpuVec3(carState.worldContact.contactNormal);
		
		// Car contact
		gpuCar.carContact.otherCarID = carState.carContact.otherCarID;
		gpuCar.carContact.cooldownTimer = carState.carContact.cooldownTimer;
		
		// Demo
		gpuCar.demoRespawnTimer = carState.demoRespawnTimer;
		
		// Ball hit info
		gpuCar.ballHitInfo.isValid = carState.ballHitInfo.isValid;
		gpuCar.ballHitInfo.relativePosOnBall = ToGpuVec3(carState.ballHitInfo.relativePosOnBall);
		gpuCar.ballHitInfo.ballPos = ToGpuVec3(carState.ballHitInfo.ballPos);
		gpuCar.ballHitInfo.extraHitVel = ToGpuVec3(carState.ballHitInfo.extraHitVel);
		gpuCar.ballHitInfo.tickCountWhenHit = carState.ballHitInfo.tickCountWhenHit;
		gpuCar.ballHitInfo.tickCountWhenExtraImpulseApplied = carState.ballHitInfo.tickCountWhenExtraImpulseApplied;
		
		// Last controls
		gpuCar.lastControls.throttle = carState.lastControls.throttle;
		gpuCar.lastControls.steer = carState.lastControls.steer;
		gpuCar.lastControls.pitch = carState.lastControls.pitch;
		gpuCar.lastControls.yaw = carState.lastControls.yaw;
		gpuCar.lastControls.roll = carState.lastControls.roll;
		gpuCar.lastControls.jump = carState.lastControls.jump;
		gpuCar.lastControls.boost = carState.lastControls.boost;
		gpuCar.lastControls.handbrake = carState.lastControls.handbrake;
		
		carIdx++;
	}

	if (numCars > 0) {
		CUDA_CHECK(cudaMemcpyAsync(
			_gpuCars,
			gpuCarsHost.data(),
			size_t(numCars) * sizeof(GpuCarState),
			cudaMemcpyHostToDevice,
			stream
		));
	}
}

void Arena::_SyncStatesFromGPU() {
	if (!_useCuda || !_gpuBall || !_gpuCars) return;

	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine || !cudaEngine->IsEnabled()) {
		RS_ERR_CLOSE("CUDA engine not available during _SyncStatesFromGPU()");
	}

	cudaEngine->MakeContextCurrent();
	cudaStream_t stream = cudaEngine->GetStreamForArena(this);

	const int numCars = static_cast<int>(_cars.size());
	if (numCars > _gpuCarsCapacity) {
		RS_ERR_CLOSE("GPU car capacity mismatch in _SyncStatesFromGPU! This should never happen.");
	}

	GpuBallState gpuBall = {};
	CUDA_CHECK(cudaMemcpyAsync(&gpuBall, _gpuBall, sizeof(GpuBallState), cudaMemcpyDeviceToHost, stream));

	std::vector<GpuCarState> gpuCarsHost;
	if (numCars > 0) {
		gpuCarsHost.resize(numCars);
		CUDA_CHECK(cudaMemcpyAsync(
			gpuCarsHost.data(),
			_gpuCars,
			size_t(numCars) * sizeof(GpuCarState),
			cudaMemcpyDeviceToHost,
			stream
		));
	}

	CUDA_CHECK(cudaStreamSynchronize(stream));

	auto isFiniteGpuVec = [](const GpuVec3& v) {
		return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
	};

	auto isReasonableGpuVec = [&](const GpuVec3& v) {
		constexpr float MAX_ABS = 1e7f;
		return isFiniteGpuVec(v) && (fabsf(v.x) < MAX_ABS) && (fabsf(v.y) < MAX_ABS) && (fabsf(v.z) < MAX_ABS);
	};
	
	// Sync ball state back
	if (!isReasonableGpuVec(gpuBall.pos) || !isReasonableGpuVec(gpuBall.vel) || !isReasonableGpuVec(gpuBall.angVel)) {
		RS_ERR_CLOSE("GPU produced invalid ball state (non-finite or out-of-range values)");
	}

	BallState ballState;
	ballState.pos = FromGpuVec3(gpuBall.pos);
	ballState.vel = FromGpuVec3(gpuBall.vel);
	ballState.angVel = FromGpuVec3(gpuBall.angVel);
	ballState.rotMat = FromGpuMat3x3(gpuBall.rotMat);
	
	// Sync heatseeker info back
	ballState.hsInfo.yTargetDir = gpuBall.hsInfo.yTargetDir;
	ballState.hsInfo.curTargetSpeed = gpuBall.hsInfo.curTargetSpeed;
	ballState.hsInfo.timeSinceHit = gpuBall.hsInfo.timeSinceHit;
	
	// Sync dropshot info back
	ballState.dsInfo.chargeLevel = gpuBall.dsInfo.chargeLevel;
	ballState.dsInfo.accumulatedHitForce = gpuBall.dsInfo.accumulatedHitForce;
	ballState.dsInfo.yTargetDir = gpuBall.dsInfo.yTargetDir;
	ballState.dsInfo.hasDamaged = gpuBall.dsInfo.hasDamaged;
	ballState.dsInfo.lastDamageTick = gpuBall.dsInfo.lastDamageTick;
	
	ball->_internalState = ballState;
	ball->_internalState.tickCountSinceUpdate = 0;
	
	// Sync car states back
	int carIdx = 0;
	for (Car* car : _cars) {
		if (carIdx >= numCars) {
			break;
		}
		
		const GpuCarState& gpuCar = gpuCarsHost[carIdx];

		if (!isReasonableGpuVec(gpuCar.pos) || !isReasonableGpuVec(gpuCar.vel) || !isReasonableGpuVec(gpuCar.angVel)) {
			RS_ERR_CLOSE("GPU produced invalid car state (non-finite or out-of-range values) for car id " << car->id);
		}
		
		CarState carState = car->_internalState;
		carState.pos = FromGpuVec3(gpuCar.pos);
		carState.vel = FromGpuVec3(gpuCar.vel);
		carState.angVel = FromGpuVec3(gpuCar.angVel);
		carState.rotMat = FromGpuMat3x3(gpuCar.rotMat);
		carState.isOnGround = gpuCar.isOnGround;
		carState.isDemoed = gpuCar.isDemoed;
		carState.boost = gpuCar.boost_amount;
		
		// Sync all state flags
		carState.hasJumped = gpuCar.hasJumped;
		carState.hasDoubleJumped = gpuCar.hasDoubleJumped;
		carState.hasFlipped = gpuCar.hasFlipped;
		carState.isJumping = gpuCar.isJumping;
		carState.isFlipping = gpuCar.isFlipping;
		carState.isBoosting = gpuCar.isBoosting;
		
		// Sync timers
		carState.jumpTime = gpuCar.jumpTime;
		carState.flipTime = gpuCar.flipTime;
		carState.airTimeSinceJump = gpuCar.airTimeSinceJump;
		carState.boostingTime = gpuCar.boostingTime;
		carState.handbrakeVal = gpuCar.handbrakeVal;
		
		// Sync flip state
		carState.flipRelTorque = FromGpuVec3(gpuCar.flipRelTorque);
		
		// Sync additional state tracking
		carState.isSupersonic = gpuCar.isSupersonic;
		carState.supersonicTime = gpuCar.supersonicTime;
		carState.timeSinceBoosted = gpuCar.timeSinceBoosted;
		carState.airTime = gpuCar.airTime;
		
		// Sync auto-flip state
		carState.isAutoFlipping = gpuCar.isAutoFlipping;
		carState.autoFlipTimer = gpuCar.autoFlipTimer;
		carState.autoFlipTorqueScale = gpuCar.autoFlipTorqueScale;
		
		// Sync world contact
		carState.worldContact.hasContact = gpuCar.worldContact.hasContact;
		carState.worldContact.contactNormal = FromGpuVec3(gpuCar.worldContact.contactNormal);
		
		// Sync car contact
		carState.carContact.otherCarID = gpuCar.carContact.otherCarID;
		carState.carContact.cooldownTimer = gpuCar.carContact.cooldownTimer;
		
		// Sync demo
		carState.demoRespawnTimer = gpuCar.demoRespawnTimer;
		
		// Sync ball hit info
		carState.ballHitInfo.isValid = gpuCar.ballHitInfo.isValid;
		carState.ballHitInfo.relativePosOnBall = FromGpuVec3(gpuCar.ballHitInfo.relativePosOnBall);
		carState.ballHitInfo.ballPos = FromGpuVec3(gpuCar.ballHitInfo.ballPos);
		carState.ballHitInfo.extraHitVel = FromGpuVec3(gpuCar.ballHitInfo.extraHitVel);
		carState.ballHitInfo.tickCountWhenHit = gpuCar.ballHitInfo.tickCountWhenHit;
		carState.ballHitInfo.tickCountWhenExtraImpulseApplied = gpuCar.ballHitInfo.tickCountWhenExtraImpulseApplied;
		
		// Sync last controls
		carState.lastControls.throttle = gpuCar.lastControls.throttle;
		carState.lastControls.steer = gpuCar.lastControls.steer;
		carState.lastControls.pitch = gpuCar.lastControls.pitch;
		carState.lastControls.yaw = gpuCar.lastControls.yaw;
		carState.lastControls.roll = gpuCar.lastControls.roll;
		carState.lastControls.jump = gpuCar.lastControls.jump;
		carState.lastControls.boost = gpuCar.lastControls.boost;
		carState.lastControls.handbrake = gpuCar.lastControls.handbrake;
		
		// Sync wheel contacts
		for (int w = 0; w < 4; w++) {
			carState.wheelsWithContact[w] = gpuCar.wheels[w].hasContact;
		}
		
		car->_internalState = carState;
		car->_internalState.tickCountSinceUpdate = 0;
		
		carIdx++;
	}
}

// ---------------------------------------------------------------------------
// _GetArenaStream
// ---------------------------------------------------------------------------

cudaStream_t Arena::_GetArenaStream() const {
	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine) return 0;
	return cudaEngine->GetStreamForArena(this);
}

// ---------------------------------------------------------------------------
// QueueActions — non-blocking; just accumulates one tick's controls.
// ---------------------------------------------------------------------------

void Arena::QueueActions(const std::unordered_map<uint32_t, CarControls>& carActions) {
	std::lock_guard<std::recursive_mutex> lock(_arenaMutex);
	_actionQueue.push_back(carActions);
}

// ---------------------------------------------------------------------------
// FlushGPU — the core "true GPU ownership" tick path.
//
// Flow:
//   1. Pack host-side action buffer for all ticksToSimulate ticks.
//   2. H2D: current CPU state → GPU (once).
//   3. H2D: all actions → _gpuActionBuffer (once).
//   4. N × (ApplyControlsKernel + LaunchGpuFullPhysicsStep) on the per-arena
//      stream — NO CPU stalls between ticks.
//   5. D2H: final GPU state → CPU (once, with one stream sync).
//   6. CPU gameplay logic (boost pads, callbacks) on the final positions.
// ---------------------------------------------------------------------------

void Arena::FlushGPU(int ticksToSimulate) {
	std::lock_guard<std::recursive_mutex> lock(_arenaMutex);

	if (!_useCuda || !_gpuBall || !_gpuCars) {
		RS_ERR_CLOSE("Arena::FlushGPU() called but GPU buffers not initialized!");
	}

	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine || !cudaEngine->IsEnabled()) {
		RS_ERR_CLOSE("Arena::FlushGPU() — CUDA engine not available!");
	}

	const int numCars  = static_cast<int>(_cars.size());
	const int numTicks = ticksToSimulate;

	// Build an ordered car list matching the GPU array layout used by _SyncStatesToGPU.
	std::vector<uint32_t> carIdOrder;
	carIdOrder.reserve(numCars);
	for (Car* car : _cars)
		carIdOrder.push_back(car->id);

	// --- Pack host-side action buffer -------------------------------------------
	// Layout: [tick0_car0 .. tick0_car(N-1), tick1_car0 .. tick1_car(N-1), ...]
	const int totalSlots = numTicks * RS_MAX(numCars, 1);
	std::vector<GpuCarControls> hostActionBuffer(totalSlots);

	for (int t = 0; t < numTicks; t++) {
		const std::unordered_map<uint32_t, CarControls>* tickActions = nullptr;
		if (t < static_cast<int>(_actionQueue.size()))
			tickActions = &_actionQueue[t];

		for (int c = 0; c < numCars; c++) {
			GpuCarControls& slot = hostActionBuffer[t * numCars + c];
			slot.carArrayIdx = static_cast<uint32_t>(c);

			if (tickActions) {
				auto it = tickActions->find(carIdOrder[c]);
				if (it != tickActions->end()) {
					const CarControls& ctrl = it->second;
					slot.throttle  = ctrl.throttle;
					slot.steer     = ctrl.steer;
					slot.pitch     = ctrl.pitch;
					slot.yaw       = ctrl.yaw;
					slot.roll      = ctrl.roll;
					slot.jump      = ctrl.jump      ? 1 : 0;
					slot.boost     = ctrl.boost     ? 1 : 0;
					slot.handbrake = ctrl.handbrake ? 1 : 0;
					slot._pad      = 0;
					continue;
				}
			}
			// No entry for this car/tick — use zero controls.
			slot.throttle = slot.steer = slot.pitch = slot.yaw = slot.roll = 0.0f;
			slot.jump = slot.boost = slot.handbrake = 0;
			slot._pad = 0;
		}
	}
	_actionQueue.clear();

	// --- Ensure GPU action-buffer capacity --------------------------------------
	if (numCars > 0 && totalSlots > _gpuActionBufferSlots) {
		if (_gpuActionBuffer)
			cudaEngine->FreeDeviceSafe<GpuCarControls>(_gpuActionBuffer);
		_gpuActionBufferSlots = RS_MAX(totalSlots * 2, 64);
		_gpuActionBuffer = cudaEngine->AllocateDeviceSafe<GpuCarControls>(_gpuActionBufferSlots);
		if (!_gpuActionBuffer)
			RS_ERR_CLOSE("Arena::FlushGPU() — failed to allocate GPU action buffer!");
	}

	cudaStream_t stream = _GetArenaStream();

	// --- Step 1: H2D — current CPU arena state → GPU (one copy) ----------------
	_SyncStatesToGPU();  // uses same per-arena stream internally

	// --- Step 2: H2D — actions → GPU (one copy) ---------------------------------
	if (numCars > 0) {
		CUDA_CHECK(cudaMemcpyAsync(
			_gpuActionBuffer,
			hostActionBuffer.data(),
			static_cast<size_t>(totalSlots) * sizeof(GpuCarControls),
			cudaMemcpyHostToDevice,
			stream
		));
	}

	// --- Step 3: N kernels on GPU, no CPU stalls --------------------------------
	cudaEngine->UpdateArenaMultiTick(
		_gpuBall,
		_gpuCars,
		numCars,
		(numCars > 0) ? _gpuActionBuffer : nullptr,
		numTicks,
		tickTime,
		_gpuArenaCollision,
		stream
	);

	// --- Step 4: D2H — final GPU state → CPU (one copy + one stream sync) ------
	_SyncStatesFromGPU();  // issues async D2H then cudaStreamSynchronize internally

	// --- Step 5: CPU-side gameplay logic (uses final positions) -----------------
	const bool hasArenaStuff = (gameMode != GameMode::THE_VOID);
	const bool ballOnly      = _cars.empty();

	if (hasArenaStuff && !ballOnly) {
		// Advance boost pad cooldown timers for all ticks at once.
		for (int i = 0; i < numTicks; i++)
			for (BoostPad* pad : _boostPads)
				pad->_PreTickUpdate(tickTime);
	}

	// Boost pickup collision uses final car positions only.
	// NOTE: pickups that occurred at intermediate positions are not detected —
	// this is a known trade-off of the batched-flush model.
	if (hasArenaStuff) {
		for (Car* car : _cars) {
			if (_config.useCustomBoostPads) {
				for (auto& boostPad : _boostPads)
					boostPad->_CheckCollide(car);
			} else {
				_boostPadGrid.CheckCollision(car);
			}
		}
	}

	if (hasArenaStuff && !ballOnly) {
		for (BoostPad* pad : _boostPads)
			pad->_PostTickUpdate(tickTime, _mutatorConfig);
	}

	// Dropshot tile update.
	if (gameMode == GameMode::DROPSHOT) {
		if (ball->_internalState.dsInfo.lastDamageTick &&
		    ball->_internalState.dsInfo.lastDamageTick == tickCount) {
			SetDropshotTilesState(_dropshotTilesState);
		}
	}

	// Goal-score callback (checked once against final ball position).
	if (_goalScoreCallback.func != nullptr && IsBallScored()) {
		_goalScoreCallback.func(
			this,
			RS_TEAM_FROM_Y(-ball->_internalState.pos.y),
			_goalScoreCallback.userInfo
		);
	}

	tickCount += numTicks;
}

// ---------------------------------------------------------------------------
// SyncToCPU — explicit D2H readback (useful if external code modified GPU state)
// ---------------------------------------------------------------------------

void Arena::SyncToCPU() {
	std::lock_guard<std::recursive_mutex> lock(_arenaMutex);
	if (_useCuda && _gpuBall)
		_SyncStatesFromGPU();
}

RS_NS_END