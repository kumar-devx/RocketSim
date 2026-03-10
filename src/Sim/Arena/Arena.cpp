#include "Arena.h"
#include "../../RocketSim.h"

#include "../Cuda/CudaEngine.h"
#include "../Cuda/GpuTypes.h"
#include "../Cuda/GpuMeshCollision.h"
#include "DropshotTiles/DropshotTiles.h"

RS_NS_START

void Arena::SetMutatorConfig(const MutatorConfig& mutatorConfig) {

	this->_mutatorConfig = mutatorConfig;
	ball->_SetPhysicsProps(mutatorConfig);
}

Car* Arena::AddCar(Team team, const CarConfig& config) {
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
	// GPU-ONLY MODE - No CPU fallback
	if (!_useCuda) {
		RS_ERR_CLOSE("Arena::Step() - GPU acceleration required but not available! "
					 "CUDA must be initialized before creating arenas.");
	}

	_StepGPU(ticksToSimulate);
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
	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine) {
		RS_ERR_CLOSE("CUDA engine not initialized! Call RocketSim::InitCuda() before creating arenas.");
	}

	if (!cudaEngine->IsEnabled()) {
		RS_ERR_CLOSE("CUDA engine initialization failed! GPU is required for this build.");
	}

	// Allocate GPU memory for ball (unified memory for easy sync)
	auto& memMgr = cudaEngine->GetMemoryManager();
	_gpuBall = memMgr.AllocateUnified<GpuBallState>(1);

	// Check if allocation succeeded
	if (!_gpuBall) {
		RS_ERR_CLOSE("Failed to allocate GPU memory for ball! GPU memory exhausted or driver issue.");
	}

	// Allocate GPU memory for cars (start with capacity for 8 cars)
	_gpuCarsCapacity = 8;
	_gpuCars = memMgr.AllocateUnified<GpuCarState>(_gpuCarsCapacity);

	// Check if allocation succeeded
	if (!_gpuCars) {
		memMgr.Free(_gpuBall);
		_gpuBall = nullptr;
		RS_ERR_CLOSE("Failed to allocate GPU memory for cars! GPU memory exhausted or driver issue.");
	}

	_useCuda = true;

	// Initial sync to GPU
	_SyncStatesToGPU();

	// Phase 2: Load collision meshes to GPU
	// Build triangles directly from parsed collision mesh files.
	try {
		std::vector<CpuBvhBuilder::Triangle> allTriangles;

		const auto& collisionMeshes = RocketSim::GetArenaCollisionMeshes(gameMode);
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
		
		// Load triangles to GPU
		if (!allTriangles.empty()) {
			_gpuArenaCollision = new GpuArenaCollisionData();
			auto gpuStream = cudaEngine->GetStream();
			LoadArenaCollisionMeshesFromTriangles(allTriangles, _gpuArenaCollision, gpuStream);
		}
	} catch (const std::exception& e) {
		// Non-critical error - GPU collision just won't be available
		RS_WARN("Failed to load arena collision meshes to GPU: " << e.what());
		_gpuArenaCollision = nullptr;
	}
}

void Arena::_CleanupCudaBuffers() {
	if (!_useCuda) return;

	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine) return;

	// CRITICAL: Synchronize GPU to ensure no operations are using this memory
	cudaEngine->Synchronize();

	auto& memMgr = cudaEngine->GetMemoryManager();

	if (_gpuBall) {
		memMgr.Free(_gpuBall);
		_gpuBall = nullptr;
	}

	if (_gpuCars) {
		memMgr.Free(_gpuCars);
		_gpuCars = nullptr;
	}

	// Phase 2: Clean up GPU mesh collision data
	if (_gpuArenaCollision) {
		UnloadArenaCollisionMeshes(_gpuArenaCollision);
		delete _gpuArenaCollision;
		_gpuArenaCollision = nullptr;
	}

	_useCuda = false;
}

void Arena::_SyncStatesToGPU() {
	if (!_useCuda || !_gpuBall || !_gpuCars) return;
	
	// Sync ball state
	BallState ballState = ball->_internalState;
	_gpuBall->pos = ToGpuVec3(ballState.pos);
	_gpuBall->vel = ToGpuVec3(ballState.vel);
	_gpuBall->angVel = ToGpuVec3(ballState.angVel);
	_gpuBall->rotMat = ToGpuMat3x3(ballState.rotMat);
	_gpuBall->radius = _mutatorConfig.ballRadius;
	_gpuBall->mass = _mutatorConfig.ballMass;
	_gpuBall->drag = _mutatorConfig.ballDrag;
	_gpuBall->friction = _mutatorConfig.ballWorldFriction;
	_gpuBall->restitution = _mutatorConfig.ballWorldRestitution;
	_gpuBall->maxSpeed = _mutatorConfig.ballMaxSpeed;
	_gpuBall->tickCount = tickCount;
	
	// Sync heatseeker info
	_gpuBall->hsInfo.yTargetDir = ballState.hsInfo.yTargetDir;
	_gpuBall->hsInfo.curTargetSpeed = ballState.hsInfo.curTargetSpeed;
	_gpuBall->hsInfo.timeSinceHit = ballState.hsInfo.timeSinceHit;
	
	// Sync dropshot info
	_gpuBall->dsInfo.chargeLevel = ballState.dsInfo.chargeLevel;
	_gpuBall->dsInfo.accumulatedHitForce = ballState.dsInfo.accumulatedHitForce;
	_gpuBall->dsInfo.yTargetDir = ballState.dsInfo.yTargetDir;
	_gpuBall->dsInfo.hasDamaged = ballState.dsInfo.hasDamaged;
	_gpuBall->dsInfo.lastDamageTick = ballState.dsInfo.lastDamageTick;
	
	// Sync car states
	int carIdx = 0;
	for (Car* car : _cars) {
		if (carIdx >= _gpuCarsCapacity) {
			// Reallocate if we have more cars than capacity
			auto* cudaEngine = RocketSim::GetCudaEngine();

			// CRITICAL: Synchronize before freeing to avoid in-page errors
			cudaEngine->Synchronize();

			auto& memMgr = cudaEngine->GetMemoryManager();

			memMgr.Free(_gpuCars);
			_gpuCars = nullptr;
			_gpuCarsCapacity = _cars.size() * 2;
			_gpuCars = memMgr.AllocateUnified<GpuCarState>(_gpuCarsCapacity);
			
			// CRITICAL: Check if reallocation succeeded
			if (!_gpuCars) {
				RS_ERR_CLOSE("Failed to reallocate GPU memory for cars! GPU memory exhausted.");
			}
		}
		
		CarState carState = car->_internalState;
		GpuCarState& gpuCar = _gpuCars[carIdx];
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
		gpuCar.ballHitInfo.ticksSinceHit = carState.ballHitInfo.ticksSinceHit;
		gpuCar.ballHitInfo.ballPos = ToGpuVec3(carState.ballHitInfo.ballPos);
		gpuCar.ballHitInfo.distFromBall = carState.ballHitInfo.distFromBall;
		
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
}

void Arena::_SyncStatesFromGPU() {
	if (!_useCuda || !_gpuBall || !_gpuCars) return;

	auto isFiniteGpuVec = [](const GpuVec3& v) {
		return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
	};

	auto isReasonableGpuVec = [&](const GpuVec3& v) {
		constexpr float MAX_ABS = 1e7f;
		return isFiniteGpuVec(v) && (fabsf(v.x) < MAX_ABS) && (fabsf(v.y) < MAX_ABS) && (fabsf(v.z) < MAX_ABS);
	};
	
	// Sync ball state back
	if (!isReasonableGpuVec(_gpuBall->pos) || !isReasonableGpuVec(_gpuBall->vel) || !isReasonableGpuVec(_gpuBall->angVel)) {
		RS_ERR_CLOSE("GPU produced invalid ball state (non-finite or out-of-range values)");
	}

	BallState ballState;
	ballState.pos = FromGpuVec3(_gpuBall->pos);
	ballState.vel = FromGpuVec3(_gpuBall->vel);
	ballState.angVel = FromGpuVec3(_gpuBall->angVel);
	ballState.rotMat = FromGpuMat3x3(_gpuBall->rotMat);
	
	// Sync heatseeker info back
	ballState.hsInfo.yTargetDir = _gpuBall->hsInfo.yTargetDir;
	ballState.hsInfo.curTargetSpeed = _gpuBall->hsInfo.curTargetSpeed;
	ballState.hsInfo.timeSinceHit = _gpuBall->hsInfo.timeSinceHit;
	
	// Sync dropshot info back
	ballState.dsInfo.chargeLevel = _gpuBall->dsInfo.chargeLevel;
	ballState.dsInfo.accumulatedHitForce = _gpuBall->dsInfo.accumulatedHitForce;
	ballState.dsInfo.yTargetDir = _gpuBall->dsInfo.yTargetDir;
	ballState.dsInfo.hasDamaged = _gpuBall->dsInfo.hasDamaged;
	ballState.dsInfo.lastDamageTick = _gpuBall->dsInfo.lastDamageTick;
	
	ball->_internalState = ballState;
	ball->_internalState.tickCountSinceUpdate = 0;
	
	// Sync car states back
	int carIdx = 0;
	for (Car* car : _cars) {
		// Safety check: ensure we don't go out of bounds
		if (carIdx >= _gpuCarsCapacity) {
			RS_ERR_CLOSE("GPU car capacity mismatch in _SyncStatesFromGPU! This should never happen.");
		}
		
		const GpuCarState& gpuCar = _gpuCars[carIdx];

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
		carState.ballHitInfo.ticksSinceHit = gpuCar.ballHitInfo.ticksSinceHit;
		carState.ballHitInfo.ballPos = FromGpuVec3(gpuCar.ballHitInfo.ballPos);
		carState.ballHitInfo.distFromBall = gpuCar.ballHitInfo.distFromBall;
		
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

void Arena::_StepGPU(int ticksToSimulate) {
	auto* cudaEngine = RocketSim::GetCudaEngine();
	if (!cudaEngine || !cudaEngine->IsEnabled()) {
		RS_ERR_CLOSE("GPU acceleration required but CUDA engine not available!");
	}

	for (int i = 0; i < ticksToSimulate; i++) {
		// Sync CPU state to GPU
		_SyncStatesToGPU();
		
		int numCars = static_cast<int>(_cars.size());
		cudaEngine->UpdateArenaBatchFullPhysics(
			_gpuBall,
			_gpuCars,
			numCars,
			tickTime,
			_gpuArenaCollision
		);
		
		// Wait for GPU to finish
		cudaEngine->Synchronize();
		
		// Sync GPU state back to CPU
		_SyncStatesFromGPU();
		
		// Update boost pads (CPU-side only)
		bool hasArenaStuff = (gameMode != GameMode::THE_VOID);
		bool ballOnly = _cars.empty();
		
		if (hasArenaStuff && !ballOnly) {
			for (BoostPad* pad : _boostPads)
				pad->_PreTickUpdate(tickTime);
		}
		
		// Physics integration is GPU-owned. CPU side only handles gameplay logic below.
		
		// Boost pad collision check
		if (hasArenaStuff) {
			for (Car* car : _cars) {
				if (_config.useCustomBoostPads) {
					for (auto& boostPad : _boostPads) {
						boostPad->_CheckCollide(car);
					}
				} else {
					_boostPadGrid.CheckCollision(car);
				}
			}
		}
		
		if (hasArenaStuff && !ballOnly) {
			for (BoostPad* pad : _boostPads)
				pad->_PostTickUpdate(tickTime, _mutatorConfig);
		}
		
		// Dropshot tiles
		if (gameMode == GameMode::DROPSHOT) {
			if (ball->_internalState.dsInfo.lastDamageTick && 
			    ball->_internalState.dsInfo.lastDamageTick == tickCount) {
				SetDropshotTilesState(_dropshotTilesState);
			}
		}
		
		// Goal score callback
		if (_goalScoreCallback.func != NULL) {
			if (IsBallScored()) {
				_goalScoreCallback.func(this, RS_TEAM_FROM_Y(-ball->_internalState.pos.y), _goalScoreCallback.userInfo);
			}
		}
		
		tickCount++;
	}
}

RS_NS_END