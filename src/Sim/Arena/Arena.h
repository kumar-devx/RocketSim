#pragma once
#include "../../BaseInc.h"
#include "../Car/Car.h"
#include "../Ball/Ball.h"
#include "../BoostPad/BoostPad.h"
#include "../CollisionMasks.h"

#include "../../CollisionMeshFile/CollisionMeshFile.h"
#include "../BoostPad/BoostPadGrid/BoostPadGrid.h"
#include "../MutatorConfig/MutatorConfig.h"
#include "ArenaConfig/ArenaConfig.h"
#include "DropshotTiles/DropshotTiles.h"

RS_NS_START

// Forward declarations for CUDA types
struct GpuBallState;
struct GpuCarState;
struct GpuArenaCollisionData;

typedef std::function<void(class Arena* arena, Team scoringTeam, void* userInfo)> GoalScoreEventFn;
typedef std::function<void(class Arena* arena, Car* bumper, Car* victim, bool isDemo, void* userInfo)> CarBumpEventFn;

// The container for all game simulation
// Stores cars, the ball, all arena collisions, and manages the overall game state
class RS_API Arena {
public:

	GameMode gameMode;

	uint32_t _lastCarID = 0;
	std::unordered_set<Car*> _cars;
	bool ownsCars = true; // If true, deleting this arena instance deletes all cars

	std::unordered_map<uint32_t, Car*> _carIDMap;
	
	Ball* ball;
	bool ownsBall = true; // If true, deleting this arena instance deletes the ball
	
	std::vector<BoostPad*> _boostPads;
	bool ownsBoostPads = true; // If true, deleing this arena instance deletes all boost pads
	
	BoostPadGrid _boostPadGrid;

	MutatorConfig _mutatorConfig;

	DropshotTilesState _dropshotTilesState;

	const MutatorConfig& GetMutatorConfig() { return _mutatorConfig; }
	void SetMutatorConfig(const MutatorConfig& mutatorConfig);

	// Time in seconds each tick (1/tickrate)
	float tickTime; 

	// Returns (1 / tickTime)
	float GetTickRate() const {
		return 1 / tickTime;
	}

	// Total ticks this arena instance has been simulated for, never resets
	uint64_t tickCount = 0;

	const std::unordered_set<Car*>& GetCars() { return _cars; }
	const std::vector<BoostPad*>& GetBoostPads() { return _boostPads; }

	// Returns true if added, false if car was already added
	bool _AddCarFromPtr(Car* car);
	Car* AddCar(Team team, const CarConfig& config = CAR_CONFIG_OCTANE);

	// Returns false if the car ID was not found in the cars list
	bool RemoveCar(uint32_t id);

	// Returns false if the car was not found in the cars list
	// NOTE: If the car was removed, the car will be freed and the pointer will be made invalid
	bool RemoveCar(Car* car) {
		return RemoveCar(car->id);
	}

	Car* GetCar(uint32_t id);

	GpuArenaCollisionData* _gpuArenaCollision = nullptr;  // GPU mesh collision data (Phase 2)

	struct {
		GoalScoreEventFn func = NULL;
		void* userInfo = NULL;
	} _goalScoreCallback;
	void SetGoalScoreCallback(GoalScoreEventFn callbackFn, void* userInfo = NULL);

	struct {
		CarBumpEventFn func = NULL;
		void* userInfo = NULL;
	} _carBumpCallback;
	void SetCarBumpCallback(CarBumpEventFn callbackFn, void* userInfo = NULL);

	// NOTE: Arena should be destroyed after use
	static Arena* Create(GameMode gameMode, const ArenaConfig& arenaConfig = {}, float tickRate = 120);
	
	// Serialize entire arena state including cars, ball, and boostpads
	void Serialize(DataStreamOut& out) const;

	// Load new arena from serialized data
	static Arena* DeserializeNew(DataStreamIn& in);

	Arena(const Arena& other) = delete; // No copy constructor, use Arena::Clone() instead
	Arena& operator =(const Arena& other) = delete; // No copy operator, use Arena::Clone() instead

	Arena(Arena&& other) = delete; // No move constructor
	Arena& operator =(Arena&& other) = delete; // No move operator

	// Get a deep copy of the arena
	Arena* Clone(bool copyCallbacks);

	// NOTE: Car ID will not be restored
	Car* DeserializeNewCar(DataStreamIn& in, Team team);

	// Simulate everything in the arena for a given number of ticks
	void Step(int ticksToSimulate = 1);

	void ResetToRandomKickoff(int seed = -1);

	// Returns true if the ball is probably going in, does not account for wall or ceiling bounces
	// NOTE: Purposefully overestimates, just like the real RL's shot prediction
	// To check which goal it will score in, use the ball's velocity
	// Margin can be manually adjusted with extraMargin (negative to prevent overestimating)
	bool IsBallProbablyGoingIn(float maxTime = 2.f, float extraMargin = 0, Team* goalTeamOut = NULL) const;

	// Returns true if the ball is in the net
	// Works for all gamemodes (and does nothing in THE_VOID)
	bool IsBallScored() const;

	// Free all associated memory
	~Arena();

	const ArenaConfig& GetArenaConfig() const {
		return _config;
	}

	// Backwards compatability
	ArenaMemWeightMode GetMemWeightMode() {
		return _config.memWeightMode;
	}

	DropshotTilesState GetDropshotTilesState() const { return _dropshotTilesState; };
	void SetDropshotTilesState(const DropshotTilesState& tilesState);

	// CUDA GPU acceleration support
	bool _useCuda = false;
	GpuBallState* _gpuBall = nullptr;
	GpuCarState* _gpuCars = nullptr;
	int _gpuCarsCapacity = 0;
	
	void _InitCudaBuffers();
	void _CleanupCudaBuffers();
	void _SyncStatesToGPU();
	void _SyncStatesFromGPU();
	void _StepGPU(int ticksToSimulate);

private:
	mutable std::recursive_mutex _arenaMutex;
	
	// Constructor for use by Arena::Create()
	Arena(GameMode gameMode, const ArenaConfig& config, float tickRate = 120);

	// Making this private because horrible memory overflows can happen if you changed it
	ArenaConfig _config;
};

RS_NS_END