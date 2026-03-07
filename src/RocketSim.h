#pragma once
#include "Sim/Car/Car.h"
#include "Sim/Ball/Ball.h"
#include "Sim/Arena/Arena.h"

#include "Math/Math.h"

// AVAILABLE DEFS FOR ROCKETSIM:
//	RS_MAX_SPEED: Define this to remove certain sanity checks for faster speed
//	RS_DONT_LOG: Define this to disable all logging output
//	RS_NO_NAMESPACE: Disable the RocketSim namespace encapsulating all RocketSim classes/structs
//	RS_CUDA_ENABLED: Automatically defined when CUDA is enabled in CMake

#ifdef RS_CUDA_ENABLED
#include "Sim/Cuda/CudaEngine.h"
#include "Sim/Cuda/ArenaBatch.h"
#endif

class btBvhTriangleMeshShape;

namespace RocketSim {
	enum class RocketSimStage : byte {
		UNINITIALIZED,
		INITIALIZING,
		INITIALIZED
	};

	typedef std::vector<byte> FileData;

	extern std::filesystem::path _collisionMeshesFolder;
	extern std::mutex _beginInitMutex;

	void Init(std::filesystem::path collisionMeshesFolder, bool silent = false);

	// Instead of loading a collision meshes folder, you can pass in the meshes in this memory-only format
	// The map sorts mesh files to their respective game modes, where each game mode has a list of mesh files
	// The mesh files themselves are just byte arrays
	void InitFromMem(const std::map<GameMode, std::vector<FileData>>& meshFilesMap, bool silent = false);

	void AssertInitialized(const char* errorMsgPrefix);

	RocketSimStage GetStage();

	std::vector<btBvhTriangleMeshShape*>& GetArenaCollisionShapes(GameMode gameMode);

#ifdef RS_CUDA_ENABLED
	// CUDA acceleration interface
	bool InitCuda();
	bool IsCudaEnabled();
	bool TestCudaSetup();

	// Internal CUDA engine accessor (for use by Arena)
	class CudaEngine;
	CudaEngine* GetCudaEngine();
#endif
}