#include "Car.h"
#include "../../RLConst.h"
#include "../CollisionMasks.h"

RS_NS_START

CarState Car::GetState() {
	return _internalState;
}

void Car::SetState(const CarState& state) {
	_internalState = state;
	_velocityImpulseCache = { 0, 0, 0 };
	_internalState.tickCountSinceUpdate = 0;
}

void Car::Demolish(float respawnDelay) {
	_internalState.isDemoed = true;
	_internalState.demoRespawnTimer = respawnDelay;
}

void Car::Respawn(GameMode gameMode, int seed, float boostAmount) {
	using namespace RLConst;

	CarState newState = CarState();

	int spawnPosIndex = Math::RandInt(0, CAR_RESPAWN_LOCATION_AMOUNT, seed);
	CarSpawnPos spawnPos = ((gameMode == GameMode::HOOPS) ? CAR_RESPAWN_LOCATIONS_HOOPS : CAR_RESPAWN_LOCATIONS_SOCCAR)[spawnPosIndex];

	newState.pos = Vec(spawnPos.x, spawnPos.y * (team == Team::BLUE ? 1 : -1), CAR_RESPAWN_Z);
	newState.rotMat = Angle(spawnPos.yawAng + (team == Team::BLUE ? 0 : M_PI), 0.f, 0.f).ToRotMat();

	newState.boost = boostAmount;
	this->SetState(newState);
}

void Car::_PreTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig) {
	using namespace RLConst;

#ifndef RS_MAX_SPEED
	// Fix inputs
	controls.ClampFix();
#endif

	if (_internalState.isDemoed) {
		_internalState.demoRespawnTimer = RS_MAX(_internalState.demoRespawnTimer - tickTime, 0);
		if (_internalState.demoRespawnTimer == 0)
			Respawn(gameMode, -1, mutatorConfig.carSpawnBoostAmount);
	}
}

void Car::_PostTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig) {
	(void)gameMode;
	(void)mutatorConfig;

	if (_internalState.isDemoed)
		return;

	{ // Update supersonic
		float speedSquared = _internalState.vel.LengthSq();

		if (_internalState.isSupersonic && _internalState.supersonicTime < RLConst::SUPERSONIC_MAINTAIN_MAX_TIME) {
			_internalState.isSupersonic =
				(speedSquared >= RLConst::SUPERSONIC_MAINTAIN_MIN_SPEED * RLConst::SUPERSONIC_MAINTAIN_MIN_SPEED);
		} else {
			_internalState.isSupersonic =
				(speedSquared >= RLConst::SUPERSONIC_START_SPEED * RLConst::SUPERSONIC_START_SPEED);
		}

		if (_internalState.isSupersonic) {
			_internalState.supersonicTime += tickTime;
		} else {
			_internalState.supersonicTime = 0;
		}
	}

	// Update car contact cooldown timer
	if (_internalState.carContact.cooldownTimer > 0)
		_internalState.carContact.cooldownTimer = RS_MAX(_internalState.carContact.cooldownTimer - tickTime, 0);

	_internalState.lastControls = controls;
}

void Car::_FinishPhysicsTick(const MutatorConfig& mutatorConfig) {
	(void)mutatorConfig;

	if (_internalState.isDemoed)
		return;

	if (!_velocityImpulseCache.IsZero()) {
		_internalState.vel += _velocityImpulseCache;
		_velocityImpulseCache = { 0, 0, 0 };
	}

	float maxSpeedSq = RLConst::CAR_MAX_SPEED * RLConst::CAR_MAX_SPEED;
	if (_internalState.vel.LengthSq() > maxSpeedSq)
		_internalState.vel = _internalState.vel.Normalized() * RLConst::CAR_MAX_SPEED;

	if (_internalState.angVel.LengthSq() > RLConst::CAR_MAX_ANG_SPEED * RLConst::CAR_MAX_ANG_SPEED)
		_internalState.angVel = _internalState.angVel.Normalized() * RLConst::CAR_MAX_ANG_SPEED;

	_internalState.tickCountSinceUpdate++;
}

bool CarState::HasFlipOrJump() const {
	return 
		isOnGround || 
		(!hasFlipped && !hasDoubleJumped && airTimeSinceJump < RLConst::DOUBLEJUMP_MAX_DELAY);
}

bool CarState::HasFlipReset() const {
	return !isOnGround && HasFlipOrJump() && !hasJumped;
}

bool CarState::GotFlipReset() const {
	return !isOnGround && !hasJumped;
}

void CarState::Serialize(DataStreamOut& out) const {
	ballHitInfo.Serialize(out);

	out.WriteMultiple(
		CARSTATE_SERIALIZATION_FIELDS
	);
}

void CarState::Deserialize(DataStreamIn& in) {

	ballHitInfo.Deserialize(in);

	in.ReadMultiple(
		CARSTATE_SERIALIZATION_FIELDS
	);
}

void Car::Serialize(DataStreamOut& out) {
	out.WriteMultiple(CAR_CONTROLS_SERIALIZATION_FIELDS(controls));
	out.WriteMultiple(CAR_CONFIG_SERIALIZATION_FIELDS(config));
	GetState().Serialize(out);
}

void Car::_Deserialize(DataStreamIn& in) {
	in.ReadMultiple(CAR_CONTROLS_SERIALIZATION_FIELDS(controls));
	in.ReadMultiple(CAR_CONFIG_SERIALIZATION_FIELDS(config));
	CarState newState;
	newState.Deserialize(in);
	_internalState = newState;
}

RS_NS_END

