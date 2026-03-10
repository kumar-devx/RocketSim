#include "MathTypes.h"

#include "../Math.h"

RS_NS_START

#define VEC_OP_VEC(op) \
Vec Vec::operator op(const Vec& other) const { return Vec(x op other.x, y op other.y, z op other.z, _w op other._w); } \
Vec& Vec::operator op##=(const Vec& other) { return *this = *this op other; }

#define VEC_OP_FLT(op) \
Vec Vec::operator op(float val) const { return Vec(x op val, y op val, z op val, _w op val); } \
Vec operator op(float val, const Vec& vec) { return Vec(val op vec.x, val op vec.y, val op vec.z, val op vec._w); } \
Vec& Vec::operator op##=(float val) { return *this = *this op val; }

VEC_OP_VEC(+)
VEC_OP_VEC(-)
VEC_OP_VEC(*)
VEC_OP_VEC(/)

VEC_OP_FLT(*)
VEC_OP_FLT(/)

////////////////////////////////////

#define MAT_OP_EACH_MAT(op) \
RotMat RotMat::operator op(const RotMat& other) const { \
	RotMat result; \
	for (int i = 0; i < 3; i++) \
		for (int j = 0; j < 4; j++) \
			result[i][j] = (*this)[i][j] op other[i][j]; \
	return result; \
} \
RotMat& RotMat::operator op##=(const RotMat& other) { \
	return *this = *this op other; \
}

MAT_OP_EACH_MAT(+)
MAT_OP_EACH_MAT(-)

#undef MAT_OP_EACH_MAT

#define MAT_OP_EACH_FLT(op) \
RotMat RotMat::operator op(float val) const { \
	RotMat result; \
	for (int i = 0; i < 3; i++) \
		for (int j = 0; j < 4; j++) \
			result[i][j] = (*this)[i][j] op val; \
	return result; \
} \
RotMat& RotMat::operator op##=(float val) { \
	return *this = *this op val; \
}

MAT_OP_EACH_FLT(*)
MAT_OP_EACH_FLT(/)

#undef MAT_OP_EACH_FLT

//////////////////////////////////////

Angle Angle::FromRotMat(RotMat mat) {
	Angle result = Angle::FromVec(mat.forward);

	Vec baseUp = Vec(0, 0, 1);
	if (fabsf(mat.forward.z) > 0.999f)
		baseUp = Vec(0, 1, 0);

	Vec baseRight = baseUp.Cross(mat.forward).Normalized();
	Vec orthoUp = mat.forward.Cross(baseRight).Normalized();

	float sinRoll = mat.right.Dot(orthoUp);
	float cosRoll = mat.right.Dot(baseRight);
	result.roll = atan2f(sinRoll, cosRoll);
	result.NormalizeFix();
	return result;
}

RotMat Angle::ToRotMat() const {
	Vec fwd = GetForwardVec();

	Vec worldUp = Vec(0, 0, 1);
	if (fabsf(fwd.z) > 0.999f)
		worldUp = Vec(0, 1, 0);

	Vec right = worldUp.Cross(fwd).Normalized();
	Vec up = fwd.Cross(right).Normalized();

	float c = cosf(roll);
	float s = sinf(roll);
	Vec rolledRight = (right * c) + (up * s);
	Vec rolledUp = fwd.Cross(rolledRight).Normalized();

	return RotMat(fwd, rolledRight, rolledUp);
}

Angle Angle::FromVec(const Vec& forward) {
	float yaw, pitch;

	if (abs(forward.y) > FLT_EPSILON || abs(forward.x) > FLT_EPSILON) {
		yaw = atan2f(forward.y, forward.x);

		float dist2D = sqrtf(forward.x * forward.x + forward.y * forward.y);
		pitch = -atan2f(-forward.z, dist2D);
	} else {
		yaw = 0;
		if (forward.z > FLT_EPSILON) {
			pitch = M_PI / 2;
		} else if (forward.z < -FLT_EPSILON) {
			pitch = -M_PI / 2;
		} else {
			pitch = 0;
		}
	}

	return Angle(yaw, pitch, 0);
}

Vec Angle::GetForwardVec() const {
	float
		cp = cosf(-pitch),
		cy = cosf(yaw),
		sy = sinf(yaw),
		sp = sinf(-pitch);

	return Vec(cp * cy, cp * sy, -sp);
}

void Angle::NormalizeFix() {
	yaw = Math::WrapNormalizeFloat(yaw, M_PI);
	pitch = Math::WrapNormalizeFloat(pitch, M_PI / 2);
	roll = Math::WrapNormalizeFloat(roll, M_PI);
}

RS_NS_END