#pragma once

// BulletLink.h: Includes most basic bullet headers, and defines some convenient typedefs/wrappers/etc.

#include <ostream>
#include <cmath>

// Prefer real Bullet types when available.
#if __has_include(<LinearMath/btQuaternion.h>) && __has_include(<LinearMath/btMatrix3x3.h>)
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>

#elif __has_include("LinearMath/btQuaternion.h") && __has_include("LinearMath/btMatrix3x3.h")
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"

#else

// Minimal compatibility subset for projects that only need btQuaternion/btMatrix3x3
// conversion helpers but do not link Bullet directly.
using btScalar = float;

class btQuaternion {
public:
	btScalar m_x, m_y, m_z, m_w;

	btQuaternion() : m_x(0), m_y(0), m_z(0), m_w(1) {}
	btQuaternion(btScalar x, btScalar y, btScalar z, btScalar w)
		: m_x(x), m_y(y), m_z(z), m_w(w) {}

	btScalar x() const { return m_x; }
	btScalar y() const { return m_y; }
	btScalar z() const { return m_z; }
	btScalar w() const { return m_w; }

	btScalar getX() const { return m_x; }
	btScalar getY() const { return m_y; }
	btScalar getZ() const { return m_z; }
	btScalar getW() const { return m_w; }

	void setValue(btScalar x, btScalar y, btScalar z, btScalar w) {
		m_x = x;
		m_y = y;
		m_z = z;
		m_w = w;
	}
};

class btMatrix3x3 {
public:
	btScalar m[3][3];

	btMatrix3x3() {
		setIdentity();
	}

	explicit btMatrix3x3(const btQuaternion& q) {
		setRotation(q);
	}

	void setIdentity() {
		m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
		m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
	}

	btScalar* operator[](int row) { return m[row]; }
	const btScalar* operator[](int row) const { return m[row]; }

	void setRotation(const btQuaternion& q) {
		const btScalar x = q.x();
		const btScalar y = q.y();
		const btScalar z = q.z();
		const btScalar w = q.w();

		const btScalar xx = x * x;
		const btScalar yy = y * y;
		const btScalar zz = z * z;
		const btScalar xy = x * y;
		const btScalar xz = x * z;
		const btScalar yz = y * z;
		const btScalar wx = w * x;
		const btScalar wy = w * y;
		const btScalar wz = w * z;

		m[0][0] = 1 - 2 * (yy + zz);
		m[0][1] = 2 * (xy - wz);
		m[0][2] = 2 * (xz + wy);

		m[1][0] = 2 * (xy + wz);
		m[1][1] = 1 - 2 * (xx + zz);
		m[1][2] = 2 * (yz - wx);

		m[2][0] = 2 * (xz - wy);
		m[2][1] = 2 * (yz + wx);
		m[2][2] = 1 - 2 * (xx + yy);
	}

	void getRotation(btQuaternion& q) const {
		const btScalar trace = m[0][0] + m[1][1] + m[2][2];
		if (trace > 0) {
			const btScalar s = std::sqrt(trace + 1.0f) * 2.0f;
			q.setValue(
				(m[2][1] - m[1][2]) / s,
				(m[0][2] - m[2][0]) / s,
				(m[1][0] - m[0][1]) / s,
				0.25f * s
			);
		} else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
			const btScalar s = std::sqrt(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.0f;
			q.setValue(
				0.25f * s,
				(m[0][1] + m[1][0]) / s,
				(m[0][2] + m[2][0]) / s,
				(m[2][1] - m[1][2]) / s
			);
		} else if (m[1][1] > m[2][2]) {
			const btScalar s = std::sqrt(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.0f;
			q.setValue(
				(m[0][1] + m[1][0]) / s,
				0.25f * s,
				(m[1][2] + m[2][1]) / s,
				(m[0][2] - m[2][0]) / s
			);
		} else {
			const btScalar s = std::sqrt(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.0f;
			q.setValue(
				(m[0][2] + m[2][0]) / s,
				(m[1][2] + m[2][1]) / s,
				0.25f * s,
				(m[1][0] - m[0][1]) / s
			);
		}
	}
};

class btBvhTriangleMeshShape;

#endif

//  BulletPhysics Units (1m) to Unreal Units (2cm) conversion scale
#define BT_TO_UU (50.f)

// Unreal Units (2cm) to BulletPhysics Units (1m) conversion scale
#define UU_TO_BT (1.f/50.f)

// Enum values for Bullet btCollisionObject userinfo usage
enum : int {
	BT_USERINFO_NONE,

	BT_USERINFO_TYPE_CAR,
	BT_USERINFO_TYPE_BALL,
	BT_USERINFO_TYPE_DROPSHOT_TILE,
};
