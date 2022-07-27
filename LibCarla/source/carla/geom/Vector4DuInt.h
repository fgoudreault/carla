// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/MsgPack.h"

#include <cmath>
#include <limits>

namespace carla {
namespace geom {

  class Vector4DuInt {
  public:

    // =========================================================================
    // -- Public data members --------------------------------------------------
    // =========================================================================

    uint8_t x = 0;

    uint8_t y = 0;

    uint8_t z = 0;

    uint8_t w = 0;

    // =========================================================================
    // -- Constructors ---------------------------------------------------------
    // =========================================================================

    Vector4DuInt() = default;

    Vector4DuInt(uint8_t ix, uint8_t iy, uint8_t iz, uint8_t iw)
      : x(ix),
        y(iy),
        z(iz),
	w(iw) {}

    // =========================================================================
    // -- Other methods --------------------------------------------------------
    // =========================================================================

    uint64_t SquaredLength() const {
      return x * x + y * y + z * z + w * w;
    }

    double Length() const {
       return std::sqrt(SquaredLength());
    }

    // =========================================================================
    // -- Arithmetic operators -------------------------------------------------
    // =========================================================================

    Vector4DuInt &operator+=(const Vector4DuInt &rhs) {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
      w += rhs.w;
      return *this;
    }

    friend Vector4DuInt operator+(Vector4DuInt lhs, const Vector4DuInt &rhs) {
      lhs += rhs;
      return lhs;
    }

    Vector4DuInt &operator-=(const Vector4DuInt &rhs) {
      x -= rhs.x;
      y -= rhs.y;
      z -= rhs.z;
      w -= rhs.w;
      return *this;
    }

    friend Vector4DuInt operator-(Vector4DuInt lhs, const Vector4DuInt &rhs) {
      lhs -= rhs;
      return lhs;
    }

    Vector4DuInt &operator*=(uint8_t rhs) {
      x *= rhs;
      y *= rhs;
      z *= rhs;
      w *= rhs;
      return *this;
    }

    friend Vector4DuInt operator*(Vector4DuInt lhs, uint8_t rhs) {
      lhs *= rhs;
      return lhs;
    }

    friend Vector4DuInt operator*(uint8_t lhs, Vector4DuInt rhs) {
      rhs *= lhs;
      return rhs;
    }

    Vector4DuInt &operator/=(uint8_t rhs) {
      x /= rhs;
      y /= rhs;
      z /= rhs;
      w /= rhs;
      return *this;
    }

    friend Vector4DuInt operator/(Vector4DuInt lhs, uint8_t rhs) {
      lhs /= rhs;
      return lhs;
    }

    friend Vector4DuInt operator/(uint8_t lhs, Vector4DuInt rhs) {
      rhs /= lhs;
      return rhs;
    }

    // =========================================================================
    // -- Comparison operators -------------------------------------------------
    // =========================================================================

    bool operator==(const Vector4DuInt &rhs) const {
      return (x == rhs.x) && (y == rhs.y) && (z == rhs.z) && (w == rhs.w);
    }

    bool operator!=(const Vector4DuInt &rhs) const {
      return !(*this == rhs);
    }


    // =========================================================================
    /// @todo The following is copy-pasted from MSGPACK_DEFINE_ARRAY.
    /// This is a workaround for an issue in msgpack library. The
    /// MSGPACK_DEFINE_ARRAY macro is shadowing our `z` variable.
    /// https://github.com/msgpack/msgpack-c/issues/709
    // =========================================================================
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        clmdep_msgpack::type::make_define_array(x, y, z, w).msgpack_pack(pk);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o)
    {
        clmdep_msgpack::type::make_define_array(x, y, z, w).msgpack_unpack(o);
    }
    template <typename MSGPACK_OBJECT>
    void msgpack_object(MSGPACK_OBJECT* o, clmdep_msgpack::zone& sneaky_variable_that_shadows_z) const
    {
        clmdep_msgpack::type::make_define_array(x, y, z, w).msgpack_object(o, sneaky_variable_that_shadows_z);
    }
    // =========================================================================
  };

} // namespace geom
} // namespace carla
