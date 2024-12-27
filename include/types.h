#ifndef LIBVECQUAT_TYPES_H__
#define LIBVECQUAT_TYPES_H__

namespace vecquat {

using Number = float;

struct Vector {
  Number x() const {
    return components_[0];
  }
  Number y() const {
    return components_[1];
  }
  Number z() const {
    return components_[2];
  }
  Vector& set_x(Number const value) {
    components_[0] = value;
    return *this;
  }
  Vector& set_y(Number const value) {
    components_[1] = value;
    return *this;
  }
  Vector& set_z(Number const value) {
    components_[2] = value;
    return *this;
  }
  Number operator[](int i) const {
    return components_[i];
  }
  Number& operator[](int i) {
    return components_[i];
  }
  Vector(Number x, Number y, Number z) : components_{x, y, z} {}
  Vector(Vector const& v) : components_{v.components_[0], v.components_[1], v.components_[2]} {}

private:
  Number components_[3] = {0, 0, 0};
};

inline Number operator*(Vector const& v1, Vector const& v2) {
  return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}

inline Vector operator+(Vector const& v1, Vector const& v2) {
  return Vector(v1.x() + v2.x(), v1.y() + v2.y(), v1.z() + v2.z());
}

struct Quaternion {};

} // namespace vecquat

#endif
