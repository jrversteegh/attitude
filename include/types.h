#ifndef LIBVECQUAT_TYPES_H__
#define LIBVECQUAT_TYPES_H__


namespace vecquat {

using Number = float;

struct Vector {
  Number x() const { return components_[0]; }
  Number y() const { return components_[1]; }
  Number z() const { return components_[2]; }
  Number operator[](int i) const { return components_[i]; }
private:
  Number components_[3] = {};
};

struct Quaternion {
};

} // namespace vecquat




#endif
