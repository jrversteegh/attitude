#include <iostream>
#include <random>

#include <benchmark/benchmark.h>

#include "attitude/types.h"

using namespace attitude;

auto r() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<Number> dis(0.0, 1.0);
  return dis(gen);
}

Quaternion get_random_quaternion() {
  return Quaternion{r(), r(), r(), r()};
}

Matrix3 get_random_matrix() {
  return Matrix3(r(), r(), r(), r(), r(), r(), r(), r(), r());
}

RotationMatrix get_random_rotation_matrix() {
  return RotationMatrix(r(), r(), r(), r(), r(), r());
}

static void benchmark_method_mul(benchmark::State& state) {
  Quaternion q1 = get_random_quaternion();
  Quaternion q2 = get_random_quaternion();
  benchmark::DoNotOptimize(q1);
  benchmark::DoNotOptimize(q2);
  for (auto _ : state) {
    auto value = q1.mul(q2);
    benchmark::DoNotOptimize(value);
  }
}

static void benchmark_operator_mul(benchmark::State& state) {
  Quaternion q1 = get_random_quaternion();
  Quaternion q2 = get_random_quaternion();
  benchmark::DoNotOptimize(q1);
  benchmark::DoNotOptimize(q2);
  for (auto _ : state) {
    auto value = q1 * q2;
    benchmark::DoNotOptimize(value);
  }
}

static void benchmark_matrix_mul(benchmark::State& state) {
  Matrix3 m1 = get_random_matrix();
  Matrix3 m2 = get_random_matrix();
  benchmark::DoNotOptimize(m1);
  benchmark::DoNotOptimize(m2);
  for (auto _ : state) {
    auto value = m1 * m2;
    benchmark::DoNotOptimize(value);
  }
}

static void benchmark_rotation_matrix_mul(benchmark::State& state) {
  RotationMatrix m1 = get_random_rotation_matrix();
  RotationMatrix m2 = get_random_rotation_matrix();
  benchmark::DoNotOptimize(m1);
  benchmark::DoNotOptimize(m2);
  for (auto _ : state) {
    auto value = m1 * m2;
    benchmark::DoNotOptimize(value);
  }
}

BENCHMARK(benchmark_method_mul);
BENCHMARK(benchmark_operator_mul);
BENCHMARK(benchmark_matrix_mul);
BENCHMARK(benchmark_rotation_matrix_mul);
BENCHMARK_MAIN();
