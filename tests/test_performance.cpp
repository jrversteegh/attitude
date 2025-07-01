#include <iostream>
#include <random>

#include <benchmark/benchmark.h>

#include "attitude/types.h"

using namespace attitude;

using Q = Quaternion<>;
using M = Matrix<>;
using T = Tensor<>;

auto r() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_real_distribution<Number> dis(0.0, 1.0);
  return dis(gen);
}

Q get_random_quaternion() {
  return Q{r(), r(), r(), r()};
}

M get_random_matrix() {
  return M(r(), r(), r(), r(), r(), r(), r(), r(), r());
}

T get_random_tensor() {
  return T(r(), r(), r(), r(), r(), r());
}

static void benchmark_quaternion_mul(benchmark::State& state) {
  Q q1 = get_random_quaternion();
  Q q2 = get_random_quaternion();
  benchmark::DoNotOptimize(q1);
  benchmark::DoNotOptimize(q2);
  for (auto _ : state) {
    auto value = q1 * q2;
    benchmark::DoNotOptimize(value);
  }
}

static void benchmark_matrix_mul(benchmark::State& state) {
  M m1 = get_random_matrix();
  M m2 = get_random_matrix();
  benchmark::DoNotOptimize(m1);
  benchmark::DoNotOptimize(m2);
  for (auto _ : state) {
    auto value = m1 * m2;
    benchmark::DoNotOptimize(value);
  }
}

static void benchmark_tensor_mul(benchmark::State& state) {
  T t1 = get_random_tensor();
  T t2 = get_random_tensor();
  benchmark::DoNotOptimize(t1);
  benchmark::DoNotOptimize(t2);
  for (auto _ : state) {
    auto value = t1 * t2;
    benchmark::DoNotOptimize(value);
  }
}

BENCHMARK(benchmark_quaternion_mul);
BENCHMARK(benchmark_matrix_mul);
BENCHMARK(benchmark_tensor_mul);
BENCHMARK_MAIN();
