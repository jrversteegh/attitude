#include <random>

#include <benchmark/benchmark.h>

#include "attitude/types.h"

using namespace attitude;

Quaternion get_random_quaternion() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<Number> dis(0.0, 1.0);
  return Quaternion{dis(gen), dis(gen), dis(gen), dis(gen)};
}

static void benchmark_method_mul(benchmark::State& state) {
  Quaternion q1 = get_random_quaternion();
  Quaternion q2 = get_random_quaternion();
  for (auto _: state) {
    auto value = q1.mul(q2);
    benchmark::DoNotOptimize(value);
  }
}

static void benchmark_operator_mul(benchmark::State& state) {
  Quaternion q1 = get_random_quaternion();
  Quaternion q2 = get_random_quaternion();
  for (auto _: state) {
    auto value = q1 * q2;
    benchmark::DoNotOptimize(value);
  }
}

BENCHMARK(benchmark_method_mul);
BENCHMARK(benchmark_operator_mul);
BENCHMARK_MAIN();
