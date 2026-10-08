#ifndef MODELICA_MODELS_NATIVE_INNOVATION_DUMP_H
#define MODELICA_MODELS_NATIVE_INNOVATION_DUMP_H

#include <cstdio>
#include <cstdlib>
#include <cstdint>

inline void writeNativeInnovation(std::uint64_t publication_us,
    std::uint64_t fusion_us, std::uint64_t sample_us, const char *sensor,
    unsigned axis, const char *stage, double innovation, double variance,
    double observation_variance, double gate_ratio, int fused, bool navigation)
{
    static FILE *stream = []() -> FILE * {
        const char *path = std::getenv("NATIVE_INNOVATION_PATH");
        if (!path) return nullptr;
        FILE *file = std::fopen(path, "w");
        if (!file) { std::perror(path); std::abort(); }
        std::fputs("publication_us,fusion_us,sample_us,sensor,axis,stage,innovation,"
            "innovation_variance,observation_variance,gate_ratio,fused,navigation\n", file);
        return file;
    }();
    if (stream && std::fprintf(stream,
        "%llu,%llu,%llu,%s,%u,%s,%.17g,%.17g,%.17g,%.17g,%d,%d\n",
        static_cast<unsigned long long>(publication_us),
        static_cast<unsigned long long>(fusion_us),
        static_cast<unsigned long long>(sample_us), sensor, axis, stage,
        innovation, variance, observation_variance, gate_ratio, fused,
        static_cast<int>(navigation)) < 0) {
        std::perror("native innovation observer");
        std::abort();
    }
}

#endif
