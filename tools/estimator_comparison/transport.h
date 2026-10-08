#ifndef ESTIMATOR_COMPARISON_TRANSPORT_H
#define ESTIMATOR_COMPARISON_TRANSPORT_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum { GPS_SOURCE, FLOW_SOURCE, MAG_SOURCE, BARO_SOURCE, SOURCE_COUNT };
enum { TRANSPORT_CAPACITY = 128 };

typedef struct {
  double release_s;
  double row[25];
} TransportPacket;

typedef struct {
  TransportPacket packets[TRANSPORT_CAPACITY];
  double held[25];
  double last_stamp_s;
  double last_release_s;
  unsigned head, count;
  bool valid, fresh;
  uint64_t delivered, delivered_digest;
} TransportQueue;

typedef struct {
  TransportQueue queues[SOURCE_COUNT];
  double delay_s[SOURCE_COUNT];
  double jitter_s;
  uint32_t random_state[SOURCE_COUNT];
} SensorTransport;

static void transport_startup(SensorTransport *transport,
                              const double delay_s[4], double jitter_s,
                              uint32_t seed) {
  memset(transport, 0, sizeof(*transport));
  transport->jitter_s = jitter_s;
  for (unsigned source = 0; source < SOURCE_COUNT; ++source) {
    transport->delay_s[source] = delay_s[source];
    transport->queues[source].last_stamp_s = -1e30;
    transport->queues[source].delivered_digest = UINT64_C(14695981039346656037);
    transport->random_state[source] = seed + 7919u * (source + 1);
  }
}

static bool transport_step(SensorTransport *transport, const double row[25],
                           const char *scenario) {
  const unsigned stamps[SOURCE_COUNT] = {8, 15, 20, 20};
  const bool gps_available =
      strcmp(scenario, "denied") != 0 &&
      !(strcmp(scenario, "transition") == 0 && row[8] >= 25 && row[8] < 40);
  const bool available[SOURCE_COUNT] = {gps_available && row[7] > .5, true,
                                        row[19] > .5, row[19] > .5};
  for (unsigned source = 0; source < SOURCE_COUNT; ++source) {
    TransportQueue *queue = &transport->queues[source];
    const double stamp = row[stamps[source]];
    queue->fresh = false;
    if (available[source] && stamp >= 0 && stamp > queue->last_stamp_s + 1e-9) {
      if (queue->count == TRANSPORT_CAPACITY)
        return false;
      uint32_t *random = &transport->random_state[source];
      *random = 1664525u * *random + 1013904223u;
      const double jitter = transport->jitter_s * (*random / 4294967296.0);
      const unsigned slot = (queue->head + queue->count) % TRANSPORT_CAPACITY;
      TransportPacket *packet = &queue->packets[slot];
      packet->release_s = fmax(queue->last_release_s,
                               stamp + transport->delay_s[source] + jitter);
      memcpy(packet->row, row, sizeof(packet->row));
      queue->last_stamp_s = stamp;
      queue->last_release_s = packet->release_s;
      ++queue->count;
    }
    if (queue->count &&
        queue->packets[queue->head].release_s <= row[0] + 1e-9) {
      memcpy(queue->held, queue->packets[queue->head].row, sizeof(queue->held));
      const unsigned char *bytes = (const unsigned char *)queue->held;
      for (unsigned byte = 0; byte < sizeof(queue->held); ++byte) {
        queue->delivered_digest ^= bytes[byte];
        queue->delivered_digest *= UINT64_C(1099511628211);
      }
      queue->head = (queue->head + 1) % TRANSPORT_CAPACITY;
      --queue->count;
      queue->valid = queue->fresh = true;
      ++queue->delivered;
    }
  }
  return true;
}

#endif
