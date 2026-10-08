#include "transport.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv) {
  if (argc != 9)
    return 2;
  double delays[4];
  for (unsigned source = 0; source < SOURCE_COUNT; ++source)
    delays[source] = strtod(argv[3 + source], NULL) * .001;
  const double jitter = strtod(argv[7], NULL) * .001;
  SensorTransport transport;
  transport_startup(&transport, delays, jitter, strtoul(argv[8], NULL, 10));
  FILE *input = fopen(argv[1], "r");
  if (!input)
    return 1;
  char line[4096];
  if (!fgets(line, sizeof(line), input))
    return 1;
  puts("arrival_t_s,source,measurement_t_s,gx,gy,gz");
  while (fgets(line, sizeof(line), input)) {
    double row[25];
    char *cursor = line;
    for (unsigned column = 0; column < 25; ++column) {
      char *end;
      row[column] = strtod(cursor, &end);
      if (end == cursor || !isfinite(row[column]) ||
          (column < 24 && *end != ','))
        return 1;
      cursor = end + (column < 24);
    }
    if (!transport_step(&transport, row, argv[2]))
      return 1;
    const unsigned stamps[SOURCE_COUNT] = {8, 15, 20, 20};
    for (unsigned source = 0; source < SOURCE_COUNT; ++source) {
      const TransportQueue *queue = &transport.queues[source];
      if (queue->fresh)
        printf("%.17g,%u,%.17g,%.17g,%.17g,%.17g\n", row[0], source,
               queue->held[stamps[source]], queue->held[1], queue->held[2],
               queue->held[3]);
    }
  }
  fclose(input);
  fprintf(stderr, "{\"transport_delivered\":[");
  for (unsigned source = 0; source < SOURCE_COUNT; ++source)
    fprintf(stderr, "%s%llu", source ? "," : "",
            (unsigned long long)transport.queues[source].delivered);
  fprintf(stderr, "],\"transport_digest\":[");
  for (unsigned source = 0; source < SOURCE_COUNT; ++source)
    fprintf(stderr, "%s\"%016llx\"", source ? "," : "",
            (unsigned long long)transport.queues[source].delivered_digest);
  fprintf(stderr, "]}\n");
  return 0;
}
