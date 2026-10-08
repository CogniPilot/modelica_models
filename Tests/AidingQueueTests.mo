within Tests;

model AidingQueueTests
  "A ripe arrival at an empty release is delivered once without a queue write"
  function checkQueue
    import Estimation.FusionHorizon.*;
    output Boolean passed;
  protected
    Real queue[2, 2];
    Real stored[2];
    Real deliveredRow[2];
    Real age;
    Integer slot;
    Integer head;
    Integer tail;
    Integer count;
    Integer arrival;
    Integer delivery;
    Boolean delivered;
  algorithm
    queue := zeros(2, 2);
    (slot, stored, head, tail, count, deliveredRow, age, delivered,
     arrival, delivery) := stepQueue(false, queue, 1, 1, 0, true,
      {1.0, 42.0}, false, true, 1.005, 0.01, 1.0e-7);
    assert(not delivered and slot == 1 and count == 1 and arrival == AidingQueued,
      "A non-release arrival was lost instead of queued");
    (slot, stored, head, tail, count, deliveredRow, age, delivered,
     arrival, delivery) := stepQueue(false, queue, 1, 1, 0, true,
      {1.0, 42.0}, true, true, 1.005, 0.01, 1.0e-7);
    assert(delivered and slot == 0 and head == 1 and tail == 1 and count == 0,
      "Direct delivery changed the FIFO or scheduled a second delivery");
    assert(deliveredRow[2] == 42.0 and abs(age - 0.005) < 1.0e-12,
      "Direct delivery read a queue slot instead of the arrival payload");
    assert(arrival == AidingDeliveredOnArrival
      and delivery == AidingDeliveredAtHorizon,
      "Direct delivery did not report its admission and delivery");
    (slot, stored, head, tail, count, deliveredRow, age, delivered,
     arrival, delivery) := stepQueue(false, queue, 1, 1, 0, true,
      {1.02, 99.0}, true, true, 1.005, 0.01, 1.0e-7);
    assert(not delivered and count == 1, "A future measurement bypassed the horizon");
    (slot, stored, head, tail, count, deliveredRow, age, delivered,
     arrival, delivery) := stepQueue(false, queue, 1, 1, 0, true,
      {0.9, 99.0}, true, true, 1.005, 0.01, 1.0e-7);
    assert(not delivered and arrival == AidingRefusedLate and count == 0,
      "Direct delivery bypassed the late-packet bound");
    queue := [1.0, 42.0; 1.01, 43.0];
    (slot, stored, head, tail, count, deliveredRow, age, delivered,
     arrival, delivery) := stepQueue(false, queue, 1, 1, 2, true,
      {1.005, 99.0}, true, true, 1.005, 0.01, 1.0e-7);
    assert(delivered and deliveredRow[2] == 42 and count == 2 and slot == 1,
      "A ripe arrival displaced the FIFO's older delivery");
    (slot, stored, head, tail, count, deliveredRow, age, delivered,
     arrival, delivery) := stepQueue(true, queue, 1, 1, 0, true,
      {1.0, 42.0}, true, true, 1.005, 0.01, 1.0e-7);
    assert(not delivered and count == 0 and slot == 0
      and arrival == AidingNoArrival, "A reset delivered a pre-reset arrival");
    passed := true;
  end checkQueue;
initial algorithm
  assert(checkQueue(), "Aiding queue regression failed");
end AidingQueueTests;
