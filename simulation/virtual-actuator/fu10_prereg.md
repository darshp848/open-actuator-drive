# Follow-up 10: faster reference plus 1 kHz joint velocity, fixed before running (2 Nov 2026)

Question (from the 23 Oct revision and Follow-up 9): does the sim-reference route (R+H3) recover against the
compact-model route (A2+S) on the arm push task when the reference is fast enough for contact (5-10 ms) and the
policy sees joint velocities estimated at 1 kHz instead of 20 ms differences?

Everything is as in `arm_prereg.md` (task, ES settings, seeds 0-7, staged 300+300+300 generations, randomisation
ranges, deployment plant sets, two-level bootstrap) except:

- **Joint velocity sensing (both routes, training and deployment):** joint velocity is a first-order filtered
  finite difference updated every 1 ms, `w = a*w + (1-a)*(q_k - q_{k-1})/1 ms`, `a = exp(-2*pi*50 Hz*1 ms)`
  (50 Hz cutoff). Training uses exact joint angles (as before); deployment quantizes them to 2*pi/16384 before
  differencing (as before). Puck position and velocity sensing are unchanged (1 mm noise, 20 ms differences).
- **Reference routes:** T = 10 ms (primary) and T = 5 ms (secondary), three stages each, deployed with the same
  H3 and E3 gains and `T_REF_S` set to the route's T.
- **Compact route:** retrained under the new sensing, warm-started from the T = 10 ms reference stage-1 policies,
  300 nominal + 300 randomised generations (equal 900-generation budget).
- **Primary test:** R10+H3 minus A2+S success on the 40 new plants (10 held-out + 30 new), non-inferiority margin
  -5 points, two-level bootstrap (seeds and plants), 5000 draws, 95 %. Pass = lower bound above -5.
- **Secondary (reported, not gated):** R5+H3 minus A2+S; R10+H3 minus R10+E3; out-of-range differences; each
  route's training success on its own randomised model; comparison with the Follow-up 9 numbers (R30+H3 55 %,
  R15+H3 75 %, A2+S 93 %) to separate the effect of reference speed from the velocity-sensing change.

Prediction written before running: the gap narrows but R10+H3 does not reach non-inferiority (Follow-up 9 showed the
reference route learns worse; 15 ms recovered half the gap). A pass would support Idea 1's premise; a fail means the
sentence's "match the simulator" route needs more than a faster reference.

## Amendment, 2 Nov 2026 04:20 CST, before any Follow-up 10 result was seen
Darsh needs the laptop free by 7:45 AM. The T = 5 ms secondary route is dropped (deferred). To fit, after the shared
T = 10 ms stage 1 the reference and compact stages 2 and 3 run concurrently on the GPU (`fu10_fast.sh`); seeds,
generations, warm starts, sensing and the primary test are unchanged. A watchdog stops all jobs at 7:38 AM; if
training or deployment is cut short, no primary-test result is reported from partial data.
